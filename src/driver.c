//	System Includes
#include <CoreAudio/AudioServerPlugIn.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <stdint.h>
#include <os/log.h>
#include <stdatomic.h>

#pragma mark Macros

#if TARGET_RT_BIG_ENDIAN
	#define	FourCCToCString(the4CC)	{ ((char*)&the4CC)[0], ((char*)&the4CC)[1], ((char*)&the4CC)[2], ((char*)&the4CC)[3], 0 }
#else
	#define	FourCCToCString(the4CC)	{ ((char*)&the4CC)[3], ((char*)&the4CC)[2], ((char*)&the4CC)[1], ((char*)&the4CC)[0], 0 }
#endif

static os_log_t global_log_handle = NULL;

#if DEBUG
#define	DebugMsg(fmt, ...) os_log(global_log_handle, fmt, ##__VA_ARGS__)
#else
#define DebugMsg(fmt, ...) os_log(global_log_handle, fmt, ##__VA_ARGS__)
#endif

#define DoIfFailed(inCondition, inAction, inMessage)				\
	if (inCondition)												\
	{																\
		DebugMsg(inMessage);										\
		{ inAction; }												\
	}																\

#define N_CHANNELS 16
#define SAMPLE_RATE 48000.0

// A volume and mute control pair for each channel + a master pair
static UInt32 const N_CONTROLS = 2 * (N_CHANNELS + 1);
// N_CONTROLS + the output stream
static UInt32 const N_DEVICE_OWNED_OBJS = N_CONTROLS + 1;

// The _only_ format we support (Note that it is interleaved by default)
static AudioStreamRangedDescription const FORMAT = {

	.mFormat = {
		.mSampleRate = SAMPLE_RATE,
		.mFormatID = kAudioFormatLinearPCM,
		.mFormatFlags = kAudioFormatFlagsNativeFloatPacked,
		.mBytesPerPacket = sizeof(Float32) * N_CHANNELS,
		.mFramesPerPacket = 1,
		.mBytesPerFrame = sizeof(Float32) * N_CHANNELS,
		.mChannelsPerFrame = N_CHANNELS,
		.mBitsPerChannel = sizeof(Float32) * 8,
		.mReserved = 0
	},

	.mSampleRateRange = {
		.mMaximum = SAMPLE_RATE,
		.mMinimum = SAMPLE_RATE
	}
};

// The Audio Object IDs we expose to the HAL

static AudioObjectID const PLUGIN_ID = kAudioObjectPlugInObject; /* AKA 1 */
static AudioObjectID const DEVICE_ID = 2;
static AudioObjectID const STREAM_ID = 3;

// We also expose IDs 4 through (3 + N_CONTROLS). Those represent the volume/mute controls
// Note that 0 is a sentinel/unknown value and usually represents the host/HAL itself

static AudioObjectID const FIRST_CTRL_ID = 4;
static AudioObjectID const LAST_CTRL_ID = 3 + N_CONTROLS;

// Blueprint of our driver's object tree
//  - the plug-in (ID = 1)
//		- a device (ID = 2)
//			- a single output stream (ID = 3)
//				- N_CHANNELS channels of interleaved 32 bit Float32 samples at SAMPLE_RATE Hz
//				- Data written to it is immediately sent over the network
//				- A master mute control (ID = 4)
//				- A master volume control (ID = 5)
//				Plus, for each channel: (1 <= N <= N_CHANNELS)
//					- A mute control (ID = 4 + 2 * N) (all even)
//					- A volume control (ID = 5 + 2 * N) (all odd)

static inline bool audio_object_is_control(AudioObjectID const id) {
	// All our controls' IDs are consecutive...
	return FIRST_CTRL_ID <= id && id <= LAST_CTRL_ID;
}

// Assumes that `audio_object_is_control(id)` holds. Otherwise, results are inconsistent
static inline bool control_is_volume(AudioObjectID const id) {
	return (id & 1) != 0;
}

// Assumes that `audio_object_is_control(id)` holds. It is UB otherwise.
static inline UInt32 control_channel_index(AudioObjectID const id) {
	return (id - FIRST_CTRL_ID) / 2;
}

static inline Float32 sq(Float32 const x) {
	return x * x;
}

static inline Float32 db_to_gain(Float32 const db) {
	// huh... no exp10?
	return powf(10.0f, db * 0.05f);
}

static inline Float32 gain_to_db(Float32 const gain) {
	return 10.0f * log10(sq(gain));
}

static inline Float32 linear_remap(
	Float32 const x,
	Float32 const x_start,
	Float32 const x_len,
	Float32 const y_start,
	Float32 const y_len
) {
	// FP equality check sry lol
	if (x_len == 0.0f) return x_start;
	return fmaf(x - x_start, y_len / x_len, y_start);
}

// Increment an atomic counter, saturating at numerical limits
static inline Boolean saturating_refount_inc(_Atomic(UInt32)* const i) {
    UInt32 old = atomic_load_explicit(i, __ATOMIC_RELAXED);
	UInt32 new;
    do {
        if (old == UINT32_MAX) return false;
		new = old + 1;
    } while (!atomic_compare_exchange_weak_explicit(i, &old, new, __ATOMIC_RELAXED, __ATOMIC_RELAXED));
    return true;
}

// Decrement an atomic counter, saturating at numerical limits
static inline Boolean saturating_refcount_dec(_Atomic(UInt32)* const i) {
	UInt32 old = atomic_load_explicit(i, __ATOMIC_RELAXED);
	UInt32 new;
	do {
		if (old == 0) return false;
		new = old - 1;
	} while (!atomic_compare_exchange_weak_explicit(i, &old, new, __ATOMIC_RELEASE, __ATOMIC_RELAXED));
	return true;
}

// forward declarations xD
static HRESULT		syfala_query_interface(void* inDriver, REFIID inUUID, LPVOID* outInterface);
static ULONG		syfala_add_ref(void* inDriver);
static ULONG		syfala_release(void* inDriver);
static OSStatus		syfala_initialize(AudioServerPlugInDriverRef inDriver, AudioServerPlugInHostRef inHost);
static OSStatus		syfala_create_device(AudioServerPlugInDriverRef inDriver, CFDictionaryRef inDescription, const AudioServerPlugInClientInfo* inClientInfo, AudioObjectID* outDeviceObjectID);
static OSStatus		syfala_destroy_device(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID);
static OSStatus		syfala_add_device_client(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, const AudioServerPlugInClientInfo* inClientInfo);
static OSStatus		syfala_remove_device_client(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, const AudioServerPlugInClientInfo* inClientInfo);
static OSStatus		syfala_perform_device_configuration_change(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt64 inChangeAction, void* inChangeInfo);
static OSStatus		syfala_AbortDeviceConfigurationChange(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt64 inChangeAction, void* inChangeInfo);
static Boolean		syfala_HasProperty(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress);
static OSStatus		syfala_IsPropertySettable(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, Boolean* outIsSettable);
static OSStatus		syfala_GetPropertyDataSize(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32* outDataSize);
static OSStatus		syfala_GetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32 inDataSize, UInt32* outDataSize, void* outData);
static OSStatus		syfala_SetPropertyData(AudioServerPlugInDriverRef inDriver, AudioObjectID inObjectID, pid_t inClientProcessID, const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32 inDataSize, const void* inData);
static OSStatus		syfala_StartIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID);
static OSStatus		syfala_StopIO(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID);
static OSStatus		syfala_GetZeroTimeStamp(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, Float64* outSampleTime, UInt64* outHostTime, UInt64* outSeed);
static OSStatus		syfala_WillDoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, Boolean* outWillDo, Boolean* outWillDoInPlace);
static OSStatus		syfala_BeginIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo);
static OSStatus		syfala_DoIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, AudioObjectID inStreamObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo, void* ioMainBuffer, void* ioSecondaryBuffer);
static OSStatus		syfala_EndIOOperation(AudioServerPlugInDriverRef inDriver, AudioObjectID inDeviceObjectID, UInt32 inClientID, UInt32 inOperationID, UInt32 inIOBufferFrameSize, const AudioServerPlugInIOCycleInfo* inIOCycleInfo);

static Boolean		syfala_HasPlugInProperty(const AudioObjectPropertyAddress* inAddress);
static OSStatus		syfala_IsPlugInPropertySettable(const AudioObjectPropertyAddress* inAddress, Boolean* outIsSettable);
static OSStatus		syfala_GetPlugInPropertyDataSize(const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32* outDataSize);
static OSStatus		syfala_GetPlugInPropertyData(const AudioObjectPropertyAddress* inAddress, UInt32 inQualifierDataSize, const void* inQualifierData, UInt32 inDataSize, UInt32* outDataSize, void* outData);

static Boolean		syfala_HasDeviceProperty(const AudioObjectPropertyAddress* inAddress);
static OSStatus		syfala_IsDevicePropertySettable(const AudioObjectPropertyAddress* inAddress, Boolean* outIsSettable);
static OSStatus		syfala_GetDevicePropertyDataSize(const AudioObjectPropertyAddress* inAddress, UInt32* outDataSize);
static OSStatus		syfala_GetDevicePropertyData(const AudioObjectPropertyAddress* inAddress, UInt32 inDataSize, UInt32* outDataSize, void* outData);

static Boolean		syfala_HasStreamProperty(const AudioObjectPropertyAddress* inAddress);
static OSStatus		syfala_IsStreamPropertySettable(const AudioObjectPropertyAddress* inAddress, Boolean* outIsSettable);
static OSStatus		syfala_GetStreamPropertyDataSize(const AudioObjectPropertyAddress* inAddress, UInt32* outDataSize);
static OSStatus		syfala_GetStreamPropertyData(const AudioObjectPropertyAddress* inAddress, UInt32 inDataSize, UInt32* outDataSize, void* outData);

static Boolean		syfala_HasControlProperty(AudioObjectID inObjectID, const AudioObjectPropertyAddress* inAddress);
static OSStatus		syfala_IsControlPropertySettable(AudioObjectID inObjectID, const AudioObjectPropertyAddress* inAddress, Boolean* outIsSettable);
static OSStatus		syfala_GetControlPropertyDataSize(AudioObjectID inObjectID, const AudioObjectPropertyAddress* inAddress, UInt32* outDataSize);
static OSStatus		syfala_GetControlPropertyData(AudioObjectID inObjectID, const AudioObjectPropertyAddress* inAddress, UInt32 inDataSize, UInt32* outDataSize, void* outData);
static OSStatus		syfala_SetControlPropertyData(AudioObjectID inObjectID, const AudioObjectPropertyAddress* inAddress, UInt32 inDataSize, const void* inData, UInt32* outNumberPropertiesChanged, AudioObjectPropertyAddress outChangedAddresses[2]);

#pragma mark State

static AudioServerPlugInHostRef HOST = NULL;

// All of the plugin's methods are exposed through this struct, which we return a reference to in the factory function
static AudioServerPlugInDriverInterface	DRIVER_INTERFACE = {
	._reserved = NULL,
	.QueryInterface = syfala_query_interface,
	.AddRef = syfala_add_ref,
	.Release = syfala_release,
	.Initialize = syfala_initialize,
	.CreateDevice = syfala_create_device,
	.DestroyDevice = syfala_destroy_device,
	.AddDeviceClient = syfala_add_device_client,
	.RemoveDeviceClient = syfala_remove_device_client,
	.PerformDeviceConfigurationChange = syfala_perform_device_configuration_change,
	.AbortDeviceConfigurationChange = syfala_AbortDeviceConfigurationChange,
	.HasProperty = syfala_HasProperty,
	.IsPropertySettable = syfala_IsPropertySettable,
	.GetPropertyDataSize = syfala_GetPropertyDataSize,
	.GetPropertyData = syfala_GetPropertyData,
	.SetPropertyData = syfala_SetPropertyData,
	.StartIO = syfala_StartIO,
	.StopIO = syfala_StopIO,
	.GetZeroTimeStamp = syfala_GetZeroTimeStamp,
	.WillDoIOOperation = syfala_WillDoIOOperation,
	.BeginIOOperation = syfala_BeginIOOperation,
	.DoIOOperation = syfala_DoIOOperation,
	.EndIOOperation = syfala_EndIOOperation
};

static AudioServerPlugInDriverInterface*	DRIVER_OBJECT					= &DRIVER_INTERFACE;
static AudioServerPlugInDriverRef			DRIVER_REF						= &DRIVER_OBJECT;

#define										kPlugIn_BundleID				"com.emeraude.syfala_coreaudio"

static _Atomic(UInt32)						PLUGIN_REF_COUNT				= 0;
static _Atomic(bool)						IS_OUTPUT_STREAM_ACTIVE			= true;

#define                                     kDevice_UID                     "syfala_UID"

#define                                     kDevice_ModelUID                "syfala_ModelUID"

static UInt32 const							kDevice_RingBufferSize			= 16384;

static pthread_mutex_t						gDevice_IOMutex					= PTHREAD_MUTEX_INITIALIZER;

static UInt32								gDevice_IOIsRunning				= 0;
static Float64								gDevice_HostTicksPerFrame		= 0.0;
static UInt64								gDevice_NumberTimeStamps		= 0;
static Float64								gDevice_AnchorSampleTime		= 0.0;
static UInt64								gDevice_AnchorHostTime			= 0;

static Float32 const						VOL_MIN_DB				     	= -80.0f;
static Float32 const						VOL_MAX_DB				     	= 0.0f;
static Float32 const 						VOL_DB_RANGE					= VOL_MAX_DB - VOL_MIN_DB;

static _Atomic(Float32)                     OUTPUT_VOLUME[N_CHANNELS + 1]   = {};
static _Atomic(bool)                        OUTPUT_MUTE[N_CHANNELS + 1]     = {};

//==================================================================================================
#pragma mark -
#pragma mark AudioServerPlugInDriverInterface Implementation
//==================================================================================================

#pragma mark The Interface


#pragma mark Factory


// This is the only function exposed by the library, It's name must be indicated in the CFPlugInFactories field
// the bundle's Info.plist file

void* syfala_create(
	CFAllocatorRef const inAllocator,
	CFUUIDRef const inRequestedTypeUUID
) {
	//	This is the CFPlugIn factory function. Its job is to create the implementation for the given
	//	type provided that the type is supported. Because this driver is simple and all its
	//	initialization is handled via static iniitalization when the bundle is loaded, all that
	//	needs to be done is to return the AudioServerPlugInDriverRef that points to the driver's
	//	interface. A more complicated driver would create any base line objects it needs to satisfy
	//	the IUnknown methods that are used to discover that actual interface to talk to the driver.
	//	The majority of the driver's initilization should be handled in the Initialize() method of
	//	the driver's AudioServerPlugInDriverInterface.
	
	#pragma unused(inAllocator)

	global_log_handle = os_log_create("com.emeraude.syfala_coreaudio", "driver");
	
	for (UInt32 i = 1; i <= N_CHANNELS; ++i) OUTPUT_VOLUME[i] = 1.0f;
	OUTPUT_VOLUME[0] = 0.001f;
	OUTPUT_MUTE[0] = true;
	
	DebugMsg("Factory");

    void* theAnswer = NULL;
    if(CFEqual(inRequestedTypeUUID, kAudioServerPlugInTypeUUID)) {
		DebugMsg("HAL Plug-In Interface Requested");
		theAnswer = DRIVER_REF;
		
    }
    return theAnswer;
}

#pragma mark Inheritence

static HRESULT syfala_query_interface(
	void* const inDriver,
	REFIID const inUUID,
	LPVOID* const outInterface
) {
	//	This function is called by the HAL to get the interface to talk to the plug-in through.
	//	AudioServerPlugIns are required to support the IUnknown interface and the
	//	AudioServerPlugInDriverInterface. As it happens, all interfaces must also provide the
	//	IUnknown interface, so we can always just return the single interface we made with
	//	DRIVER_OBJECT regardless of which one is asked for.

	DebugMsg("QueryInterface");

	CFUUIDBytes const IUnknown_uuid_bytes = CFUUIDGetUUIDBytes(IUnknownUUID);
	CFUUIDBytes const AudioServerPlugInDriverInterface_uuid_bytes =
		CFUUIDGetUUIDBytes(kAudioServerPlugInDriverInterfaceUUID);

	//	check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_query_interface: bad driver reference");
	DoIfFailed(outInterface == NULL, return kAudioHardwareIllegalOperationError, "syfala_query_interface: no place to store the returned interface");

	//	AudioServerPlugIns only support two interfaces, IUnknown (which has to be supported by all
	//	CFPlugIns and AudioServerPlugInDriverInterface (which is the actual interface the HAL will
	//	use).
	if(
		memcmp(&inUUID, &IUnknown_uuid_bytes, sizeof(CFUUIDBytes)) == 0 ||
		memcmp(&inUUID, &AudioServerPlugInDriverInterface_uuid_bytes, sizeof(CFUUIDBytes)) == 0
	) {
		atomic_fetch_add_explicit(&PLUGIN_REF_COUNT, 1, __ATOMIC_RELAXED);
		*outInterface = DRIVER_REF;
	} else {
		return E_NOINTERFACE;
	}
	
	return kAudioHardwareNoError;
}

static ULONG syfala_add_ref(
	void* const inDriver
) {
	//	This call returns the resulting reference count after the increment.

	DebugMsg("AddRef");
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return 0, "syfala_add_ref: bad driver reference");

	//	increment the refcount, return the new value
	return atomic_fetch_add_explicit(&PLUGIN_REF_COUNT, 1, __ATOMIC_RELAXED) + 1;
}

static ULONG syfala_release(
	void* const inDriver
) {
	//	This call returns the resulting reference count after the decrement.
	DebugMsg("Release");
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return 0, "syfala_Release: bad driver reference");
	
	// decrement the refcount, return the new value
	return atomic_fetch_sub_explicit(&PLUGIN_REF_COUNT, 1, __ATOMIC_RELAXED) - 1;
}

#pragma mark Basic Operations

static OSStatus	syfala_initialize(
	AudioServerPlugInDriverRef const inDriver,
	AudioServerPlugInHostRef const inHost
) {
	//	The job of this method is, as the name implies, to get the driver initialized. Note that when this call returns, the HAL will scan the various lists the driver
	//	maintains (such as the device list) to get the inital set of objects the driver is
	//	publishing. So, there is no need to notifiy the HAL about any objects created as part of the
	//	execution of this method.

	DebugMsg("Initialize");
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_initialize: bad driver reference");
	
	// One specific thing that needs to be done is to store the AudioServerPlugInHostRef
	// so that it can be used later. This is the object used by our plugin to notify the host
	// of property changes etc.
	HOST = inHost;

	//	calculate the host ticks per frame
	struct mach_timebase_info theTimeBaseInfo;
	mach_timebase_info(&theTimeBaseInfo);
	Float64 host_clock_freq = (Float64)theTimeBaseInfo.denom / (Float64)theTimeBaseInfo.numer;
	host_clock_freq *= 1000000000.0;
	pthread_mutex_lock(&gDevice_IOMutex);
	gDevice_HostTicksPerFrame = host_clock_freq / SAMPLE_RATE;
	pthread_mutex_unlock(&gDevice_IOMutex);

	return kAudioHardwareNoError;
}

static OSStatus	syfala_create_device(
	AudioServerPlugInDriverRef const inDriver,
	CFDictionaryRef const inDescription,
	AudioServerPlugInClientInfo const* const inClientInfo,
	AudioObjectID* const outDeviceObjectIO
) {
	//	This method is used to tell a driver that implements the Transport Manager semantics to
	//	create an AudioEndpointDevice from a set of AudioEndpoints. Since this driver is not a
	//	Transport Manager, we just check the arguments and return
	//	kAudioHardwareUnsupportedOperationError.

	DebugMsg("CreateDevice");

	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_create_device: bad driver reference");

	return kAudioHardwareUnsupportedOperationError;
}

static OSStatus	syfala_destroy_device(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID
) {
	//	This method is used to tell a driver that implements the Transport Manager semantics to
	//	destroy an AudioEndpointDevice. Since this driver is not a Transport Manager, we just check
	//	the arguments and return kAudioHardwareUnsupportedOperationError.

	#pragma unused(inDeviceObjectID)

	DebugMsg("DestroyDevice");
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_destroy_device: bad driver reference");

	return kAudioHardwareUnsupportedOperationError;
}

static OSStatus	syfala_add_device_client(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	AudioServerPlugInClientInfo const* const inClientInfo
) {
	//	This method is used to inform the driver about a new client that is using the given device.
	//	This allows the device to act differently depending on who the client is. This driver does
	//	not need to track the clients using the device, so we just check the arguments and return
	//	successfully.

	DebugMsg("AddDeviceClient");
	
	#pragma unused(inClientInfo)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_add_device_client: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_add_device_client: bad device ID");

	return kAudioHardwareNoError;
}

static OSStatus	syfala_remove_device_client(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	AudioServerPlugInClientInfo const* const inClientInfo
) {
	//	This method is used to inform the driver about a client that is no longer using the given
	//	device. This driver does not track clients, so we just check the arguments and return
	//	successfully.

	DebugMsg("RemoveDeviceClient");
	
	#pragma unused(inClientInfo)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_remove_device_client: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_remove_device_client: bad device ID");

	return kAudioHardwareNoError;
}

static OSStatus	syfala_perform_device_configuration_change(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	UInt64 const inChangeAction,
	void* const inChangeInfo
) {
	// This method is called to tell the device that it can perform the configuation change that
	// it had requested via a call to the host method, RequestDeviceConfigurationChange(). The
	// arguments, inChangeAction and inChangeInfo are the same as what was passed to
	// RequestDeviceConfigurationChange().
	//
	// The HAL guarantees that IO will be stopped while this method is in progress. The HAL will
	// also handle figuring out exactly what changed for the non-control related properties. This
	// means that the only notifications that would need to be sent here would be for either
	// custom properties the HAL doesn't know about or for controls.
	//
	// For the device implemented by this driver, only sample rate changes go through this process
	// as it is the only state that can be changed for the device that isn't a control. For this
	// change, the new sample rate is passed in the inChangeAction argument.

	DebugMsg("PerformDeviceConfigurationChange");
	
	#pragma unused(inChangeInfo)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_perform_device_configuration_change: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_perform_device_configuration_change: bad device ID");
	
	return kAudioHardwareNoError;
}

static OSStatus	syfala_AbortDeviceConfigurationChange(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	UInt64 const inChangeAction,
	void* const inChangeInfo
) {
	//	This method is called to tell the driver that a request for a config change has been denied.
	//	This provides the driver an opportunity to clean up any state associated with the request.
	//	For this driver, an aborted config change requires no action. So we just check the arguments
	//	and return

	DebugMsg("AbortDeviceConfigurationChange");

	#pragma unused(inChangeAction, inChangeInfo)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_perform_device_configuration_change: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_perform_device_configuration_change: bad device ID");

	return kAudioHardwareNoError;
}

#pragma mark Property Operations

static Boolean	syfala_HasProperty(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inObjectID,
	pid_t const inClientProcessID,
	AudioObjectPropertyAddress const* const inAddress
) {
	char const selector[5] = FourCCToCString(inAddress->mSelector);
    char const scope[5] = FourCCToCString(inAddress->mScope);

    DebugMsg(
		"Has       : %02u { %s %s %02d } by %08x",
		inObjectID,
		selector,
		scope,
		inAddress->mElement,
		inClientProcessID
	);
		
	//	check args
	DoIfFailed(inDriver != DRIVER_REF, return false, "syfala_HasProperty: bad driver reference");
	DoIfFailed(inAddress == NULL, return false, "syfala_HasProperty: no address");

	switch(inObjectID)
	{
		case PLUGIN_ID:
			return syfala_HasPlugInProperty(inAddress);
		
		case DEVICE_ID:
			return syfala_HasDeviceProperty(inAddress);
		
		case STREAM_ID:
			return syfala_HasStreamProperty(inAddress);
	}

	return audio_object_is_control(inObjectID) && syfala_HasControlProperty(inObjectID, inAddress);
}

static OSStatus	syfala_IsPropertySettable(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inObjectID,
	pid_t const inClientProcessID,
	AudioObjectPropertyAddress const* const inAddress,
	Boolean* const outIsSettable
) {
	char const selector[5] = FourCCToCString(inAddress->mSelector);
    char const scope[5] = FourCCToCString(inAddress->mScope);

    DebugMsg(
		"IsSettable: %02u { %s %s %02d } by %08x",
		inObjectID,
		selector,
		scope,
		inAddress->mElement,
		inClientProcessID
	);

	//	check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_IsPropertySettable: bad driver reference");
	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "syfala_IsPropertySettable: no address");
	DoIfFailed(outIsSettable == NULL, return kAudioHardwareIllegalOperationError, "syfala_IsPropertySettable: no place to put the return value");

	switch(inObjectID)
	{
		case PLUGIN_ID:
			return syfala_IsPlugInPropertySettable(inAddress, outIsSettable);
		
		case DEVICE_ID:
			return syfala_IsDevicePropertySettable(inAddress, outIsSettable);
		
		case STREAM_ID:
			return syfala_IsStreamPropertySettable(inAddress, outIsSettable);
	};

	return (audio_object_is_control(inObjectID)) ?
		syfala_IsControlPropertySettable(inObjectID, inAddress, outIsSettable) :
		kAudioHardwareBadObjectError;
}

static OSStatus	syfala_GetPropertyDataSize(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inObjectID,
	pid_t const inClientProcessID,
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inQualifierDataSize,
	void const* const inQualifierData,
	UInt32* const outDataSize
) {
	char const selector[5] = FourCCToCString(inAddress->mSelector);
    char const scope[5] = FourCCToCString(inAddress->mScope);

    DebugMsg(
		"DataSize  : %02u { %s %s %02d } by %08x ",
		inObjectID,
		selector,
		scope,
		inAddress->mElement,
		inClientProcessID
	);
	
	//	check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_GetPropertyDataSize: bad driver reference");
	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyDataSize: no address");
	DoIfFailed(outDataSize == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyDataSize: no place to put the return value");

	switch(inObjectID)
	{
		case PLUGIN_ID:
			return syfala_GetPlugInPropertyDataSize(
				inAddress,
				inQualifierDataSize,
				inQualifierData,
				outDataSize
			);
		
		case DEVICE_ID:
			return syfala_GetDevicePropertyDataSize(inAddress, outDataSize);
		
		case STREAM_ID:
			return syfala_GetStreamPropertyDataSize(inAddress, outDataSize);
	};

	return (audio_object_is_control(inObjectID)) ?
		syfala_GetControlPropertyDataSize(inObjectID, inAddress, outDataSize) :
		kAudioHardwareBadObjectError;
}

static OSStatus	syfala_GetPropertyData(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inObjectID,
	pid_t const inClientProcessID,
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inQualifierDataSize,
	void const* const inQualifierData,
	UInt32 const inDataSize,
	UInt32* const outDataSize,
	void* const outData
) {
	char const selector[5] = FourCCToCString(inAddress->mSelector);
    char const scope[5] = FourCCToCString(inAddress->mScope);

    DebugMsg(
		"GetData   : %02u { %s %s %02d } by %08x",
		inObjectID, 
		selector,
		scope,
		inAddress->mElement,
		inClientProcessID
	);

	//	check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_GetPropertyData: bad driver reference");
	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyData: no address");
	DoIfFailed(outDataSize == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyData: no place to put the return value size");
	DoIfFailed(outData == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyData: no place to put the return value");

	switch(inObjectID)
	{
		case PLUGIN_ID:
			return syfala_GetPlugInPropertyData(
				inAddress,
				inQualifierDataSize,
				inQualifierData,
				inDataSize,
				outDataSize,
				outData
			);

		case DEVICE_ID:
			return syfala_GetDevicePropertyData(inAddress, inDataSize, outDataSize, outData);
		
		case STREAM_ID:
			return syfala_GetStreamPropertyData(inAddress, inDataSize, outDataSize, outData);
	};

	return (audio_object_is_control(inObjectID)) ?
		syfala_GetControlPropertyData(inObjectID, inAddress, inDataSize, outDataSize, outData) :
		kAudioHardwareBadObjectError;
}

static OSStatus	syfala_SetPropertyData(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inObjectID,
	pid_t const inClientProcessID,
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inQualifierDataSize,
	void const* const inQualifierData,
	UInt32 const inDataSize,
	void const* const inData
) {
	char const selector[5] = FourCCToCString(inAddress->mSelector);
    char const scope[5] = FourCCToCString(inAddress->mScope);

    DebugMsg(
		"SetData   : %02u { %s %s %02d } by %08x",
		inObjectID,
		selector,
		scope,
		inAddress->mElement,
		inClientProcessID
	);

	//	check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_SetPropertyData: bad driver reference");
	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "syfala_SetPropertyData: no address");

	// Only 2 * (N_CHANNELS + 1) + 1 properties are settable in our plugin:
	//		- Whether the output stream is active.
	// 		- The values of the volume and mule controls for each channel + the master controls

	if (inObjectID == STREAM_ID) {
		if (inAddress->mSelector == kAudioStreamPropertyIsActive) {
			//	Changing the active state of a stream doesn't affect IO or change the structure
			//	so we can just save the state and send the notification.
			DoIfFailed(inDataSize != sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_SetStreamPropertyData: wrong size for the data for kAudioDevicePropertyNominalSampleRate");
			bool const new_state = (*((UInt32 const*)inData) != 0);
	
			atomic_store_explicit(&IS_OUTPUT_STREAM_ACTIVE, new_state, __ATOMIC_RELAXED);

			HOST->PropertiesChanged(HOST, inObjectID, 1, inAddress);
			DebugMsg("HostChange");
			return kAudioHardwareNoError;
		}

		return kAudioHardwareUnknownPropertyError;
	}

	if (audio_object_is_control(inObjectID)) {
		UInt32 n_changed_addrs = 0;
		// we might change two properties when changing a volume control because
		// it changes both the scalar value property and the decibel value property
		AudioObjectPropertyAddress changed_addrs[2];

		OSStatus const status = syfala_SetControlPropertyData(
			inObjectID,
			inAddress,
			inDataSize,
			inData,
			&n_changed_addrs,
			changed_addrs
		);
			
		if (status != 0) return status;

		if (n_changed_addrs > 0) {
			HOST->PropertiesChanged(HOST, inObjectID, n_changed_addrs, changed_addrs);
			DebugMsg("HostChange");
		}

		return kAudioHardwareNoError;
	}

	return kAudioHardwareBadObjectError;
}

#pragma mark PlugIn Property Operations

static Boolean	syfala_HasPlugInProperty(
	AudioObjectPropertyAddress const* const inAddress
) {
	//	This method returns whether or not the plug-in object has the given property.

	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyManufacturer:
		case kAudioObjectPropertyOwnedObjects:

		case kAudioPlugInPropertyDeviceList:
		case kAudioPlugInPropertyTranslateUIDToDevice:
		case kAudioPlugInPropertyResourceBundle:
			return true;
	};

	return false;
}

static OSStatus	syfala_IsPlugInPropertySettable(
	AudioObjectPropertyAddress const* const inAddress,
	Boolean* const outIsSettable
) {
	//	This method returns whether or not the given property on the plug-in object can have its
	//	value changed.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyManufacturer:
		case kAudioObjectPropertyOwnedObjects:

		case kAudioPlugInPropertyDeviceList:
		case kAudioPlugInPropertyTranslateUIDToDevice:
		case kAudioPlugInPropertyResourceBundle:
			*outIsSettable = false;
		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

static OSStatus	syfala_GetPlugInPropertyDataSize(
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inQualifierDataSize,
	void const* const inQualifierData,
	UInt32* const outDataSize
) {
	//	This method returns the byte size of the property's data.

	//	Note that for each object, this driver implements all the required properties plus a few
	//	extras that are useful but not required. There is more detailed commentary about each
	//	property in the syfala_GetPlugInPropertyData() method.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
			*outDataSize = sizeof(AudioClassID);
			break;
			
		case kAudioObjectPropertyOwner:
			*outDataSize = sizeof(AudioObjectID);
			break;
			
		case kAudioObjectPropertyManufacturer:
			*outDataSize = sizeof(CFStringRef);
			break;
			
		case kAudioObjectPropertyOwnedObjects:
		case kAudioPlugInPropertyDeviceList:
			*outDataSize = 1 * sizeof(AudioObjectID);
			break;
			
		case kAudioPlugInPropertyTranslateUIDToDevice:
			*outDataSize = sizeof(AudioObjectID);
			break;
			
		case kAudioPlugInPropertyResourceBundle:
			*outDataSize = sizeof(CFStringRef);
			break;

		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

static OSStatus	syfala_GetPlugInPropertyData(
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inQualifierDataSize,
	void const* const inQualifierData,
	UInt32 const inDataSize,
	UInt32* const outDataSize,
	void* const outData
) {
	//	Note that for each object, this driver implements all the required properties plus a few
	//	extras that are useful but not required.
	//
	//	Also, since most of the data that will get returned is static, there are few instances where
	//	it is necessary to lock the state mutex.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyBaseClass:
			//	The base class for kAudioPlugInClassID is kAudioObjectClassID
			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the plug-in");
			*((AudioClassID*)outData) = kAudioObjectClassID;
			*outDataSize = sizeof(AudioClassID);
			break;
			
		case kAudioObjectPropertyClass:
			//	The class is always kAudioPlugInClassID for regular drivers
			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: not enough space for the return value of kAudioObjectPropertyClass for the plug-in");
			*((AudioClassID*)outData) = kAudioPlugInClassID;
			*outDataSize = sizeof(AudioClassID);
			break;
			
		case kAudioObjectPropertyOwner:
			//	The plug-in doesn't have an owning object
			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: not enough space for the return value of kAudioObjectPropertyOwner for the plug-in");
			*((AudioObjectID*)outData) = kAudioObjectUnknown;
			*outDataSize = sizeof(AudioObjectID);
			break;
			
		case kAudioObjectPropertyManufacturer:
			//	This is the human readable name of the maker of the plug-in.
			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: not enough space for the return value of kAudioObjectPropertyManufacturer for the plug-in");
			*((CFStringRef*)outData) = CFSTR("GRAME CNCM");
			*outDataSize = sizeof(CFStringRef);
			break;
		

		// The plugin owns only 1 object, which is the device. So, handle these two cases the same way
		case kAudioObjectPropertyOwnedObjects:
		case kAudioPlugInPropertyDeviceList:
		{
			//	Calculate the number of items that have been requested. Note that this
			//	number is allowed to be smaller than the actual size of the list. In such
			//	case, only that number of items will be returned
			UInt32 const n_items = inDataSize / sizeof(AudioObjectID);
			//	Clamp that to the number of devices this driver implements (which is just 1)
			UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

			AudioObjectID* const data = (AudioObjectID*) outData;

			UInt32 index = 0;
			
			if(index < n_items_clamped) data[index++] = DEVICE_ID;

			//	Return how many bytes we wrote to
			*outDataSize = n_items_clamped * sizeof(AudioObjectID);
			break;
		}

		case kAudioPlugInPropertyTranslateUIDToDevice:
			//	This property takes the CFString passed in the qualifier and converts that
			//	to the object ID of the device it corresponds to. For this driver, there is
			//	just the one device. Note that it is not an error if the string in the
			//	qualifier doesn't match any devices. In such case, kAudioObjectUnknown is
			//	the object ID to return.
			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: not enough space for the return value of kAudioPlugInPropertyTranslateUIDToDevice");
			DoIfFailed(inQualifierDataSize != sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: the qualifier is the wrong size for kAudioPlugInPropertyTranslateUIDToDevice");
			DoIfFailed(inQualifierData == NULL, return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: no qualifier for kAudioPlugInPropertyTranslateUIDToDevice");
			
			if(
				CFStringCompare(
					*((CFStringRef*)inQualifierData),
					CFSTR(kDevice_UID),
					0	
				) == kCFCompareEqualTo
			) *((AudioObjectID*)outData) = DEVICE_ID;

			else *((AudioObjectID*)outData) = kAudioObjectUnknown;
			
			*outDataSize = sizeof(AudioObjectID);
			break;
			
		case kAudioPlugInPropertyResourceBundle:
			//	The resource bundle is a path relative to the path of the plug-in's bundle.
			//	To specify that the plug-in bundle itself should be used, we just return the
			//	empty string.
			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: not enough space for the return value of kAudioPlugInPropertyResourceBundle");
			*((CFStringRef*)outData) = CFSTR("");
			*outDataSize = sizeof(CFStringRef);
			break;

		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

#pragma mark Device Property Operations

static Boolean	syfala_HasDeviceProperty(
	AudioObjectPropertyAddress const* const inAddress
) {
	//	This method returns whether or not the given object has the given property.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyElementName:
			return inAddress->mElement <= N_CHANNELS;

		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyName:
		case kAudioObjectPropertyManufacturer:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioObjectPropertyControlList:

		case kAudioDevicePropertyDeviceUID:
		case kAudioDevicePropertyModelUID:
		case kAudioDevicePropertyTransportType:
		case kAudioDevicePropertyRelatedDevices:
		case kAudioDevicePropertyClockDomain:
		case kAudioDevicePropertyDeviceIsAlive:
		case kAudioDevicePropertyDeviceIsRunning:
		case kAudioDevicePropertyNominalSampleRate:
		case kAudioDevicePropertyAvailableNominalSampleRates:
		case kAudioDevicePropertyIsHidden:
		case kAudioDevicePropertyZeroTimeStampPeriod:
		case kAudioDevicePropertyIcon:
		case kAudioDevicePropertyStreams:
			return true;

		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
		case kAudioDevicePropertyLatency:
		case kAudioDevicePropertySafetyOffset:
		case kAudioDevicePropertyPreferredChannelLayout:
			return (inAddress->mScope == kAudioObjectPropertyScopeOutput) || (inAddress->mScope == kAudioObjectPropertyScopeGlobal);
	};

	return false;
}

static OSStatus	syfala_IsDevicePropertySettable(
	AudioObjectPropertyAddress const* const inAddress,
	Boolean* const outIsSettable
) {
	//	This method returns whether or not the given property on the object can have its value
	//	changed.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyElementName:
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyName:
		case kAudioObjectPropertyManufacturer:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioObjectPropertyControlList:

		case kAudioDevicePropertyDeviceUID:
		case kAudioDevicePropertyModelUID:
		case kAudioDevicePropertyTransportType:
		case kAudioDevicePropertyRelatedDevices:
		case kAudioDevicePropertyClockDomain:
		case kAudioDevicePropertyDeviceIsAlive:
		case kAudioDevicePropertyDeviceIsRunning:
		case kAudioDevicePropertyNominalSampleRate:
		case kAudioDevicePropertyAvailableNominalSampleRates:
		case kAudioDevicePropertyIsHidden:
		case kAudioDevicePropertyZeroTimeStampPeriod:
		case kAudioDevicePropertyStreams:
		case kAudioDevicePropertyIcon:
		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
		case kAudioDevicePropertyLatency:
		case kAudioDevicePropertySafetyOffset:
		case kAudioDevicePropertyPreferredChannelLayout:
			*outIsSettable = false;
			break;

		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

static OSStatus	syfala_GetDevicePropertyDataSize(
	AudioObjectPropertyAddress const* const inAddress,
	UInt32* const outDataSize
) {
	//	This method returns the byte size of the property's data.
	
	//	Note that for each object, this driver implements all the required properties plus a few
	//	extras that are useful but not required. There is more detailed commentary about each
	//	property in the syfala_GetDevicePropertyData() method.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyElementName:
			*outDataSize = sizeof(CFStringRef);
			break;

		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
			*outDataSize = sizeof(AudioClassID);
			break;
		
		case kAudioObjectPropertyOwner:
			*outDataSize = sizeof(AudioObjectID);
			break;
		
		case kAudioObjectPropertyName:
		case kAudioObjectPropertyManufacturer:
			*outDataSize = sizeof(CFStringRef);
			break;
		
		case kAudioObjectPropertyOwnedObjects:
			switch(inAddress->mScope)
			{
				case kAudioObjectPropertyScopeGlobal:
				case kAudioObjectPropertyScopeOutput:
					*outDataSize = N_DEVICE_OWNED_OBJS * sizeof(AudioObjectID);
					break;
				default:
					return kAudioHardwareUnknownPropertyError;
			};
			break;
		
		case kAudioObjectPropertyControlList:

			*outDataSize = N_CONTROLS * sizeof(AudioObjectID);
			break;

		case kAudioDevicePropertyDeviceUID:
			*outDataSize = sizeof(CFStringRef);
			break;

		case kAudioDevicePropertyModelUID:
			*outDataSize = sizeof(CFStringRef);
			break;

		case kAudioDevicePropertyTransportType:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyRelatedDevices:
			*outDataSize = 1 * sizeof(AudioObjectID);
			break;

		case kAudioDevicePropertyClockDomain:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyDeviceIsAlive:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyDeviceIsRunning:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyNominalSampleRate:
			*outDataSize = sizeof(Float64);
			break;

		case kAudioDevicePropertyAvailableNominalSampleRates:
			*outDataSize = 1 * sizeof(AudioValueRange);
			break;

		case kAudioDevicePropertyIsHidden:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyZeroTimeStampPeriod:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyIcon:
			*outDataSize = sizeof(CFURLRef);
			break;

		case kAudioDevicePropertyStreams:
			switch(inAddress->mScope)
			{
				case kAudioObjectPropertyScopeGlobal:
				case kAudioObjectPropertyScopeOutput:
					*outDataSize = 1 * sizeof(AudioObjectID);
					break;
				case kAudioObjectPropertyScopeInput:
					*outDataSize = 0 * sizeof(AudioObjectID);
					break;

				default:
					return kAudioHardwareUnknownPropertyError;
			};
			break;

		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyLatency:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertySafetyOffset:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyPreferredChannelLayout:
			*outDataSize = offsetof(AudioChannelLayout, mChannelDescriptions) + (1 * sizeof(AudioChannelDescription));
			break;

		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

static OSStatus	syfala_GetDevicePropertyData(
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inDataSize,
	UInt32* const outDataSize,
	void* const outData
) {	
	//	Note that for each object, this driver implements all the required properties plus a few
	//	extras that are useful but not required.
	//
	//	Also, since most of the data that will get returned is static, there are few instances where
	//	it is necessary to lock the state mutex.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyElementName: {
			//	This is the human readable name of the maker of the plug-in.
			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyElementName for the device");

			CFStringRef* const data = (CFStringRef*) outData;
			AudioObjectPropertyElement const elem = inAddress->mElement;

			if (elem == kAudioObjectPropertyElementMain /* AKA 0 */) {
				*data = CFSTR("Master");
			} else if (elem <= N_CHANNELS) {
				*data = CFStringCreateWithFormat(NULL, NULL, CFSTR("%u"), elem);
			} else {
				*data = CFSTR("<Unknown>");
			}

			*outDataSize = sizeof(CFStringRef);
			break;
		}

		case kAudioObjectPropertyBaseClass:
			//	The base class for kAudioDeviceClassID is kAudioObjectClassID
			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the device");
			*((AudioClassID*)outData) = kAudioObjectClassID;
			*outDataSize = sizeof(AudioClassID);
			break;

		case kAudioObjectPropertyClass:
			//	The class is always kAudioDeviceClassID for devices created by drivers
			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyClass for the device");
			*((AudioClassID*)outData) = kAudioDeviceClassID;
			*outDataSize = sizeof(AudioClassID);
			break;

		case kAudioObjectPropertyOwner:
			//	The device's owner is the plug-in object
			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyOwner for the device");
			*((AudioObjectID*)outData) = PLUGIN_ID;
			*outDataSize = sizeof(AudioObjectID);
			break;

		case kAudioObjectPropertyName:
			//	This is the human readable name of the device.
			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyName for the device");
			*((CFStringRef*)outData) = CFSTR("NAME");
			*outDataSize = sizeof(CFStringRef);
			break;

		case kAudioObjectPropertyManufacturer:
			//	This is the human readable name of the maker of the device.
			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyManufacturer for the device");
			*((CFStringRef*)outData) = CFSTR("ME"); 
			*outDataSize = sizeof(CFStringRef);
			break;

		// These two cases return the same list but with the output
		// stream id included if all owned objects are requested

		//	The device owns its streams and controls.
		case kAudioObjectPropertyOwnedObjects:
		case kAudioObjectPropertyControlList:
			
			switch(inAddress->mScope)
			{
				case kAudioObjectPropertyScopeGlobal:
				case kAudioObjectPropertyScopeOutput:
				{
					//	Calculate the number of items that have been requested. Note that this
					//	number is allowed to be smaller than the actual size of the list. In such
					//	case, only that number of items will be returned.

					// --  TO CHATGPT  --:
					// use write_get_size with an iterator/array for properties with the comment above
					UInt32 const n_items = inDataSize / sizeof(AudioObjectID);
					bool const include_stream_obj = inAddress->mSelector == kAudioObjectPropertyOwnedObjects;
					UInt32 const max_n_items = N_CONTROLS + ((include_stream_obj) ? 1 : 0);
					UInt32 const n_items_clamped = (n_items > max_n_items) ? max_n_items : n_items; 

					AudioObjectID* const data = (AudioObjectID*) outData;

					UInt32 index = 0;

					// output stream, return that only if we actually requested it
					if (index < n_items_clamped && include_stream_obj)
						data[index++] = STREAM_ID;

					for (AudioObjectID id = FIRST_CTRL_ID; id <= LAST_CTRL_ID; ++id)
						if (index < n_items_clamped) data[index++] = id;

					//	report how much we wrote
					*outDataSize = n_items_clamped * sizeof(AudioObjectID);
					break;
				}
				case kAudioObjectPropertyScopeInput:
					*outDataSize = 0 * sizeof(AudioObjectID);
					break;

				default:
					return kAudioHardwareUnknownPropertyError;
			};
			
			break;

		case kAudioDevicePropertyDeviceUID:
			//	This is a CFString that is a persistent token that can identify the same
			//	audio device across boot sessions. Note that two instances of the same
			//	device must have different values for this property.
			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceUID for the device");
			*((CFStringRef*)outData) = CFSTR(kDevice_UID);
			*outDataSize = sizeof(CFStringRef);
			break;

		case kAudioDevicePropertyModelUID:
			//	This is a CFString that is a persistent token that can identify audio
			//	devices that are the same kind of device. Note that two instances of the
			//	save device must have the same value for this property.
			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyModelUID for the device");
			*((CFStringRef*)outData) = CFSTR(kDevice_ModelUID);
			*outDataSize = sizeof(CFStringRef);
			break;

		case kAudioDevicePropertyTransportType:
			//	This value represents how the device is attached to the system. This can be
			//	any 32 bit integer, but common values for this property are defined in
			//	<CoreAudio/AudioHardwareBase.h>
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyTransportType for the device");
			*((UInt32*)outData) = kAudioDeviceTransportTypeAVB;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyRelatedDevices:
		{
			//	The related devices property identifys device objects that are very closely
			//	related. Generally, this is for relating devices that are packaged together
			//	in the hardware such as when the input side and the output side of a piece
			//	of hardware can be clocked separately and therefore need to be represented
			//	as separate AudioDevice objects. In such case, both devices would report
			//	that they are related to each other. Note that at minimum, a device is
			//	related to itself, so this list will always be at least one item long.

			//	Calculate the number of items that have been requested. Note that this
			//	number is allowed to be smaller than the actual size of the list. In such
			//	case, only that number of items will be returned
			UInt32 const n_items = inDataSize / sizeof(AudioObjectID);
			UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

			AudioObjectID* const data = (AudioObjectID*) outData;
			
			UInt32 index = 0;

			//	Write the devices' object IDs into the return value
			if(index < n_items_clamped) data[index++] = DEVICE_ID;
			
			//	report how much we wrote
			*outDataSize = n_items_clamped * sizeof(AudioObjectID);
			break;
		}

		case kAudioDevicePropertyClockDomain:
			//	This property allows the device to declare what other devices it is
			//	synchronized with in hardware. The way it works is that if two devices have
			//	the same value for this property and the value is not zero, then the two
			//	devices are synchronized in hardware. Note that a device that either can't
			//	be synchronized with others or doesn't know should return 0 for this
			//	property.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyClockDomain for the device");
			*((UInt32*)outData) = 0;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyDeviceIsAlive:
			//	This property returns whether or not the device is alive. Note that it is
			//	note uncommon for a device to be dead but still momentarily availble in the
			//	device list. In the case of this device, it will always be alive.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceIsAlive for the device");
			*((UInt32*)outData) = 1;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyDeviceIsRunning:
			//	This property returns whether or not IO is running for the device
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceIsRunning for the device");
			pthread_mutex_lock(&gDevice_IOMutex);
			*((UInt32*)outData) = (gDevice_IOIsRunning > 0) ? 1 : 0;
			pthread_mutex_unlock(&gDevice_IOMutex);
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyNominalSampleRate:
			//	This property returns the nominal sample rate of the device.
			DoIfFailed(inDataSize < sizeof(Float64), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyNominalSampleRate for the device");
			*((Float64*)outData) = SAMPLE_RATE;
			*outDataSize = sizeof(Float64);
			break;

		case kAudioDevicePropertyAvailableNominalSampleRates:
		{
			//	This returns all nominal sample rates the device supports as an array of
			//	AudioValueRangeStructs. Note that for discrete sampler rates, the range
			//	will have the minimum value equal to the maximum value.
			
			//	Calculate the number of items that have been requested. Note that this
			//	number is allowed to be smaller than the actual size of the list. In such
			//	case, only that number of items will be returned
			UInt32 const n_items = inDataSize / sizeof(AudioValueRange);
			UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

			AudioValueRange* const data = (AudioValueRange*) outData;

			UInt32 index = 0;

			if (index < n_items_clamped) {
				AudioValueRange* const range = &data[index++];
				range->mMinimum = SAMPLE_RATE;
				range->mMaximum = SAMPLE_RATE;
			}
			
			//	report how much we wrote
			*outDataSize = n_items_clamped * sizeof(AudioValueRange);
			break;
		}

		case kAudioDevicePropertyIsHidden:
			//	This returns whether or not the device is visible to clients.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyIsHidden for the device");
			*((UInt32*)outData) = 0;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyZeroTimeStampPeriod:
			//	This property returns how many frames the HAL should expect to see between
			//	successive sample times in the zero time stamps this device provides.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyZeroTimeStampPeriod for the device");
			*((UInt32*)outData) = kDevice_RingBufferSize;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyIcon:
			//	This is a CFURL that points to the device's Icon in the plug-in's resource bundle.
			DoIfFailed(inDataSize < sizeof(CFURLRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceUID for the device");
			CFBundleRef theBundle = CFBundleGetBundleWithIdentifier(CFSTR(kPlugIn_BundleID));
			DoIfFailed(theBundle == NULL, return kAudioHardwareUnspecifiedError, "syfala_GetDevicePropertyData: could not get the plug-in bundle for kAudioDevicePropertyIcon");
			CFURLRef theURL = CFBundleCopyResourceURL(theBundle, CFSTR("DeviceIcon.icns"), NULL, NULL);
			DoIfFailed(theURL == NULL, return kAudioHardwareUnspecifiedError, "syfala_GetDevicePropertyData: could not get the URL for kAudioDevicePropertyIcon");
			*((CFURLRef*)outData) = theURL;
			*outDataSize = sizeof(CFURLRef);
			break;

		case kAudioDevicePropertyStreams:
			switch(inAddress->mScope)
			{
				case kAudioObjectPropertyScopeGlobal:
				case kAudioObjectPropertyScopeOutput:
				{
					//	Calculate the number of items that have been requested. Note that this
					//	number is allowed to be smaller than the actual size of the list. In such
					//	case, only that number of items will be returned
					UInt32 const n_items = inDataSize / sizeof(AudioObjectID);
					UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

					AudioObjectID* const data = (AudioObjectID*) outData;

					UInt32 index = 0;

					if (index < n_items_clamped)
						data[index++] = STREAM_ID;

					//	report how much we wrote
					*outDataSize = n_items_clamped * sizeof(AudioObjectID);
					break;
				}

				default:
					return kAudioHardwareUnknownPropertyError;
			};

			break;

		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
			//	This property returns whether or not the device wants to be able to be the
			//	default device for content. This is the device that iTunes and QuickTime
			//	will use to play their content on and FaceTime will use as it's microhphone.
			//	Nearly all devices should allow for this.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceCanBeDefaultDevice for the device");
			*((UInt32*)outData) = 1;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
			//	This property returns whether or not the device wants to be the system
			//	default device. This is the device that is used to play interface sounds and
			//	other incidental or UI-related sounds on. Most devices should allow this
			//	although devices with lots of latency may not want to.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceCanBeDefaultSystemDevice for the device");
			*((UInt32*)outData) = 1;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyLatency:
			//	This property returns the presentation latency of the device. For this,
			//	device, the value is 0 due to the fact that it always vends silence.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyLatency for the device");
			*((UInt32*)outData) = 0;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertySafetyOffset:
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertySafetyOffset for the device");
			*((UInt32*)outData) = 0;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioDevicePropertyPreferredChannelLayout:
		{
			UInt32 const size = offsetof(AudioChannelLayout, mChannelDescriptions) + (1 * sizeof(AudioChannelDescription));

			AudioChannelLayout* const data = (AudioChannelLayout*) outData;
			
			DoIfFailed(inDataSize < size, return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyPreferredChannelLayout for the device");

			data->mChannelLayoutTag = kAudioChannelLayoutTag_UseChannelDescriptions;
			data->mChannelBitmap = 0;
			data->mNumberChannelDescriptions = 1;

			AudioChannelDescription* const desc = &data->mChannelDescriptions[0];

			desc->mChannelLabel = 1;
			desc->mChannelFlags = 0;
			desc->mCoordinates[0] = 0;
			desc->mCoordinates[1] = 0;
			desc->mCoordinates[2] = 0;
			*outDataSize = size;
			break;
		}

		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

#pragma mark Stream Property Operations

static Boolean	syfala_HasStreamProperty(
	AudioObjectPropertyAddress const* inAddress
) {
	//	This method returns whether or not the given object has the given property.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioObjectPropertyName:

		case kAudioStreamPropertyDirection:
		case kAudioStreamPropertyTerminalType:
		case kAudioStreamPropertyStartingChannel:
		case kAudioStreamPropertyLatency:
		case kAudioStreamPropertyAvailableVirtualFormats:
		case kAudioStreamPropertyAvailablePhysicalFormats:
		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat:
		case kAudioStreamPropertyIsActive:
			return true;
	};

	return false;
}

static OSStatus	syfala_IsStreamPropertySettable(
	AudioObjectPropertyAddress const* const inAddress,
	Boolean* const outIsSettable
) {
	//	This method returns whether or not the given property on the object can have its value
	//	changed.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioObjectPropertyName:
	
		case kAudioStreamPropertyDirection:
		case kAudioStreamPropertyTerminalType:
		case kAudioStreamPropertyStartingChannel:
		case kAudioStreamPropertyLatency:
		case kAudioStreamPropertyAvailableVirtualFormats:
		case kAudioStreamPropertyAvailablePhysicalFormats:
		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat:
			*outIsSettable = false;
			break;
		
		case kAudioStreamPropertyIsActive:
			*outIsSettable = true;
			break;
		
		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

static OSStatus	syfala_GetStreamPropertyDataSize(
	AudioObjectPropertyAddress const* const inAddress,
	UInt32* const outDataSize
) {
	//	This method returns the byte size of the property's data.
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyBaseClass:
			*outDataSize = sizeof(AudioClassID);
			break;

		case kAudioObjectPropertyClass:
			*outDataSize = sizeof(AudioClassID);
			break;

		case kAudioObjectPropertyOwner:
			*outDataSize = sizeof(AudioObjectID);
			break;

		case kAudioObjectPropertyOwnedObjects:
			*outDataSize = 0 * sizeof(AudioObjectID);
			break;

		case kAudioObjectPropertyName:
			*outDataSize = sizeof(CFStringRef);
			break;


		case kAudioStreamPropertyDirection:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioStreamPropertyTerminalType:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioStreamPropertyStartingChannel:
			*outDataSize = sizeof(UInt32);
			break;
		
		case kAudioStreamPropertyLatency:
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioStreamPropertyAvailableVirtualFormats:
		case kAudioStreamPropertyAvailablePhysicalFormats:
			*outDataSize = 1 * sizeof(AudioStreamRangedDescription);
			break;

		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat:
			*outDataSize = sizeof(AudioStreamBasicDescription);
			break;

		case kAudioStreamPropertyIsActive:
			*outDataSize = sizeof(UInt32);
			break;

		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

static OSStatus	syfala_GetStreamPropertyData(
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inDataSize,
	UInt32* const outDataSize,
	void* const outData
) {
	switch(inAddress->mSelector)
	{
		case kAudioObjectPropertyBaseClass:
			//	The base class for kAudioStreamClassID is kAudioObjectClassID
			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the stream");
			*((AudioClassID*)outData) = kAudioObjectClassID;
			*outDataSize = sizeof(AudioClassID);
			break;
			
		case kAudioObjectPropertyClass:
			//	The class is always kAudioStreamClassID for streams created by drivers
			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioObjectPropertyClass for the stream");
			*((AudioClassID*)outData) = kAudioStreamClassID;
			*outDataSize = sizeof(AudioClassID);
			break;
			
		case kAudioObjectPropertyOwner:
			//	The stream's owner is the device object
			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioObjectPropertyOwner for the stream");
			*((AudioObjectID*)outData) = DEVICE_ID;
			*outDataSize = sizeof(AudioObjectID);
			break;

		case kAudioObjectPropertyOwnedObjects:
			//	Streams do not own any objects
			*outDataSize = 0 * sizeof(AudioObjectID);
			break;

		case kAudioObjectPropertyName:
			//	This is the human readable name of the stream
			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioObjectPropertyName for the stream");
			*((CFStringRef*)outData) = CFSTR("Output Stream");
			*outDataSize = sizeof(CFStringRef);
			break;

		case kAudioStreamPropertyDirection:
			//	This returns whether the stream is an input stream or an output stream.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyDirection for the stream");
			*((UInt32*)outData) = 0;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioStreamPropertyTerminalType:
			//	This returns a value that indicates what is at the other end of the stream
			//	such as a speaker or headphones, or a microphone. Values for this property
			//	are defined in <CoreAudio/AudioHardwareBase.h>
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyTerminalType for the stream");
			*((UInt32*)outData) = kAudioStreamTerminalTypeUnknown;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioStreamPropertyStartingChannel:
			//	This property returns the absolute channel number for the first channel in
			//	the stream. For exmaple, if a device has two output streams with two
			//	channels each, then the starting channel number for the first stream is 1
			//	and ths starting channel number fo the second stream is 3.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyStartingChannel for the stream");
			*((UInt32*)outData) = 1;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioStreamPropertyLatency:
			//	This property returns any additonal presentation latency the stream has.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyStartingChannel for the stream");
			*((UInt32*)outData) = 0;
			*outDataSize = sizeof(UInt32);
			break;

		case kAudioStreamPropertyAvailableVirtualFormats:
		case kAudioStreamPropertyAvailablePhysicalFormats:
		{
			//	This returns an array of AudioStreamRangedDescriptions that describe what
			//	formats are supported.

			//	Calculate the number of items that have been requested. Note that this
			//	number is allowed to be smaller than the actual size of the list. In such
			//	case, only that number of items will be returned
			UInt32 const n_items = inDataSize / sizeof(AudioStreamRangedDescription);
			UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

			AudioStreamRangedDescription* const data = (AudioStreamRangedDescription*) outData;

			UInt32 index = 0;

			if (index < n_items_clamped) data[index++] = FORMAT;

			//	report how much we wrote
			*outDataSize = n_items_clamped * sizeof(AudioStreamRangedDescription);
			break;
		}

		case kAudioStreamPropertyVirtualFormat:
		case kAudioStreamPropertyPhysicalFormat:
			//	This returns the current format of the stream in an
			//	AudioStreamBasicDescription. Note that we need to hold the state lock to get
			//	this value.
			//	Note that for devices that don't override the mix operation, the virtual
			//	format has to be the same as the physical format.
			DoIfFailed(inDataSize < sizeof(AudioStreamBasicDescription), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyVirtualFormat for the stream");
			*((AudioStreamBasicDescription*)outData) = FORMAT.mFormat;
			*outDataSize = sizeof(AudioStreamBasicDescription);
			break;

		case kAudioStreamPropertyIsActive:
			//	This property tells the device whether or not the given stream is going to
			//	be used for IO. Note that we need to take the state lock to examine this
			//	value.
			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyIsActive for the stream");
			*((UInt32*)outData) = atomic_load_explicit(&IS_OUTPUT_STREAM_ACTIVE, __ATOMIC_RELAXED);
			*outDataSize = sizeof(UInt32);
			break;

		default:
			return kAudioHardwareUnknownPropertyError;
	};

	return kAudioHardwareNoError;
}

#pragma mark Control Property Operations

static Boolean	syfala_HasControlProperty(
	AudioObjectID const inObjectID,
	AudioObjectPropertyAddress const* const inAddress
) {
	switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioControlPropertyScope:
		case kAudioControlPropertyElement:
			return true;
	}

	if (control_is_volume(inObjectID)) {
		switch (inAddress->mSelector) {
			case kAudioLevelControlPropertyScalarValue:
			case kAudioLevelControlPropertyDecibelValue:
			case kAudioLevelControlPropertyDecibelRange:
			case kAudioLevelControlPropertyConvertScalarToDecibels:
			case kAudioLevelControlPropertyConvertDecibelsToScalar:
				return true;
		}
	} else {
		if (inAddress->mSelector == kAudioBooleanControlPropertyValue) {
			return true;
		}
	}

	return false;
}

static OSStatus	syfala_IsControlPropertySettable(
	AudioObjectID const inObjectID,
	AudioObjectPropertyAddress const* const inAddress,
	Boolean* const outIsSettable
) {
	switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
		case kAudioObjectPropertyClass:
		case kAudioObjectPropertyOwner:
		case kAudioObjectPropertyOwnedObjects:
		case kAudioControlPropertyScope:
		case kAudioControlPropertyElement:
			*outIsSettable = false;
			return kAudioHardwareNoError;
	}

	if (control_is_volume(inObjectID)) {
		switch (inAddress->mSelector) {
			case kAudioLevelControlPropertyDecibelRange:
			case kAudioLevelControlPropertyConvertScalarToDecibels:
			case kAudioLevelControlPropertyConvertDecibelsToScalar:
				*outIsSettable = false;
				return kAudioHardwareNoError;
			
			case kAudioLevelControlPropertyScalarValue:
			case kAudioLevelControlPropertyDecibelValue:
				*outIsSettable = true;
				return kAudioHardwareNoError;
		}
	} else {
		if (inAddress->mSelector == kAudioBooleanControlPropertyValue) {
			*outIsSettable = true;
			return kAudioHardwareNoError;
		}
	}

	return kAudioHardwareUnknownPropertyError;
}

static OSStatus	syfala_GetControlPropertyDataSize(
	AudioObjectID const inObjectID,
	AudioObjectPropertyAddress const* const inAddress,
	UInt32* const outDataSize
) {
	switch (inAddress->mSelector) {
		case kAudioObjectPropertyBaseClass:
			*outDataSize = sizeof(AudioClassID);
			return kAudioHardwareNoError;

		case kAudioObjectPropertyClass:
			*outDataSize = sizeof(AudioClassID);
			return kAudioHardwareNoError;

		case kAudioObjectPropertyOwner:
			*outDataSize = sizeof(AudioObjectID);
			return kAudioHardwareNoError;

		case kAudioObjectPropertyOwnedObjects:
			*outDataSize = 0 * sizeof(AudioObjectID);
			return kAudioHardwareNoError;

		case kAudioControlPropertyScope:
			*outDataSize = sizeof(AudioObjectPropertyScope);
			return kAudioHardwareNoError;

		case kAudioControlPropertyElement:
			*outDataSize = sizeof(AudioObjectPropertyElement);
			return kAudioHardwareNoError;
	}

	if (control_is_volume(inObjectID)) {
		switch (inAddress->mSelector) {
			case kAudioLevelControlPropertyScalarValue:
				*outDataSize = sizeof(Float32);
				return kAudioHardwareNoError;

			case kAudioLevelControlPropertyDecibelValue:
				*outDataSize = sizeof(Float32);
				return kAudioHardwareNoError;

			case kAudioLevelControlPropertyDecibelRange:
				*outDataSize = sizeof(AudioValueRange);
				return kAudioHardwareNoError;

			case kAudioLevelControlPropertyConvertScalarToDecibels:
				*outDataSize = sizeof(Float32);
				return kAudioHardwareNoError;

			case kAudioLevelControlPropertyConvertDecibelsToScalar:
				*outDataSize = sizeof(Float32);
				return kAudioHardwareNoError;
		}
	} else {
		if (inAddress->mSelector == kAudioBooleanControlPropertyValue) {
			*outDataSize = sizeof(UInt32);
			return kAudioHardwareNoError;
		}
	}

	return kAudioHardwareUnknownPropertyError;
}

static OSStatus	syfala_GetControlPropertyData(
	AudioObjectID const inObjectID,
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inDataSize,
	UInt32* const outDataSize,
	void* const outData
) {
	AudioObjectPropertyElement const ctrl_index = control_channel_index(inObjectID);

	switch (inAddress->mSelector)
	{
		case kAudioObjectPropertyOwner:
			//	The control's owner is the device object
			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyOwner for the volume control");
			*((AudioObjectID*)outData) = DEVICE_ID;
			*outDataSize = sizeof(AudioObjectID);
			return kAudioHardwareNoError;

		case kAudioObjectPropertyOwnedObjects:
			//	Controls do not own any objects
			*outDataSize = 0 * sizeof(AudioObjectID);
			return kAudioHardwareNoError;

		case kAudioControlPropertyScope:
			//	This property returns the scope that the control is attached to.
			DoIfFailed(inDataSize < sizeof(AudioObjectPropertyScope), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioControlPropertyScope for the volume control");
			*((AudioObjectPropertyScope*)outData) = kAudioObjectPropertyScopeOutput;
			*outDataSize = sizeof(AudioObjectPropertyScope);
			return kAudioHardwareNoError;

		case kAudioControlPropertyElement:
			//	This property returns the element that the control is attached to.
			DoIfFailed(inDataSize < sizeof(AudioObjectPropertyElement), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioControlPropertyElement for the volume control");
			*((AudioObjectPropertyElement*)outData) = ctrl_index;
			*outDataSize = sizeof(AudioObjectPropertyElement);
			return kAudioHardwareNoError;

	}

	//	Note that for each object, this driver implements all the required properties plus a few
	//	extras that are useful but not required.
	//
	//	Also, since most of the data that will get returned is static, there are few instances where
	//	it is necessary to lock the state mutex.
	if (control_is_volume(inObjectID)) {
		// we are a volume control
		switch(inAddress->mSelector)
		{
			case kAudioObjectPropertyBaseClass:
				//	The base class for kAudioVolumeControlClassID is kAudioLevelControlClassID
				DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the volume control");
				*((AudioClassID*)outData) = kAudioLevelControlClassID;
				*outDataSize = sizeof(AudioClassID);
				return kAudioHardwareNoError;

			case kAudioObjectPropertyClass:
				//	Volume controls are of the class, kAudioVolumeControlClassID
				DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyClass for the volume control");
				*((AudioClassID*)outData) = kAudioVolumeControlClassID;
				*outDataSize = sizeof(AudioClassID);
				return kAudioHardwareNoError;
			
			case kAudioLevelControlPropertyScalarValue: {
				//	This returns the value of the control in the normalized range of 0 to 1.
				DoIfFailed(inDataSize < sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyScalarValue for the volume control");
				Float32 const gain = atomic_load_explicit(&OUTPUT_VOLUME[ctrl_index], __ATOMIC_RELAXED);
				Float32 const db = gain_to_db(gain);
				Float32 const db_clamped = fminf(fmaxf(db, VOL_MIN_DB), VOL_MAX_DB);
				Float32 const norm = linear_remap(db, VOL_MIN_DB, VOL_DB_RANGE, 0.0f, 1.0f);
				*((Float32*)outData) = sq(norm);
				*outDataSize = sizeof(Float32);
				return kAudioHardwareNoError;
			}

			case kAudioLevelControlPropertyDecibelValue: {
				//	This returns the dB value of the control.
				DoIfFailed(inDataSize < sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyDecibelValue for the volume control");

				Float32 const gain = atomic_load_explicit(&OUTPUT_VOLUME[ctrl_index], __ATOMIC_RELAXED);
				Float32 const db = gain_to_db(gain);
				Float32 const db_clamped = fminf(fmaxf(db, VOL_MIN_DB), VOL_MAX_DB);
				*((Float32*)outData) = db_clamped;
				
				//	report how much we wrote
				*outDataSize = sizeof(Float32);
				return kAudioHardwareNoError;
			}

			case kAudioLevelControlPropertyDecibelRange: {
				//	This returns the dB range of the control.
				DoIfFailed(inDataSize < sizeof(AudioValueRange), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyDecibelRange for the volume control");

				AudioValueRange* const data = (AudioValueRange*) outData;

				data->mMinimum = VOL_MIN_DB;
				data->mMaximum = VOL_MAX_DB;
				*outDataSize = sizeof(AudioValueRange);
				return kAudioHardwareNoError;
			}

			case kAudioLevelControlPropertyConvertScalarToDecibels: {
				//	This takes the scalar value in outData and converts it to dB.
				DoIfFailed(inDataSize < sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyDecibelValue for the volume control");

				Float32* const data = (Float32*) outData;

				Float32 const val = sqrtf(fminf(fmaxf(*data, 0.0f), 1.0f));

				*data = linear_remap(val, 0.0f, 1.0f, VOL_MIN_DB, VOL_DB_RANGE);

				//	report how much we wrote
				*outDataSize = sizeof(Float32);
				return kAudioHardwareNoError;
			}

			case kAudioLevelControlPropertyConvertDecibelsToScalar: {
				DoIfFailed(inDataSize < sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyScalarValue for the volume control");
				//	This takes the dB value in outData and converts it to scalar.
				Float32* const data = (Float32*) outData;

				Float32 const val = fminf(fmaxf(*data, VOL_MIN_DB), VOL_MAX_DB);

				*data = sq(linear_remap(val, VOL_MIN_DB, VOL_DB_RANGE, 0.0f, 1.0f));
				
				//	report how much we wrote
				*outDataSize = sizeof(Float32);
				return kAudioHardwareNoError;
			}
		}
	} else {
		// we are a mute control
		switch(inAddress->mSelector)
		{
			case kAudioObjectPropertyBaseClass:
				//	The base class for kAudioMuteControlClassID is kAudioBooleanControlClassID
				DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the mute control");
				*((AudioClassID*)outData) = kAudioBooleanControlClassID;
				*outDataSize = sizeof(AudioClassID);
				return kAudioHardwareNoError;
				
			case kAudioObjectPropertyClass:
				//	Mute controls are of the class, kAudioMuteControlClassID
				DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyClass for the mute control");
				*((AudioClassID*)outData) = kAudioMuteControlClassID;
				*outDataSize = sizeof(AudioClassID);
				return kAudioHardwareNoError;

			case kAudioBooleanControlPropertyValue:
				//	This returns the value of the mute control where 0 means that mute is off
				//	and audio can be heard and 1 means that mute is on and audio cannot be heard.
				//	Note that we need to take the state lock to examine this value.
				DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioBooleanControlPropertyValue for the mute control");
				*((UInt32*)outData) = (UInt32) atomic_load_explicit(&OUTPUT_MUTE[ctrl_index], __ATOMIC_RELAXED);
				*outDataSize = sizeof(UInt32);
				return kAudioHardwareNoError;
		}
	}

	return kAudioHardwareUnknownPropertyError;
}

static OSStatus	syfala_SetControlPropertyData(
	AudioObjectID const inObjectID,
	AudioObjectPropertyAddress const* const inAddress,
	UInt32 const inDataSize,
	void const* const inData,
	UInt32* const outNumberPropertiesChanged,
	AudioObjectPropertyAddress outChangedAddresses[2]
) {
	//	initialize the returned number of changed properties
	*outNumberPropertiesChanged = 0;

	AudioObjectPropertyElement const ctrl_index = control_channel_index(inObjectID);

	if (control_is_volume(inObjectID)) {
		if (inAddress->mSelector == kAudioLevelControlPropertyScalarValue) {
			// Note that if this value changes, it is implied that the dB value changed too.
			DoIfFailed(inDataSize != sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_SetControlPropertyData: wrong size for the data for kAudioLevelControlPropertyScalarValue");

			Float32 const new_vol_norm = sqrtf(fminf(fmaxf(*((Float32 const*)inData), 0.0f), 1.0f));
			Float32 const new_vol_db = linear_remap(new_vol_norm, 0.0f, 1.0f, VOL_MIN_DB, VOL_DB_RANGE);
			Float32 const new_vol_gain = db_to_gain(new_vol_db);

			atomic_store_explicit(&OUTPUT_VOLUME[ctrl_index], new_vol_gain, __ATOMIC_RELAXED);

			*outNumberPropertiesChanged = 2;

			AudioObjectPropertyAddress* const first_addr = &outChangedAddresses[0];
			first_addr->mSelector = kAudioLevelControlPropertyScalarValue;
			first_addr->mScope = kAudioObjectPropertyScopeOutput;
			first_addr->mElement = ctrl_index;

			AudioObjectPropertyAddress* const second_addr = &outChangedAddresses[1];
			second_addr->mSelector = kAudioLevelControlPropertyDecibelValue;
			second_addr->mScope = kAudioObjectPropertyScopeOutput;
			second_addr->mElement = ctrl_index;

			return kAudioHardwareNoError;

		} else if (inAddress->mSelector == kAudioLevelControlPropertyDecibelValue) {
			//	Note that if this value changes, it is implied that the scalar value changes as well.
			DoIfFailed(inDataSize != sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_SetControlPropertyData: wrong size for the data for kAudioLevelControlPropertyScalarValue");
			
			Float32 const new_vol_db = fminf(fmaxf(*((const Float32*)inData), VOL_MIN_DB), VOL_MAX_DB);
			Float32 const new_vol_gain = db_to_gain(new_vol_db);
			
			atomic_store_explicit(&OUTPUT_VOLUME[ctrl_index], new_vol_gain, __ATOMIC_RELAXED);
			
			*outNumberPropertiesChanged = 2;

			AudioObjectPropertyAddress* const first_addr = &outChangedAddresses[0];
			first_addr->mSelector = kAudioLevelControlPropertyDecibelValue;
			first_addr->mScope = kAudioObjectPropertyScopeOutput;
			first_addr->mElement = ctrl_index;

			AudioObjectPropertyAddress* const second_addr = &outChangedAddresses[1];
			second_addr->mSelector = kAudioLevelControlPropertyScalarValue;
			second_addr->mScope = kAudioObjectPropertyScopeOutput;
			second_addr->mElement = ctrl_index;

			return kAudioHardwareNoError;
		}

		return kAudioHardwareUnknownPropertyError;
	} else {
		if (inAddress->mSelector ==  kAudioBooleanControlPropertyValue) {
			DoIfFailed(inDataSize != sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_SetControlPropertyData: wrong size for the data for kAudioBooleanControlPropertyValue");

			atomic_store_explicit(&OUTPUT_MUTE[ctrl_index], *((const UInt32*)inData) != 0, __ATOMIC_RELAXED);

			*outNumberPropertiesChanged = 1;

			AudioObjectPropertyAddress* const addr = &outChangedAddresses[0];
			addr->mSelector = kAudioBooleanControlPropertyValue;
			addr->mScope = kAudioObjectPropertyScopeOutput;
			addr->mElement = kAudioObjectPropertyElementMain;

			return kAudioHardwareNoError;
		}

		return kAudioHardwareUnknownPropertyError;
	}
}

#pragma mark IO Operations

static OSStatus	syfala_StartIO(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	UInt32 const inClientID
) {
	DebugMsg("StartIO");

	//	This call tells the device that IO is starting for the given client. When this routine
	//	returns, the device's clock is running and it is ready to have data read/written. It is
	//	important to note that multiple clients can have IO running on the device at the same time.
	//	So, work only needs to be done when the first client starts. All subsequent starts simply
	//	increment the counter.
	
	#pragma unused(inClientID)

	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_StartIO: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_StartIO: bad device ID");

	//	we need to hold the state lock
	pthread_mutex_lock(&gDevice_IOMutex);
	
	//	figure out what we need to do
	if(gDevice_IOIsRunning == UINT32_MAX) {
		//	overflowing is an error
		return kAudioHardwareIllegalOperationError;
	} else if(gDevice_IOIsRunning == 0) {
		//	We need to start the hardware, which in this case is just anchoring the time line.
		gDevice_IOIsRunning = 1;
		gDevice_NumberTimeStamps = 0;
		gDevice_AnchorSampleTime = 0;
		gDevice_AnchorHostTime = mach_absolute_time();
	} else {
		//	IO is already running, so just bump the counter
		++gDevice_IOIsRunning;
	}
	
	//	unlock the state lock
	pthread_mutex_unlock(&gDevice_IOMutex);
	
	return kAudioHardwareNoError;
}

static OSStatus	syfala_StopIO(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	UInt32 const inClientID
) {
	DebugMsg("StopIO");	

	//	This call tells the device that the client has stopped IO. The driver can stop the hardware
	//	once all clients have stopped.
	
	#pragma unused(inClientID)

	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_StopIO: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_StopIO: bad device ID");

	//	we need to hold the state lock
	pthread_mutex_lock(&gDevice_IOMutex);
	
	//	figure out what we need to do
	if(gDevice_IOIsRunning == 0) {
		//	underflowing is an error
		return kAudioHardwareIllegalOperationError;
	} else if(gDevice_IOIsRunning == 1) {
		//	We need to stop the hardware, which in this case means that there's nothing to do.
		gDevice_IOIsRunning = 0;
	} else {
		//	IO is still running, so just bump the counter
		--gDevice_IOIsRunning;
	}
	
	//	unlock the state lock
	pthread_mutex_unlock(&gDevice_IOMutex);
	
	return kAudioHardwareNoError;
}

static OSStatus	syfala_GetZeroTimeStamp(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	UInt32 const inClientID,
	Float64* const outSampleTime,
	UInt64* const outHostTime,
	UInt64* const outSeed
) {
	DebugMsg("GetZeroTimeStamp");	

	//	This method returns the current zero time stamp for the device. The HAL models the timing of
	//	a device as a series of time stamps that relate the sample time to a host time. The zero
	//	time stamps are spaced such that the sample times are the value of
	//	kAudioDevicePropertyZeroTimeStampPeriod apart. This is often modeled using a ring buffer
	//	where the zero time stamp is updated when wrapping around the ring buffer.
	//
	//	For this device, the zero time stamps' sample time increments every kDevice_RingBufferSize
	//	frames and the host time increments by kDevice_RingBufferSize * gDevice_HostTicksPerFrame.
	
	#pragma unused(inClientID)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_GetZeroTimeStamp: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_GetZeroTimeStamp: bad device ID");

	//	we need to hold the locks
	pthread_mutex_lock(&gDevice_IOMutex);
	
	//	get the current host time
	UInt64 const theCurrentHostTime = mach_absolute_time();
	
	//	calculate the next host time
	Float64 const theHostTicksPerRingBuffer = gDevice_HostTicksPerFrame * ((Float64)kDevice_RingBufferSize);
	Float64 const theHostTickOffset = ((Float64)(gDevice_NumberTimeStamps + 1)) * theHostTicksPerRingBuffer;
	UInt64 const theNextHostTime = gDevice_AnchorHostTime + ((UInt64)theHostTickOffset);
	
	//	go to the next time if the next host time is less than the current time
	if(theNextHostTime <= theCurrentHostTime) ++gDevice_NumberTimeStamps;
	
	//	set the return values
	*outSampleTime = gDevice_NumberTimeStamps * kDevice_RingBufferSize;
	*outHostTime = gDevice_AnchorHostTime + (((Float64)gDevice_NumberTimeStamps) * theHostTicksPerRingBuffer);
	*outSeed = 1;
	
	//	unlock the state lock
	pthread_mutex_unlock(&gDevice_IOMutex);
	
	return kAudioHardwareNoError;
}

static OSStatus	syfala_WillDoIOOperation(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	UInt32 const inClientID,
	UInt32 const inOperationID,
	Boolean* const outWillDo,
	Boolean* const outWillDoInPlace
) {
	DebugMsg("WillDoIOOperation");	

	//	This method returns whether or not the device will do a given IO operation. For this device,
	//	we only support reading input data and writing output data.
	
	#pragma unused(inClientID)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_WillDoIOOperation: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_WillDoIOOperation: bad device ID");

	//	figure out if we support the operation
	bool willDo = false;
	bool willDoInPlace = true;
	
	if (inOperationID == kAudioServerPlugInIOOperationWriteMix) {
		willDo = true;
		willDoInPlace = true;
	}
	
	//	fill out the return values
	if(outWillDo != NULL) *outWillDo = willDo;
	if(outWillDoInPlace != NULL) *outWillDoInPlace = willDoInPlace;
	return kAudioHardwareNoError;
}

static OSStatus	syfala_BeginIOOperation(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	UInt32 const inClientID,
	UInt32 const inOperationID,
	UInt32 const inIOBufferFrameSize,
	AudioServerPlugInIOCycleInfo const* const inIOCycleInfo
) {	
	//This is called at the beginning of an IO operation. This device doesn't do anything, so just
	// check args and return.

	DebugMsg("BeginIOOperation");
	
	#pragma unused(inClientID, inOperationID, inIOBufferFrameSize, inIOCycleInfo)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_BeginIOOperation: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_BeginIOOperation: bad device ID");

	return kAudioHardwareNoError;
}

static OSStatus	syfala_DoIOOperation(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	AudioObjectID const inStreamObjectID,
	UInt32 const inClientID,
	UInt32 const inOperationID,
	UInt32 const inIOBufferFrameSize,
	AudioServerPlugInIOCycleInfo const* const inIOCycleInfo,
	void* const ioMainBuffer,
	void* const ioSecondaryBuffer
) {
	//	This is called to actually perform a given operation. For this device, all we need to do is
	//	clear the buffer for the ReadInput operation.

	DebugMsg("DoIOOperation");
	
	#pragma unused(inClientID, inIOCycleInfo, ioSecondaryBuffer)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_DoIOOperation: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_DoIOOperation: bad device ID");
	DoIfFailed(inStreamObjectID != STREAM_ID, return kAudioHardwareBadObjectError, "syfala_DoIOOperation: bad stream ID");

	if(inOperationID == kAudioServerPlugInIOOperationWriteMix) {}

	return kAudioHardwareNoError;
}

static OSStatus	syfala_EndIOOperation(
	AudioServerPlugInDriverRef const inDriver,
	AudioObjectID const inDeviceObjectID,
	UInt32 const inClientID,
	UInt32 const inOperationID,
	UInt32 const inIOBufferFrameSize,
	AudioServerPlugInIOCycleInfo const* const inIOCycleInfo
) {
	//	This is called at the end of an IO operation. This device doesn't do anything, so just check
	//	the arguments and return.
	DebugMsg("EndIOOperation");
	
	#pragma unused(inClientID, inOperationID, inIOBufferFrameSize, inIOCycleInfo)
	
	// check args
	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_EndIOOperation: bad driver reference");
	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_EndIOOperation: bad device ID");

	return kAudioHardwareNoError;
}
