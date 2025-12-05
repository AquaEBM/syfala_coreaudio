use core::{
    mem, ptr,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use std::sync::Mutex;

use atomic_float::AtomicF32;
use coreaudio_sys::*;
use oslog::OsLogger;

const N_CHANNELS: UInt32 = 16;
const SAMPLE_RATE: Float64 = 48000.;

// A volume and mute control pair for each channel + a master pair
const N_CONTROLS: UInt32 = N_CHANNELS.strict_add(1).strict_mul(2);
// N_CONTROLS + the output stream
const N_DEVICE_OWNED_OBJS: UInt32 = N_CONTROLS.strict_add(1);

// The _only_ format we support (Note that it is interleaved by default)
const FORMAT: AudioStreamRangedDescription = AudioStreamRangedDescription {
    mFormat: AudioStreamBasicDescription {
        mSampleRate: SAMPLE_RATE,
        mFormatID: kAudioFormatLinearPCM,
        mFormatFlags: kAudioFormatFlagsNativeFloatPacked,
        mBytesPerPacket: N_CHANNELS.strict_mul(size_of::<Float32>() as UInt32),
        mFramesPerPacket: 1,
        mBytesPerFrame: N_CHANNELS.strict_mul(size_of::<Float32>() as UInt32),
        mChannelsPerFrame: N_CHANNELS,
        mBitsPerChannel: 8u32.strict_mul(size_of::<Float32>() as UInt32),
        mReserved: 0,
    },

    mSampleRateRange: AudioValueRange {
        mMaximum: SAMPLE_RATE,
        mMinimum: SAMPLE_RATE,
    },
};

// The Audio Object IDs we expose to the HAL

const PLUGIN_ID: AudioObjectID = kAudioObjectPlugInObject; /* AKA 1 */
const DEVICE_ID: AudioObjectID = 2;
const STREAM_ID: AudioObjectID = 3;

// We also expose IDs 4 through (3 + N_CONTROLS). Those represent the volume/mute controls
// Note that 0 is a sentinel/unknown value and usually represents the host/HAL itself

const FIRST_CTRL_ID: AudioObjectID = 4;
const LAST_CTRL_ID: AudioObjectID = N_CONTROLS.strict_add(3);

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

#[inline(always)]
fn audio_object_is_control(id: AudioObjectID) -> bool {
    // All our controls' IDs are consecutive...
    FIRST_CTRL_ID <= id && id <= LAST_CTRL_ID
}

/// Assumes that `audio_object_is_control(id)` holds. Otherwise, results are inconsistent
// TODO: now that we're using rust, we can do better than that
#[inline(always)]
fn control_is_volume(id: AudioObjectID) -> bool {
    (id & 1) != 0
}

/// Assumes that `audio_object_is_control(id)` holds.  Otherwise, results are inconsistent.
/// // TODO: now that we're using rust, we can do better than that
#[inline(always)]
fn control_channel_index(id: AudioObjectID) -> UInt32 {
    (id.strict_sub(FIRST_CTRL_ID)) / 2
}

#[inline(always)]
fn sq(x: Float32) -> Float32 {
    x * x
}

#[inline(always)]
fn db_to_gain(db: Float32) -> Float32 {
    // huh... no exp10?
    f32::powf(10., db * 0.05)
}

#[inline(always)]
fn gain_to_db(gain: Float32) -> Float32 {
    10. * f32::log10(sq(gain))
}

#[inline(always)]
fn linear_remap(
    x: Float32,
    x_start: Float32,
    x_len: Float32,
    y_start: Float32,
    y_len: Float32,
) -> Float32 {
    // FP equality check sry lol
    if x_len == 0. {
        return x_start;
    }
    return f32::mul_add(x - x_start, y_len / x_len, y_start);
}

// Increment an atomic counter, saturating at numerical limits
#[inline(always)]
fn saturating_refount_inc(i: &AtomicU32) -> u32 {
    let mut old = i.load(Ordering::Relaxed);
    loop {
        let Some(new) = old.checked_add(1) else {
            return old;
        };

        let Err(changed) = i.compare_exchange_weak(old, new, Ordering::Relaxed, Ordering::Relaxed)
        else {
            return old;
        };

        old = changed;
    }
}

// Decrement an atomic counter, saturating at numerical limits
#[inline(always)]
fn saturating_refount_dec(i: &AtomicU32) -> u32 {
    let mut old = i.load(Ordering::Relaxed);
    loop {
        let Some(new) = old.checked_sub(1) else {
            break;
        };

        let Err(changed) = i.compare_exchange_weak(old, new, Ordering::Relaxed, Ordering::Relaxed)
        else {
            break;
        };

        old = changed;
    }
    old
}

static mut HOST: Option<&AudioServerPlugInHostInterface> = None;

// We redefine the driver interface from the coreaudio bindings because the generated one isn't
// Sync (can't be put in statics), because of the _reserved field (raw pointer). so we redefine
// it and unsafe impl Sync {}

#[repr(C)]
pub struct AudioServerPlugInDriverInterface {
    pub _reserved: *mut std::os::raw::c_void,
    pub query_interface:
        Option<unsafe extern "C" fn(*mut std::os::raw::c_void, REFIID, *mut LPVOID) -> HRESULT>,
    pub add_ref: Option<unsafe extern "C" fn(*mut std::os::raw::c_void) -> ULONG>,
    pub release: Option<unsafe extern "C" fn(*mut std::os::raw::c_void) -> ULONG>,
    pub initialize: Option<
        unsafe extern "C" fn(AudioServerPlugInDriverRef, AudioServerPlugInHostRef) -> OSStatus,
    >,
    pub create_device: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            CFDictionaryRef,
            *const AudioServerPlugInClientInfo,
            *mut AudioObjectID,
        ) -> OSStatus,
    >,
    pub destroy_device:
        Option<unsafe extern "C" fn(AudioServerPlugInDriverRef, AudioObjectID) -> OSStatus>,
    pub add_device_client: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            *const AudioServerPlugInClientInfo,
        ) -> OSStatus,
    >,
    pub remove_device_client: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            *const AudioServerPlugInClientInfo,
        ) -> OSStatus,
    >,
    pub perform_device_configuration_change: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            UInt64,
            *mut std::os::raw::c_void,
        ) -> OSStatus,
    >,
    pub abort_device_configuration_change: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            UInt64,
            *mut std::os::raw::c_void,
        ) -> OSStatus,
    >,
    pub has_property: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            pid_t,
            *const AudioObjectPropertyAddress,
        ) -> Boolean,
    >,
    pub is_property_settable: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            pid_t,
            *const AudioObjectPropertyAddress,
            *mut Boolean,
        ) -> OSStatus,
    >,
    pub get_property_data_size: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            pid_t,
            *const AudioObjectPropertyAddress,
            UInt32,
            *const std::os::raw::c_void,
            *mut UInt32,
        ) -> OSStatus,
    >,
    pub get_property_data: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            pid_t,
            *const AudioObjectPropertyAddress,
            UInt32,
            *const std::os::raw::c_void,
            UInt32,
            *mut UInt32,
            *mut std::os::raw::c_void,
        ) -> OSStatus,
    >,
    pub set_property_data: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            pid_t,
            *const AudioObjectPropertyAddress,
            UInt32,
            *const std::os::raw::c_void,
            UInt32,
            *const std::os::raw::c_void,
        ) -> OSStatus,
    >,
    pub start_io:
        Option<unsafe extern "C" fn(AudioServerPlugInDriverRef, AudioObjectID, UInt32) -> OSStatus>,
    pub stop_io:
        Option<unsafe extern "C" fn(AudioServerPlugInDriverRef, AudioObjectID, UInt32) -> OSStatus>,
    pub get_zero_time_stamp: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            UInt32,
            *mut Float64,
            *mut UInt64,
            *mut UInt64,
        ) -> OSStatus,
    >,
    pub will_do_iooperation: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            UInt32,
            UInt32,
            *mut Boolean,
            *mut Boolean,
        ) -> OSStatus,
    >,
    pub begin_iooperation: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            UInt32,
            UInt32,
            UInt32,
            *const AudioServerPlugInIOCycleInfo,
        ) -> OSStatus,
    >,
    pub do_iooperation: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            AudioObjectID,
            UInt32,
            UInt32,
            UInt32,
            *const AudioServerPlugInIOCycleInfo,
            *mut std::os::raw::c_void,
            *mut std::os::raw::c_void,
        ) -> OSStatus,
    >,
    pub end_iooperation: Option<
        unsafe extern "C" fn(
            AudioServerPlugInDriverRef,
            AudioObjectID,
            UInt32,
            UInt32,
            UInt32,
            *const AudioServerPlugInIOCycleInfo,
        ) -> OSStatus,
    >,
}

unsafe impl Sync for AudioServerPlugInDriverInterface {}

// All of the plugin's methods are exposed through this struct, which we return a reference to in the factory function
static DRIVER_INTERFACE: AudioServerPlugInDriverInterface = AudioServerPlugInDriverInterface {
    _reserved: core::ptr::null_mut(),
    query_interface: Some(syfala_query_interface),
    add_ref: Some(syfala_add_ref),
    release: Some(syfala_release),
    initialize: Some(syfala_initialize),
    create_device: Some(syfala_create_device),
    destroy_device: Some(syfala_destroy_device),
    add_device_client: Some(syfala_add_device_client),
    remove_device_client: Some(syfala_remove_device_client),
    perform_device_configuration_change: Some(syfala_perform_device_configuration_change),
    abort_device_configuration_change: Some(syfala_abort_device_configuration_change),
    has_property: Some(syfala_HasProperty),
    is_property_settable: Some(syfala_IsPropertySettable),
    get_property_data_size: Some(syfala_GetPropertyDataSize),
    get_property_data: Some(syfala_GetPropertyData),
    set_property_data: Some(syfala_SetPropertyData),
    start_io: Some(syfala_StartIO),
    stop_io: Some(syfala_StopIO),
    get_zero_time_stamp: Some(syfala_GetZeroTimeStamp),
    will_do_iooperation: Some(syfala_WillDoIOOperation),
    begin_iooperation: Some(syfala_BeginIOOperation),
    do_iooperation: Some(syfala_DoIOOperation),
    end_iooperation: Some(syfala_EndIOOperation),
};

static DRIVER_OBJECT: &AudioServerPlugInDriverInterface = &DRIVER_INTERFACE;

const PLUGIN_BUNDLE_ID: &str = "com.emeraude.syfala_coreaudio";

static PLUGIN_REF_COUNT: AtomicU32 = AtomicU32::new(0);
static IS_OUTPUT_STREAM_ACTIVE: AtomicBool = AtomicBool::new(true);

const DEVICE_UID: &str = "syfala_UID";

const DEVICE_MODEL_UID: &str = "syfala_ModelUID";

const DEVICE_RING_BUF_SIZE: UInt32 = 16384;

struct IOInfo {
    is_running: UInt32,
    host_ticks_per_frame: Float64,
    n_timestamps: UInt64,
    anchor_sample_time: Float64,
    anchor_host_time: UInt64,
}

static DEVICE_IO: Mutex<IOInfo> = Mutex::new(IOInfo {
    is_running: 0,
    host_ticks_per_frame: 0.,
    n_timestamps: 0,
    anchor_sample_time: 0.,
    anchor_host_time: 0,
});

const VOL_MIN_DB: Float32 = -80.;
const VOL_MAX_DB: Float32 = 0.;
const VOL_DB_RANGE: Float32 = VOL_MAX_DB - VOL_MIN_DB;

static OUTPUT_VOLUME: [AtomicF32; N_CHANNELS.strict_add(1) as usize] =
    [const { AtomicF32::new(1.) }; _];
static OUTPUT_MUTE: [AtomicBool; N_CHANNELS.strict_add(1) as usize] =
    [const { AtomicBool::new(false) }; _];

// These IDs are defined as macros in the coreaudio headers, bindgen can't generate them so we do
// it ourselves. We also can't use constants sincee they are created using extern functions

// kAudioServerPlugInTypeUUID
#[inline(always)]
fn asp_type_uuid_macro() -> CFUUIDRef {
    // SAFETY: arguments are valid
    unsafe {
        CFUUIDGetConstantUUIDWithBytes(
            core::ptr::null(),
            0x44,
            0x3A,
            0xBA,
            0xB8,
            0xE7,
            0xB3,
            0x49,
            0x1A,
            0xB9,
            0x85,
            0xBE,
            0xB9,
            0x18,
            0x70,
            0x30,
            0xDB,
        )
    }
}

// IUnknownUUID
#[inline(always)]
fn i_unknown_uuid_macro() -> CFUUIDRef {
    // SAFETY: arguments are valid
    unsafe {
        CFUUIDGetConstantUUIDWithBytes(
            kCFAllocatorSystemDefault,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0xC0,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x46,
        )
    }
}

// kAudioServerPlugInDriverInterfaceUUID
#[inline(always)]
fn asp_driver_interface_type_uuid_macro() -> CFUUIDRef {
    // SAFETY: arguments are valid
    unsafe {
        CFUUIDGetConstantUUIDWithBytes(
            core::ptr::null_mut(),
            0xEE,
            0xA5,
            0x77,
            0x3D,
            0xCC,
            0x43,
            0x49,
            0xF1,
            0x8E,
            0x00,
            0x8F,
            0x96,
            0xE7,
            0xD2,
            0x3B,
            0x17,
        )
    }
}

#[inline(always)]
fn cfstr(str: &'static str) -> CFStringRef {
    unsafe { __CFStringMakeConstantString(str.as_ptr().cast()) }
}

#[inline(always)]
const fn raw_driver_ptr() -> *mut std::os::raw::c_void {
    (&DRIVER_OBJECT as *const &AudioServerPlugInDriverInterface)
        .cast_mut()
        .cast()
}

// This is the only function exported as a symbol by the library, It's name must be indicated in
// the CFPlugInFactories field the bundle's Info.plist file.
#[unsafe(no_mangle)]
unsafe extern "C" fn syfala_create(
    _in_allocator: CFAllocatorRef,
    in_requested_type_uuid: CFUUIDRef,
) -> *mut std::os::raw::c_void {
    //	This is the CFPlugIn factory function. Its job is to create the implementation for the given
    //	type provided that the type is supported. Because this driver is simple and all its
    //	initialization is handled via static iniitalization when the bundle is loaded, all that
    //	needs to be done is to return the AudioServerPlugInDriverRef that points to the driver's
    //	interface. A more complicated driver would create any base line objects it needs to satisfy
    //	the IUnknown methods that are used to discover that actual interface to talk to the driver.
    //	The majority of the driver's initilization should be handled in the Initialize() method of
    //	the driver's AudioServerPlugInDriverInterface.

    log::set_boxed_logger(Box::new(OsLogger::new("com.emeraude.syfala_coreaudio")));

    OUTPUT_VOLUME[0].store(0.001, Ordering::Relaxed);

    log::debug!("Factory");

    let asp_type_uuid = asp_type_uuid_macro();

    let mut ret = core::ptr::null_mut();
    // SAFETY: arguments are valid
    if unsafe { CFEqual(in_requested_type_uuid.cast(), asp_type_uuid.cast()) } != 0 {
        log::debug!("HAL Plug-In Interface Requested");
        ret = raw_driver_ptr();
    }
    ret
}

#[inline(always)]
fn eq_driver_ptr(driver: *const std::os::raw::c_void) -> bool {
    ptr::eq(driver, raw_driver_ptr())
}

#[inline(always)]
const fn cfuuid_as_bytes(uuid: &CFUUIDBytes) -> &[u8; 16] {
    // SAFETY: CFUUIDBytes has the same layout as [u8 ; 16] (#[repr(C)])
    // lifetimes are correctly inferred
    unsafe { mem::transmute(uuid) }
}

// another macro we have to redefine
// E_NOINTERFACE
const E_NOINTERFACE: HRESULT = 0x80000004;

unsafe extern "C" fn syfala_query_interface(
    driver: *mut std::os::raw::c_void,
    in_uuid: REFIID,
    out_interface: *mut LPVOID,
) -> HRESULT {
    //	This function is called by the HAL to get the interface to talk to the plug-in through.
    //	AudioServerPlugIns are required to support the IUnknown interface and the
    //	AudioServerPlugInDriverInterface. As it happens, all interfaces must also provide the
    //	IUnknown interface, so we can always just return the single interface we made with
    //	DRIVER_OBJECT regardless of which one is asked for.

    log::debug!("QueryInterface");

    let i_unknown_uuid = i_unknown_uuid_macro();
    let asp_driver_interface_type_uuid = asp_driver_interface_type_uuid_macro();

    // SAFETY: arguments are valid (passed by the HAL)
    let iunknown_uuid_bytes: CFUUIDBytes = unsafe { CFUUIDGetUUIDBytes(i_unknown_uuid) };
    let audio_server_plug_in_driver_interface_uuid_bytes: CFUUIDBytes =
        unsafe { CFUUIDGetUUIDBytes(asp_driver_interface_type_uuid) };

    if !eq_driver_ptr(driver) {
        log::error!("syfala_query_interface: bad driver reference");
        return kAudioHardwareBadObjectError as HRESULT;
    }

    if out_interface.is_null() {
        log::error!("syfala_query_interface: no place to store the returned interface");
        return kAudioHardwareIllegalOperationError as HRESULT;
    }

    let in_uuid_bytes = cfuuid_as_bytes(&in_uuid);

    //	AudioServerPlugIns only support two interfaces, IUnknown (which has to be supported by all
    //	CFPlugIns and AudioServerPlugInDriverInterface (which is the actual interface the HAL will
    //	use).
    if in_uuid_bytes == cfuuid_as_bytes(&iunknown_uuid_bytes)
        || in_uuid_bytes == cfuuid_as_bytes(&audio_server_plug_in_driver_interface_uuid_bytes)
    {
        saturating_refount_inc(&PLUGIN_REF_COUNT);
        unsafe { out_interface.write(raw_driver_ptr()) };
    } else {
        return E_NOINTERFACE;
    }

    return kAudioHardwareNoError as HRESULT;
}

unsafe extern "C" fn syfala_add_ref(driver: *mut std::os::raw::c_void) -> ULONG {
    //	This call returns the resulting reference count after the increment.

    log::debug!("AddRef");

    // check args
    if !eq_driver_ptr(driver) {
        log::error!("syfala_add_ref: bad driver reference");
        return 0 as ULONG;
    }

    //	increment the refcount, return the new value
    return saturating_refount_inc(&PLUGIN_REF_COUNT).saturating_add(1);
}

unsafe extern "C" fn syfala_release(driver: *mut std::os::raw::c_void) -> ULONG {
    //	This call returns the resulting reference count after the decrement.
    log::debug!("Release");

    // check args
    if !eq_driver_ptr(driver) {
        log::error!("syfala_add_ref: bad driver reference");
        return 0 as ULONG;
    }

    // decrement the refcount, return the new value
    return saturating_refount_dec(&PLUGIN_REF_COUNT).saturating_sub(1);
}

unsafe extern "C" fn syfala_initialize(
    driver: AudioServerPlugInDriverRef,
    host: AudioServerPlugInHostRef,
) -> OSStatus {
    //	The job of this method is, as the name implies, to get the driver initialized. Note that when this call returns, the HAL will scan the various lists the driver
    //	maintains (such as the device list) to get the inital set of objects the driver is
    //	publishing. So, there is no need to notifiy the HAL about any objects created as part of the
    //	execution of this method.

    log::debug!("Initialize");

    if !eq_driver_ptr(driver.cast()) {
        log::error!("syfala_initialize: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    // One specific thing that needs to be done is to store the AudioServerPlugInHostRef
    // so that it can be used later. This is the object used by our plugin to notify the host
    // of property changes etc.
    // SAFETY: the HAL guarantees this isn't called concurrently, and that `host` outlives
    // our whole plugin ('static lifetime)
    unsafe { HOST = host.as_ref() };

    #[repr(C)]
    #[allow(non_camel_case_types)]
    pub struct mach_timebase_info_data_t {
        pub numer: u32,
        pub denom: u32,
    }

    unsafe extern "C" {
        pub safe fn mach_timebase_info(info: *mut mach_timebase_info_data_t) -> i32;
    }

    //	calculate the host ticks per frame
    let mut the_timebase_info = mach_timebase_info_data_t { numer: 0, denom: 0 };
    mach_timebase_info(ptr::from_mut(&mut the_timebase_info));
    let host_clock_freq = the_timebase_info.denom as Float64 / the_timebase_info.numer as Float64;
    DEVICE_IO.lock().unwrap().host_ticks_per_frame = host_clock_freq * (1000000000. / SAMPLE_RATE);

    return kAudioHardwareNoError as OSStatus;
}

unsafe extern "C" fn syfala_create_device(
    driver: AudioServerPlugInDriverRef,
    _description: CFDictionaryRef,
    _client_info: *const AudioServerPlugInClientInfo,
    _device_object_id: *mut AudioObjectID,
) -> OSStatus {
    //	This method is used to tell a driver that implements the Transport Manager semantics to
    //	create an AudioEndpointDevice from a set of AudioEndpoints. Since this driver is not a
    //	Transport Manager, we just check the arguments and return
    //	kAudioHardwareUnsupportedOperationError.

    log::debug!("CreateDevice");

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("syfala_create_device: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    return kAudioHardwareUnsupportedOperationError as OSStatus;
}

unsafe extern "C" fn syfala_destroy_device(
    driver: AudioServerPlugInDriverRef,
    _device_object_id: AudioObjectID,
) -> OSStatus {
    //	This method is used to tell a driver that implements the Transport Manager semantics to
    //	destroy an AudioEndpointDevice. Since this driver is not a Transport Manager, we just check
    //	the arguments and return kAudioHardwareUnsupportedOperationError.

    log::debug!("DestroyDevice");

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("syfala_destroy_device: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    return kAudioHardwareUnsupportedOperationError as OSStatus;
}

unsafe extern "C" fn syfala_add_device_client(
    driver: AudioServerPlugInDriverRef,
    device_object_id: AudioObjectID,
    _client_info: *const AudioServerPlugInClientInfo,
) -> OSStatus {
    //	This method is used to inform the driver about a new client that is using the given device.
    //	This allows the device to act differently depending on who the client is. This driver does
    //	not need to track the clients using the device, so we just check the arguments and return
    //	successfully.

    log::debug!("AddDeviceClient");

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("syfala_add_device_client: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_object_id != DEVICE_ID {
        log::error!("syfala_add_device_client: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    return kAudioHardwareNoError as OSStatus;
}

unsafe extern "C" fn syfala_remove_device_client(
    driver: AudioServerPlugInDriverRef,
    device_object_id: AudioObjectID,
    _client_info: *const AudioServerPlugInClientInfo,
) -> OSStatus {
    //	This method is used to inform the driver about a client that is no longer using the given
    //	device. This driver does not track clients, so we just check the arguments and return
    //	successfully.

    log::debug!("RemoveDeviceClient");

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("syfala_remove_device_client: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_object_id != DEVICE_ID {
        log::error!("syfala_remove_device_client: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    return kAudioHardwareNoError as OSStatus;
}

unsafe extern "C" fn syfala_perform_device_configuration_change(
    driver: AudioServerPlugInDriverRef,
    device_object_id: AudioObjectID,
    _change_action: UInt64,
    _change_info: *mut std::os::raw::c_void,
) -> OSStatus {
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

    log::debug!("PerformDeviceConfigurationChange");

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("syfala_perform_device_configuration_change: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_object_id != DEVICE_ID {
        log::error!("syfala_perform_device_configuration_change: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    return kAudioHardwareNoError as OSStatus;
}

unsafe extern "C" fn syfala_abort_device_configuration_change(
    driver: AudioServerPlugInDriverRef,
    device_object_id: AudioObjectID,
    _change_action: UInt64,
    _change_info: *mut std::os::raw::c_void,
) -> OSStatus {
    //	This method is called to tell the driver that a request for a config change has been denied.
    //	This provides the driver an opportunity to clean up any state associated with the request.
    //	For this driver, an aborted config change requires no action. So we just check the arguments
    //	and return

    log::debug!("AbortDeviceConfigurationChange");

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("syfala_abort_device_configuration_change: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_object_id != DEVICE_ID {
        log::error!("syfala_abort_device_configuration_change: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    return kAudioHardwareNoError as OSStatus;
}

// static Boolean	syfala_HasProperty(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inObjectID,
// 	pid_t const inClientProcessID,
// 	AudioObjectPropertyAddress const* const inAddress
// ) {
// 	char const selector[5] = FourCCToCString(inAddress->mSelector);
//     char const scope[5] = FourCCToCString(inAddress->mScope);

//     DebugMsg(
// 		"Has       : %02u { %s %s %02d } by %08x",
// 		inObjectID,
// 		selector,
// 		scope,
// 		inAddress->mElement,
// 		inClientProcessID
// 	);

// 	//	check args
// 	DoIfFailed(inDriver != DRIVER_REF, return false, "syfala_HasProperty: bad driver reference");
// 	DoIfFailed(inAddress == NULL, return false, "syfala_HasProperty: no address");

// 	switch(inObjectID)
// 	{
// 		case PLUGIN_ID:
// 			return syfala_HasPlugInProperty(inAddress);

// 		case DEVICE_ID:
// 			return syfala_HasDeviceProperty(inAddress);

// 		case STREAM_ID:
// 			return syfala_HasStreamProperty(inAddress);
// 	}

// 	return audio_object_is_control(inObjectID) && syfala_HasControlProperty(inObjectID, inAddress);
// }

// static OSStatus	syfala_IsPropertySettable(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inObjectID,
// 	pid_t const inClientProcessID,
// 	AudioObjectPropertyAddress const* const inAddress,
// 	Boolean* const outIsSettable
// ) {
// 	char const selector[5] = FourCCToCString(inAddress->mSelector);
//     char const scope[5] = FourCCToCString(inAddress->mScope);

//     DebugMsg(
// 		"IsSettable: %02u { %s %s %02d } by %08x",
// 		inObjectID,
// 		selector,
// 		scope,
// 		inAddress->mElement,
// 		inClientProcessID
// 	);

// 	//	check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_IsPropertySettable: bad driver reference");
// 	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "syfala_IsPropertySettable: no address");
// 	DoIfFailed(outIsSettable == NULL, return kAudioHardwareIllegalOperationError, "syfala_IsPropertySettable: no place to put the return value");

// 	switch(inObjectID)
// 	{
// 		case PLUGIN_ID:
// 			return syfala_IsPlugInPropertySettable(inAddress, outIsSettable);

// 		case DEVICE_ID:
// 			return syfala_IsDevicePropertySettable(inAddress, outIsSettable);

// 		case STREAM_ID:
// 			return syfala_IsStreamPropertySettable(inAddress, outIsSettable);
// 	};

// 	return (audio_object_is_control(inObjectID)) ?
// 		syfala_IsControlPropertySettable(inObjectID, inAddress, outIsSettable) :
// 		kAudioHardwareBadObjectError;
// }

// static OSStatus	syfala_GetPropertyDataSize(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inObjectID,
// 	pid_t const inClientProcessID,
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32 const inQualifierDataSize,
// 	void const* const inQualifierData,
// 	UInt32* const outDataSize
// ) {
// 	char const selector[5] = FourCCToCString(inAddress->mSelector);
//     char const scope[5] = FourCCToCString(inAddress->mScope);

//     DebugMsg(
// 		"DataSize  : %02u { %s %s %02d } by %08x ",
// 		inObjectID,
// 		selector,
// 		scope,
// 		inAddress->mElement,
// 		inClientProcessID
// 	);

// 	//	check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_GetPropertyDataSize: bad driver reference");
// 	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyDataSize: no address");
// 	DoIfFailed(outDataSize == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyDataSize: no place to put the return value");

// 	switch(inObjectID)
// 	{
// 		case PLUGIN_ID:
// 			return syfala_GetPlugInPropertyDataSize(
// 				inAddress,
// 				inQualifierDataSize,
// 				inQualifierData,
// 				outDataSize
// 			);

// 		case DEVICE_ID:
// 			return syfala_GetDevicePropertyDataSize(inAddress, outDataSize);

// 		case STREAM_ID:
// 			return syfala_GetStreamPropertyDataSize(inAddress, outDataSize);
// 	};

// 	return (audio_object_is_control(inObjectID)) ?
// 		syfala_GetControlPropertyDataSize(inObjectID, inAddress, outDataSize) :
// 		kAudioHardwareBadObjectError;
// }

// static OSStatus	syfala_GetPropertyData(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inObjectID,
// 	pid_t const inClientProcessID,
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32 const inQualifierDataSize,
// 	void const* const inQualifierData,
// 	UInt32 const inDataSize,
// 	UInt32* const outDataSize,
// 	void* const outData
// ) {
// 	char const selector[5] = FourCCToCString(inAddress->mSelector);
//     char const scope[5] = FourCCToCString(inAddress->mScope);

//     DebugMsg(
// 		"GetData   : %02u { %s %s %02d } by %08x",
// 		inObjectID,
// 		selector,
// 		scope,
// 		inAddress->mElement,
// 		inClientProcessID
// 	);

// 	//	check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_GetPropertyData: bad driver reference");
// 	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyData: no address");
// 	DoIfFailed(outDataSize == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyData: no place to put the return value size");
// 	DoIfFailed(outData == NULL, return kAudioHardwareIllegalOperationError, "syfala_GetPropertyData: no place to put the return value");

// 	switch(inObjectID)
// 	{
// 		case PLUGIN_ID:
// 			return syfala_GetPlugInPropertyData(
// 				inAddress,
// 				inQualifierDataSize,
// 				inQualifierData,
// 				inDataSize,
// 				outDataSize,
// 				outData
// 			);

// 		case DEVICE_ID:
// 			return syfala_GetDevicePropertyData(inAddress, inDataSize, outDataSize, outData);

// 		case STREAM_ID:
// 			return syfala_GetStreamPropertyData(inAddress, inDataSize, outDataSize, outData);
// 	};

// 	return (audio_object_is_control(inObjectID)) ?
// 		syfala_GetControlPropertyData(inObjectID, inAddress, inDataSize, outDataSize, outData) :
// 		kAudioHardwareBadObjectError;
// }

// static OSStatus	syfala_SetPropertyData(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inObjectID,
// 	pid_t const inClientProcessID,
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32 const inQualifierDataSize,
// 	void const* const inQualifierData,
// 	UInt32 const inDataSize,
// 	void const* const inData
// ) {
// 	char const selector[5] = FourCCToCString(inAddress->mSelector);
//     char const scope[5] = FourCCToCString(inAddress->mScope);

//     DebugMsg(
// 		"SetData   : %02u { %s %s %02d } by %08x",
// 		inObjectID,
// 		selector,
// 		scope,
// 		inAddress->mElement,
// 		inClientProcessID
// 	);

// 	//	check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_SetPropertyData: bad driver reference");
// 	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "syfala_SetPropertyData: no address");

// 	// Only 2 * (N_CHANNELS + 1) + 1 properties are settable in our plugin:
// 	//		- Whether the output stream is active.
// 	// 		- The values of the volume and mule controls for each channel + the master controls

// 	if (inObjectID == STREAM_ID) {
// 		if (inAddress->mSelector == kAudioStreamPropertyIsActive) {
// 			//	Changing the active state of a stream doesn't affect IO or change the structure
// 			//	so we can just save the state and send the notification.
// 			DoIfFailed(inDataSize != sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_SetStreamPropertyData: wrong size for the data for kAudioDevicePropertyNominalSampleRate");
// 			bool const new_state = (*((UInt32 const*)inData) != 0);

// 			atomic_store_explicit(&IS_OUTPUT_STREAM_ACTIVE, new_state, __ATOMIC_RELAXED);

// 			HOST->PropertiesChanged(HOST, inObjectID, 1, inAddress);
// 			DebugMsg("HostChange");
// 			return kAudioHardwareNoError;
// 		}

// 		return kAudioHardwareUnknownPropertyError;
// 	}

// 	if (audio_object_is_control(inObjectID)) {
// 		UInt32 n_changed_addrs = 0;
// 		// we might change two properties when changing a volume control because
// 		// it changes both the scalar value property and the decibel value property
// 		AudioObjectPropertyAddress changed_addrs[2];

// 		OSStatus const status = syfala_SetControlPropertyData(
// 			inObjectID,
// 			inAddress,
// 			inDataSize,
// 			inData,
// 			&n_changed_addrs,
// 			changed_addrs
// 		);

// 		if (status != 0) return status;

// 		if (n_changed_addrs > 0) {
// 			HOST->PropertiesChanged(HOST, inObjectID, n_changed_addrs, changed_addrs);
// 			DebugMsg("HostChange");
// 		}

// 		return kAudioHardwareNoError;
// 	}

// 	return kAudioHardwareBadObjectError;
// }



fn syfala_has_plug_in_property(selector: AudioObjectPropertySelector) -> bool {
    //	This method returns whether or not the plug-in object has the given property.

    match selector {
        kAudioObjectPropertyBaseClass
        | kAudioObjectPropertyClass
        | kAudioObjectPropertyOwner
        | kAudioObjectPropertyManufacturer
        | kAudioObjectPropertyOwnedObjects
        | kAudioPlugInPropertyDeviceList
        | kAudioPlugInPropertyTranslateUIDToDevice
        | kAudioPlugInPropertyResourceBundle => true,
        _ => false,
    }
}

fn syfala_is_plug_in_property_settable(
    selector: AudioObjectPropertySelector,
) -> Result<bool, OSStatus> {
    //	This method returns whether or not the given property on the plug-in object can have its
    //	value changed.
    match selector {
        kAudioObjectPropertyBaseClass
        | kAudioObjectPropertyClass
        | kAudioObjectPropertyOwner
        | kAudioObjectPropertyManufacturer
        | kAudioObjectPropertyOwnedObjects
        | kAudioPlugInPropertyDeviceList
        | kAudioPlugInPropertyTranslateUIDToDevice
        | kAudioPlugInPropertyResourceBundle => Ok(true),
        _ => Err(kAudioHardwareUnknownPropertyError as OSStatus),
    }
}

fn syfala_get_plug_in_property_data_size(selector: AudioObjectPropertySelector) -> Option<UInt32> {
    //	This method returns the byte size of the property's data.

    //	Note that for each object, this driver implements all the required properties plus a few
    //	extras that are useful but not required. There is more detailed commentary about each
    //	property in the syfala_GetPlugInPropertyData() method.
    match selector {
        kAudioObjectPropertyBaseClass | kAudioObjectPropertyClass => {
            Some(size_of::<AudioClassID>().try_into().unwrap())
        }
        kAudioObjectPropertyOwner => Some(size_of::<AudioObjectID>().try_into().unwrap()),
        kAudioObjectPropertyManufacturer => Some(size_of::<CFStringRef>().try_into().unwrap()),
        kAudioObjectPropertyOwnedObjects | kAudioPlugInPropertyDeviceList => {
            Some(size_of::<AudioObjectID>().strict_mul(1).try_into().unwrap())
        }
        kAudioPlugInPropertyTranslateUIDToDevice => {
            Some(size_of::<AudioObjectID>().try_into().unwrap())
        }
        kAudioPlugInPropertyResourceBundle => Some(size_of::<CFStringRef>().try_into().unwrap()),
        _ => None,
    }
}



// Not the cleanest, i know,
/// None: kAudioHardwareUnknownPropertyError
/// Some(None): kAudioHardwareBadPropertySizeError,
/// Some(Some(n)): kAudioHardwareNoError,
unsafe fn syfala_get_plug_in_property_data(
    selector: AudioObjectPropertySelector,
    qualifier_data_size: UInt32,
    qualifier_data: *const std::os::raw::c_void,
    data_size: UInt32,
    out_data: *mut std::os::raw::c_void,
) -> Option<Option<UInt32>> {
    //	Note that for each object, this driver implements all the required properties plus a few
    //	extras that are useful but not required.
    //
    //	Also, since most of the data that will get returned is static, there are few instances where
    //	it is necessary to lock the state mutex.

    let Some(size) = syfala_get_plug_in_property_data_size(selector) else {
        return None;
    };

    if data_size < size {
        return Some(None);
    }

    match selector {
        //	The base class for kAudioPlugInClassID is kAudioObjectClassID
        kAudioObjectPropertyBaseClass => unsafe {
            ptr::write(out_data.cast(), kAudioObjectClassID)
        },
        //	The class is always kAudioPlugInClassID for regular drivers
        kAudioObjectPropertyClass => unsafe { ptr::write(out_data.cast(), kAudioPlugInClassID) },
        //	The plug-in doesn't have an owning object
        kAudioObjectPropertyOwner => unsafe { ptr::write(out_data.cast(), kAudioObjectUnknown) },
        //	This is the human readable name of the maker of the plug-in.
        kAudioObjectPropertyManufacturer => unsafe {
            ptr::write(out_data.cast(), cfstr("GRAME CNCM"))
        },

        // The plugin owns only 1 object, which is the device. So, handle these two cases the same way
        kAudioObjectPropertyOwnedObjects
        | kAudioPlugInPropertyDeviceList => {
        	//	Calculate the number of items that have been requested. Note that this
        	//	number is allowed to be smaller than the actual size of the list. In such
        	//	case, only that number of items will be returned
        	UInt32 const n_items = inDataSize / sizeof(AudioObjectID);
        	//	Clamp that to the number of devices this driver implements (which is just 1)
        	UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

        	AudioObjectID* const data = (AudioObjectID*) out_data;

        	UInt32 index = 0;

        	if(index < n_items_clamped) data[index++] = DEVICE_ID;

        	//	Return how many bytes we wrote to
        	*outDataSize = n_items_clamped * sizeof(AudioObjectID);
        	break;
        },

        // kAudioPlugInPropertyTranslateUIDToDevice => {
        // 	//	This property takes the CFString passed in the qualifier and converts that
        // 	//	to the object ID of the device it corresponds to. For this driver, there is
        // 	//	just the one device. Note that it is not an error if the string in the
        // 	//	qualifier doesn't match any devices. In such case, kAudioObjectUnknown is
        // 	//	the object ID to return.
        // 	DoIfFailed(data_size < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: not enough space for the return value of kAudioPlugInPropertyTranslateUIDToDevice");
        // 	DoIfFailed(qualifier_data_size != sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: the qualifier is the wrong size for kAudioPlugInPropertyTranslateUIDToDevice");
        // 	DoIfFailed(qualifier_data == NULL, return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: no qualifier for kAudioPlugInPropertyTranslateUIDToDevice");

        // 	if(
        // 		CFStringCompare(
        // 			*((CFStringRef*)qualifier_data),
        // 			CFSTR(kDevice_UID),
        // 			0
        // 		) == kCFCompareEqualTo
        // 	) *((AudioObjectID*)out_data) = DEVICE_ID;

        // 	else *((AudioObjectID*)out_data) = kAudioObjectUnknown;

        // 	*outDataSize = sizeof(AudioObjectID);
        // 	break;
        // },

        // kAudioPlugInPropertyResourceBundle => {
        // 	//	The resource bundle is a path relative to the path of the plug-in's bundle.
        // 	//	To specify that the plug-in bundle itself should be used, we just return the
        // 	//	empty string.
        // 	DoIfFailed(data_size < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetPlugInPropertyData: not enough space for the return value of kAudioPlugInPropertyResourceBundle");
        // 	*((CFStringRef*)out_data) = CFSTR("");
        // 	*outDataSize = sizeof(CFStringRef);
        // 	break;
        // },
        _ => unreachable!(),
    }

    Some(Some(size))
}

// #pragma mark Device Property Operations

// static Boolean	syfala_HasDeviceProperty(
// 	AudioObjectPropertyAddress const* const inAddress
// ) {
// 	//	This method returns whether or not the given object has the given property.
// 	switch(inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyElementName:
// 			return inAddress->mElement <= N_CHANNELS;

// 		case kAudioObjectPropertyBaseClass:
// 		case kAudioObjectPropertyClass:
// 		case kAudioObjectPropertyOwner:
// 		case kAudioObjectPropertyName:
// 		case kAudioObjectPropertyManufacturer:
// 		case kAudioObjectPropertyOwnedObjects:
// 		case kAudioObjectPropertyControlList:

// 		case kAudioDevicePropertyDeviceUID:
// 		case kAudioDevicePropertyModelUID:
// 		case kAudioDevicePropertyTransportType:
// 		case kAudioDevicePropertyRelatedDevices:
// 		case kAudioDevicePropertyClockDomain:
// 		case kAudioDevicePropertyDeviceIsAlive:
// 		case kAudioDevicePropertyDeviceIsRunning:
// 		case kAudioDevicePropertyNominalSampleRate:
// 		case kAudioDevicePropertyAvailableNominalSampleRates:
// 		case kAudioDevicePropertyIsHidden:
// 		case kAudioDevicePropertyZeroTimeStampPeriod:
// 		case kAudioDevicePropertyIcon:
// 		case kAudioDevicePropertyStreams:
// 			return true;

// 		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
// 		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
// 		case kAudioDevicePropertyLatency:
// 		case kAudioDevicePropertySafetyOffset:
// 		case kAudioDevicePropertyPreferredChannelLayout:
// 			return (inAddress->mScope == kAudioObjectPropertyScopeOutput) || (inAddress->mScope == kAudioObjectPropertyScopeGlobal);
// 	};

// 	return false;
// }

// static OSStatus	syfala_IsDevicePropertySettable(
// 	AudioObjectPropertyAddress const* const inAddress,
// 	Boolean* const outIsSettable
// ) {
// 	//	This method returns whether or not the given property on the object can have its value
// 	//	changed.
// 	switch(inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyElementName:
// 		case kAudioObjectPropertyBaseClass:
// 		case kAudioObjectPropertyClass:
// 		case kAudioObjectPropertyOwner:
// 		case kAudioObjectPropertyName:
// 		case kAudioObjectPropertyManufacturer:
// 		case kAudioObjectPropertyOwnedObjects:
// 		case kAudioObjectPropertyControlList:

// 		case kAudioDevicePropertyDeviceUID:
// 		case kAudioDevicePropertyModelUID:
// 		case kAudioDevicePropertyTransportType:
// 		case kAudioDevicePropertyRelatedDevices:
// 		case kAudioDevicePropertyClockDomain:
// 		case kAudioDevicePropertyDeviceIsAlive:
// 		case kAudioDevicePropertyDeviceIsRunning:
// 		case kAudioDevicePropertyNominalSampleRate:
// 		case kAudioDevicePropertyAvailableNominalSampleRates:
// 		case kAudioDevicePropertyIsHidden:
// 		case kAudioDevicePropertyZeroTimeStampPeriod:
// 		case kAudioDevicePropertyStreams:
// 		case kAudioDevicePropertyIcon:
// 		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
// 		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
// 		case kAudioDevicePropertyLatency:
// 		case kAudioDevicePropertySafetyOffset:
// 		case kAudioDevicePropertyPreferredChannelLayout:
// 			*outIsSettable = false;
// 			break;

// 		default:
// 			return kAudioHardwareUnknownPropertyError;
// 	};

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_GetDevicePropertyDataSize(
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32* const outDataSize
// ) {
// 	//	This method returns the byte size of the property's data.

// 	//	Note that for each object, this driver implements all the required properties plus a few
// 	//	extras that are useful but not required. There is more detailed commentary about each
// 	//	property in the syfala_GetDevicePropertyData() method.
// 	switch(inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyElementName:
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioObjectPropertyBaseClass:
// 		case kAudioObjectPropertyClass:
// 			*outDataSize = sizeof(AudioClassID);
// 			break;

// 		case kAudioObjectPropertyOwner:
// 			*outDataSize = sizeof(AudioObjectID);
// 			break;

// 		case kAudioObjectPropertyName:
// 		case kAudioObjectPropertyManufacturer:
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioObjectPropertyOwnedObjects:
// 			switch(inAddress->mScope)
// 			{
// 				case kAudioObjectPropertyScopeGlobal:
// 				case kAudioObjectPropertyScopeOutput:
// 					*outDataSize = N_DEVICE_OWNED_OBJS * sizeof(AudioObjectID);
// 					break;
// 				default:
// 					return kAudioHardwareUnknownPropertyError;
// 			};
// 			break;

// 		case kAudioObjectPropertyControlList:

// 			*outDataSize = N_CONTROLS * sizeof(AudioObjectID);
// 			break;

// 		case kAudioDevicePropertyDeviceUID:
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioDevicePropertyModelUID:
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioDevicePropertyTransportType:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyRelatedDevices:
// 			*outDataSize = 1 * sizeof(AudioObjectID);
// 			break;

// 		case kAudioDevicePropertyClockDomain:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyDeviceIsAlive:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyDeviceIsRunning:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyNominalSampleRate:
// 			*outDataSize = sizeof(Float64);
// 			break;

// 		case kAudioDevicePropertyAvailableNominalSampleRates:
// 			*outDataSize = 1 * sizeof(AudioValueRange);
// 			break;

// 		case kAudioDevicePropertyIsHidden:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyZeroTimeStampPeriod:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyIcon:
// 			*outDataSize = sizeof(CFURLRef);
// 			break;

// 		case kAudioDevicePropertyStreams:
// 			switch(inAddress->mScope)
// 			{
// 				case kAudioObjectPropertyScopeGlobal:
// 				case kAudioObjectPropertyScopeOutput:
// 					*outDataSize = 1 * sizeof(AudioObjectID);
// 					break;
// 				case kAudioObjectPropertyScopeInput:
// 					*outDataSize = 0 * sizeof(AudioObjectID);
// 					break;

// 				default:
// 					return kAudioHardwareUnknownPropertyError;
// 			};
// 			break;

// 		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyLatency:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertySafetyOffset:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyPreferredChannelLayout:
// 			*outDataSize = offsetof(AudioChannelLayout, mChannelDescriptions) + (1 * sizeof(AudioChannelDescription));
// 			break;

// 		default:
// 			return kAudioHardwareUnknownPropertyError;
// 	};

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_GetDevicePropertyData(
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32 const inDataSize,
// 	UInt32* const outDataSize,
// 	void* const outData
// ) {
// 	//	Note that for each object, this driver implements all the required properties plus a few
// 	//	extras that are useful but not required.
// 	//
// 	//	Also, since most of the data that will get returned is static, there are few instances where
// 	//	it is necessary to lock the state mutex.
// 	switch(inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyElementName: {
// 			//	This is the human readable name of the maker of the plug-in.
// 			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyElementName for the device");

// 			CFStringRef* const data = (CFStringRef*) outData;
// 			AudioObjectPropertyElement const elem = inAddress->mElement;

// 			if (elem == kAudioObjectPropertyElementMain /* AKA 0 */) {
// 				*data = CFSTR("Master");
// 			} else if (elem <= N_CHANNELS) {
// 				*data = CFStringCreateWithFormat(NULL, NULL, CFSTR("%u"), elem);
// 			} else {
// 				*data = CFSTR("<Unknown>");
// 			}

// 			*outDataSize = sizeof(CFStringRef);
// 			break;
// 		}

// 		case kAudioObjectPropertyBaseClass:
// 			//	The base class for kAudioDeviceClassID is kAudioObjectClassID
// 			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the device");
// 			*((AudioClassID*)outData) = kAudioObjectClassID;
// 			*outDataSize = sizeof(AudioClassID);
// 			break;

// 		case kAudioObjectPropertyClass:
// 			//	The class is always kAudioDeviceClassID for devices created by drivers
// 			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyClass for the device");
// 			*((AudioClassID*)outData) = kAudioDeviceClassID;
// 			*outDataSize = sizeof(AudioClassID);
// 			break;

// 		case kAudioObjectPropertyOwner:
// 			//	The device's owner is the plug-in object
// 			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyOwner for the device");
// 			*((AudioObjectID*)outData) = PLUGIN_ID;
// 			*outDataSize = sizeof(AudioObjectID);
// 			break;

// 		case kAudioObjectPropertyName:
// 			//	This is the human readable name of the device.
// 			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyName for the device");
// 			*((CFStringRef*)outData) = CFSTR("Space Bar");
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioObjectPropertyManufacturer:
// 			//	This is the human readable name of the maker of the device.
// 			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioObjectPropertyManufacturer for the device");
// 			*((CFStringRef*)outData) = CFSTR("GRAME");
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		// These two cases return the same list but with the output
// 		// stream id included if all owned objects are requested

// 		//	The device owns its streams and controls.
// 		case kAudioObjectPropertyOwnedObjects:
// 		case kAudioObjectPropertyControlList:

// 			switch(inAddress->mScope)
// 			{
// 				case kAudioObjectPropertyScopeGlobal:
// 				case kAudioObjectPropertyScopeOutput:
// 				{
// 					//	Calculate the number of items that have been requested. Note that this
// 					//	number is allowed to be smaller than the actual size of the list. In such
// 					//	case, only that number of items will be returned
// 					UInt32 const n_items = inDataSize / sizeof(AudioObjectID);
// 					bool const include_stream_obj = inAddress->mSelector == kAudioObjectPropertyOwnedObjects;
// 					UInt32 const max_n_items = N_CONTROLS + ((include_stream_obj) ? 1 : 0);
// 					UInt32 const n_items_clamped = (n_items > max_n_items) ? max_n_items : n_items;

// 					AudioObjectID* const data = (AudioObjectID*) outData;

// 					UInt32 index = 0;

// 					// output stream, return that only if we actually requested it
// 					if (index < n_items_clamped && include_stream_obj)
// 						data[index++] = STREAM_ID;

// 					for (AudioObjectID id = FIRST_CTRL_ID; id <= LAST_CTRL_ID; ++id)
// 						if (index < n_items_clamped) data[index++] = id;

// 					//	report how much we wrote
// 					*outDataSize = n_items_clamped * sizeof(AudioObjectID);
// 					break;
// 				}
// 				case kAudioObjectPropertyScopeInput:
// 					*outDataSize = 0 * sizeof(AudioObjectID);
// 					break;

// 				default:
// 					return kAudioHardwareUnknownPropertyError;
// 			};

// 			break;

// 		case kAudioDevicePropertyDeviceUID:
// 			//	This is a CFString that is a persistent token that can identify the same
// 			//	audio device across boot sessions. Note that two instances of the same
// 			//	device must have different values for this property.
// 			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceUID for the device");
// 			*((CFStringRef*)outData) = CFSTR(kDevice_UID);
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioDevicePropertyModelUID:
// 			//	This is a CFString that is a persistent token that can identify audio
// 			//	devices that are the same kind of device. Note that two instances of the
// 			//	save device must have the same value for this property.
// 			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyModelUID for the device");
// 			*((CFStringRef*)outData) = CFSTR(kDevice_ModelUID);
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioDevicePropertyTransportType:
// 			//	This value represents how the device is attached to the system. This can be
// 			//	any 32 bit integer, but common values for this property are defined in
// 			//	<CoreAudio/AudioHardwareBase.h>
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyTransportType for the device");
// 			*((UInt32*)outData) = kAudioDeviceTransportTypeAVB;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyRelatedDevices:
// 		{
// 			//	The related devices property identifys device objects that are very closely
// 			//	related. Generally, this is for relating devices that are packaged together
// 			//	in the hardware such as when the input side and the output side of a piece
// 			//	of hardware can be clocked separately and therefore need to be represented
// 			//	as separate AudioDevice objects. In such case, both devices would report
// 			//	that they are related to each other. Note that at minimum, a device is
// 			//	related to itself, so this list will always be at least one item long.

// 			//	Calculate the number of items that have been requested. Note that this
// 			//	number is allowed to be smaller than the actual size of the list. In such
// 			//	case, only that number of items will be returned
// 			UInt32 const n_items = inDataSize / sizeof(AudioObjectID);
// 			UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

// 			AudioObjectID* const data = (AudioObjectID*) outData;

// 			UInt32 index = 0;

// 			//	Write the devices' object IDs into the return value
// 			if(index < n_items_clamped) data[index++] = DEVICE_ID;

// 			//	report how much we wrote
// 			*outDataSize = n_items_clamped * sizeof(AudioObjectID);
// 			break;
// 		}

// 		case kAudioDevicePropertyClockDomain:
// 			//	This property allows the device to declare what other devices it is
// 			//	synchronized with in hardware. The way it works is that if two devices have
// 			//	the same value for this property and the value is not zero, then the two
// 			//	devices are synchronized in hardware. Note that a device that either can't
// 			//	be synchronized with others or doesn't know should return 0 for this
// 			//	property.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyClockDomain for the device");
// 			*((UInt32*)outData) = 0;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyDeviceIsAlive:
// 			//	This property returns whether or not the device is alive. Note that it is
// 			//	note uncommon for a device to be dead but still momentarily availble in the
// 			//	device list. In the case of this device, it will always be alive.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceIsAlive for the device");
// 			*((UInt32*)outData) = 1;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyDeviceIsRunning:
// 			//	This property returns whether or not IO is running for the device
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceIsRunning for the device");
// 			pthread_mutex_lock(&gDevice_IOMutex);
// 			*((UInt32*)outData) = (gDevice_IOIsRunning > 0) ? 1 : 0;
// 			pthread_mutex_unlock(&gDevice_IOMutex);
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyNominalSampleRate:
// 			//	This property returns the nominal sample rate of the device.
// 			DoIfFailed(inDataSize < sizeof(Float64), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyNominalSampleRate for the device");
// 			*((Float64*)outData) = SAMPLE_RATE;
// 			*outDataSize = sizeof(Float64);
// 			break;

// 		case kAudioDevicePropertyAvailableNominalSampleRates:
// 		{
// 			//	This returns all nominal sample rates the device supports as an array of
// 			//	AudioValueRangeStructs. Note that for discrete sampler rates, the range
// 			//	will have the minimum value equal to the maximum value.

// 			//	Calculate the number of items that have been requested. Note that this
// 			//	number is allowed to be smaller than the actual size of the list. In such
// 			//	case, only that number of items will be returned
// 			UInt32 const n_items = inDataSize / sizeof(AudioValueRange);
// 			UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

// 			AudioValueRange* const data = (AudioValueRange*) outData;

// 			UInt32 index = 0;

// 			if (index < n_items_clamped) {
// 				AudioValueRange* const range = &data[index++];
// 				range->mMinimum = SAMPLE_RATE;
// 				range->mMaximum = SAMPLE_RATE;
// 			}

// 			//	report how much we wrote
// 			*outDataSize = n_items_clamped * sizeof(AudioValueRange);
// 			break;
// 		}

// 		case kAudioDevicePropertyIsHidden:
// 			//	This returns whether or not the device is visible to clients.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyIsHidden for the device");
// 			*((UInt32*)outData) = 0;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyZeroTimeStampPeriod:
// 			//	This property returns how many frames the HAL should expect to see between
// 			//	successive sample times in the zero time stamps this device provides.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyZeroTimeStampPeriod for the device");
// 			*((UInt32*)outData) = kDevice_RingBufferSize;
// 			*outDataSize = sizeof(UInt32); // TODO?
// 			break;

// 		case kAudioDevicePropertyIcon:
// 			//	This is a CFURL that points to the device's Icon in the plug-in's resource bundle.
// 			DoIfFailed(inDataSize < sizeof(CFURLRef), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceUID for the device");
// 			CFBundleRef theBundle = CFBundleGetBundleWithIdentifier(CFSTR(kPlugIn_BundleID));
// 			DoIfFailed(theBundle == NULL, return kAudioHardwareUnspecifiedError, "syfala_GetDevicePropertyData: could not get the plug-in bundle for kAudioDevicePropertyIcon");
// 			CFURLRef theURL = CFBundleCopyResourceURL(theBundle, CFSTR("DeviceIcon.icns"), NULL, NULL);
// 			DoIfFailed(theURL == NULL, return kAudioHardwareUnspecifiedError, "syfala_GetDevicePropertyData: could not get the URL for kAudioDevicePropertyIcon");
// 			*((CFURLRef*)outData) = theURL;
// 			*outDataSize = sizeof(CFURLRef);
// 			break;

// 		case kAudioDevicePropertyStreams:
// 			switch(inAddress->mScope)
// 			{
// 				case kAudioObjectPropertyScopeGlobal:
// 				case kAudioObjectPropertyScopeOutput:
// 				{
// 					//	Calculate the number of items that have been requested. Note that this
// 					//	number is allowed to be smaller than the actual size of the list. In such
// 					//	case, only that number of items will be returned
// 					UInt32 const n_items = inDataSize / sizeof(AudioObjectID);
// 					UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

// 					AudioObjectID* const data = (AudioObjectID*) outData;

// 					UInt32 index = 0;

// 					if (index < n_items_clamped)
// 						data[index++] = STREAM_ID;

// 					//	report how much we wrote
// 					*outDataSize = n_items_clamped * sizeof(AudioObjectID);
// 					break;
// 				}

// 				default:
// 					return kAudioHardwareUnknownPropertyError;
// 			};

// 			break;

// 		case kAudioDevicePropertyDeviceCanBeDefaultDevice:
// 			//	This property returns whether or not the device wants to be able to be the
// 			//	default device for content. This is the device that iTunes and QuickTime
// 			//	will use to play their content on and FaceTime will use as it's microhphone.
// 			//	Nearly all devices should allow for this.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceCanBeDefaultDevice for the device");
// 			*((UInt32*)outData) = 1;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
// 			//	This property returns whether or not the device wants to be the system
// 			//	default device. This is the device that is used to play interface sounds and
// 			//	other incidental or UI-related sounds on. Most devices should allow this
// 			//	although devices with lots of latency may not want to.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyDeviceCanBeDefaultSystemDevice for the device");
// 			*((UInt32*)outData) = 1;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyLatency:
// 			//	This property returns the presentation latency of the device. For this,
// 			//	device, the value is 0 due to the fact that it always vends silence.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyLatency for the device");
// 			*((UInt32*)outData) = 0;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertySafetyOffset:
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertySafetyOffset for the device");
// 			*((UInt32*)outData) = 0;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioDevicePropertyPreferredChannelLayout:
// 		{
// 			UInt32 const size = offsetof(AudioChannelLayout, mChannelDescriptions) + (1 * sizeof(AudioChannelDescription));

// 			AudioChannelLayout* const data = (AudioChannelLayout*) outData;

// 			DoIfFailed(inDataSize < size, return kAudioHardwareBadPropertySizeError, "syfala_GetDevicePropertyData: not enough space for the return value of kAudioDevicePropertyPreferredChannelLayout for the device");

// 			data->mChannelLayoutTag = kAudioChannelLayoutTag_UseChannelDescriptions;
// 			data->mChannelBitmap = 0;
// 			data->mNumberChannelDescriptions = 1;

// 			AudioChannelDescription* const desc = &data->mChannelDescriptions[0];

// 			desc->mChannelLabel = 1;
// 			desc->mChannelFlags = 0;
// 			desc->mCoordinates[0] = 0;
// 			desc->mCoordinates[1] = 0;
// 			desc->mCoordinates[2] = 0;
// 			*outDataSize = size;
// 			break;
// 		}

// 		default:
// 			return kAudioHardwareUnknownPropertyError;
// 	};

// 	return kAudioHardwareNoError;
// }

// #pragma mark Stream Property Operations

// static Boolean	syfala_HasStreamProperty(
// 	AudioObjectPropertyAddress const* inAddress
// ) {
// 	//	This method returns whether or not the given object has the given property.
// 	switch(inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyBaseClass:
// 		case kAudioObjectPropertyClass:
// 		case kAudioObjectPropertyOwner:
// 		case kAudioObjectPropertyOwnedObjects:
// 		case kAudioObjectPropertyName:

// 		case kAudioStreamPropertyDirection:
// 		case kAudioStreamPropertyTerminalType:
// 		case kAudioStreamPropertyStartingChannel:
// 		case kAudioStreamPropertyLatency:
// 		case kAudioStreamPropertyAvailableVirtualFormats:
// 		case kAudioStreamPropertyAvailablePhysicalFormats:
// 		case kAudioStreamPropertyVirtualFormat:
// 		case kAudioStreamPropertyPhysicalFormat:
// 		case kAudioStreamPropertyIsActive:
// 			return true;
// 	};

// 	return false;
// }

// static OSStatus	syfala_IsStreamPropertySettable(
// 	AudioObjectPropertyAddress const* const inAddress,
// 	Boolean* const outIsSettable
// ) {
// 	//	This method returns whether or not the given property on the object can have its value
// 	//	changed.
// 	switch(inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyBaseClass:
// 		case kAudioObjectPropertyClass:
// 		case kAudioObjectPropertyOwner:
// 		case kAudioObjectPropertyOwnedObjects:
// 		case kAudioObjectPropertyName:

// 		case kAudioStreamPropertyDirection:
// 		case kAudioStreamPropertyTerminalType:
// 		case kAudioStreamPropertyStartingChannel:
// 		case kAudioStreamPropertyLatency:
// 		case kAudioStreamPropertyAvailableVirtualFormats:
// 		case kAudioStreamPropertyAvailablePhysicalFormats:
// 		case kAudioStreamPropertyVirtualFormat:
// 		case kAudioStreamPropertyPhysicalFormat:
// 			*outIsSettable = false;
// 			break;

// 		case kAudioStreamPropertyIsActive:
// 			*outIsSettable = true;
// 			break;

// 		default:
// 			return kAudioHardwareUnknownPropertyError;
// 	};

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_GetStreamPropertyDataSize(
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32* const outDataSize
// ) {
// 	//	This method returns the byte size of the property's data.
// 	switch(inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyBaseClass:
// 			*outDataSize = sizeof(AudioClassID);
// 			break;

// 		case kAudioObjectPropertyClass:
// 			*outDataSize = sizeof(AudioClassID);
// 			break;

// 		case kAudioObjectPropertyOwner:
// 			*outDataSize = sizeof(AudioObjectID);
// 			break;

// 		case kAudioObjectPropertyOwnedObjects:
// 			*outDataSize = 0 * sizeof(AudioObjectID);
// 			break;

// 		case kAudioObjectPropertyName:
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioStreamPropertyDirection:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioStreamPropertyTerminalType:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioStreamPropertyStartingChannel:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioStreamPropertyLatency:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioStreamPropertyAvailableVirtualFormats:
// 		case kAudioStreamPropertyAvailablePhysicalFormats:
// 			*outDataSize = 1 * sizeof(AudioStreamRangedDescription);
// 			break;

// 		case kAudioStreamPropertyVirtualFormat:
// 		case kAudioStreamPropertyPhysicalFormat:
// 			*outDataSize = sizeof(AudioStreamBasicDescription);
// 			break;

// 		case kAudioStreamPropertyIsActive:
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		default:
// 			return kAudioHardwareUnknownPropertyError;
// 	};

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_GetStreamPropertyData(
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32 const inDataSize,
// 	UInt32* const outDataSize,
// 	void* const outData
// ) {
// 	switch(inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyBaseClass:
// 			//	The base class for kAudioStreamClassID is kAudioObjectClassID
// 			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the stream");
// 			*((AudioClassID*)outData) = kAudioObjectClassID;
// 			*outDataSize = sizeof(AudioClassID);
// 			break;

// 		case kAudioObjectPropertyClass:
// 			//	The class is always kAudioStreamClassID for streams created by drivers
// 			DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioObjectPropertyClass for the stream");
// 			*((AudioClassID*)outData) = kAudioStreamClassID;
// 			*outDataSize = sizeof(AudioClassID);
// 			break;

// 		case kAudioObjectPropertyOwner:
// 			//	The stream's owner is the device object
// 			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioObjectPropertyOwner for the stream");
// 			*((AudioObjectID*)outData) = DEVICE_ID;
// 			*outDataSize = sizeof(AudioObjectID);
// 			break;

// 		case kAudioObjectPropertyOwnedObjects:
// 			//	Streams do not own any objects
// 			*outDataSize = 0 * sizeof(AudioObjectID);
// 			break;

// 		case kAudioObjectPropertyName:
// 			//	This is the human readable name of the stream
// 			DoIfFailed(inDataSize < sizeof(CFStringRef), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioObjectPropertyName for the stream");
// 			*((CFStringRef*)outData) = CFSTR("Output Stream");
// 			*outDataSize = sizeof(CFStringRef);
// 			break;

// 		case kAudioStreamPropertyDirection:
// 			//	This returns whether the stream is an input stream or an output stream.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyDirection for the stream");
// 			*((UInt32*)outData) = 0;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioStreamPropertyTerminalType:
// 			//	This returns a value that indicates what is at the other end of the stream
// 			//	such as a speaker or headphones, or a microphone. Values for this property
// 			//	are defined in <CoreAudio/AudioHardwareBase.h>
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyTerminalType for the stream");
// 			*((UInt32*)outData) = kAudioStreamTerminalTypeUnknown;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioStreamPropertyStartingChannel:
// 			//	This property returns the absolute channel number for the first channel in
// 			//	the stream. For exmaple, if a device has two output streams with two
// 			//	channels each, then the starting channel number for the first stream is 1
// 			//	and ths starting channel number fo the second stream is 3.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyStartingChannel for the stream");
// 			*((UInt32*)outData) = 1;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioStreamPropertyLatency:
// 			//	This property returns any additonal presentation latency the stream has.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyStartingChannel for the stream");
// 			*((UInt32*)outData) = 0;
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		case kAudioStreamPropertyAvailableVirtualFormats:
// 		case kAudioStreamPropertyAvailablePhysicalFormats:
// 		{
// 			//	This returns an array of AudioStreamRangedDescriptions that describe what
// 			//	formats are supported.

// 			//	Calculate the number of items that have been requested. Note that this
// 			//	number is allowed to be smaller than the actual size of the list. In such
// 			//	case, only that number of items will be returned
// 			UInt32 const n_items = inDataSize / sizeof(AudioStreamRangedDescription);
// 			UInt32 const n_items_clamped = (n_items > 1) ? 1 : n_items;

// 			AudioStreamRangedDescription* const data = (AudioStreamRangedDescription*) outData;

// 			UInt32 index = 0;

// 			if (index < n_items_clamped) data[index++] = FORMAT;

// 			//	report how much we wrote
// 			*outDataSize = n_items_clamped * sizeof(AudioStreamRangedDescription);
// 			break;
// 		}

// 		case kAudioStreamPropertyVirtualFormat:
// 		case kAudioStreamPropertyPhysicalFormat:
// 			//	This returns the current format of the stream in an
// 			//	AudioStreamBasicDescription. Note that we need to hold the state lock to get
// 			//	this value.
// 			//	Note that for devices that don't override the mix operation, the virtual
// 			//	format has to be the same as the physical format.
// 			DoIfFailed(inDataSize < sizeof(AudioStreamBasicDescription), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyVirtualFormat for the stream");
// 			*((AudioStreamBasicDescription*)outData) = FORMAT.mFormat;
// 			*outDataSize = sizeof(AudioStreamBasicDescription);
// 			break;

// 		case kAudioStreamPropertyIsActive:
// 			//	This property tells the device whether or not the given stream is going to
// 			//	be used for IO. Note that we need to take the state lock to examine this
// 			//	value.
// 			DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetStreamPropertyData: not enough space for the return value of kAudioStreamPropertyIsActive for the stream");
// 			*((UInt32*)outData) = atomic_load_explicit(&IS_OUTPUT_STREAM_ACTIVE, __ATOMIC_RELAXED);
// 			*outDataSize = sizeof(UInt32);
// 			break;

// 		default:
// 			return kAudioHardwareUnknownPropertyError;
// 	};

// 	return kAudioHardwareNoError;
// }

// #pragma mark Control Property Operations

// static Boolean	syfala_HasControlProperty(
// 	AudioObjectID const inObjectID,
// 	AudioObjectPropertyAddress const* const inAddress
// ) {
// 	switch (inAddress->mSelector) {
// 		case kAudioObjectPropertyBaseClass:
// 		case kAudioObjectPropertyClass:
// 		case kAudioObjectPropertyOwner:
// 		case kAudioObjectPropertyOwnedObjects:
// 		case kAudioControlPropertyScope:
// 		case kAudioControlPropertyElement:
// 			return true;
// 	}

// 	if (control_is_volume(inObjectID)) {
// 		switch (inAddress->mSelector) {
// 			case kAudioLevelControlPropertyScalarValue:
// 			case kAudioLevelControlPropertyDecibelValue:
// 			case kAudioLevelControlPropertyDecibelRange:
// 			case kAudioLevelControlPropertyConvertScalarToDecibels:
// 			case kAudioLevelControlPropertyConvertDecibelsToScalar:
// 				return true;
// 		}
// 	} else {
// 		if (inAddress->mSelector == kAudioBooleanControlPropertyValue) {
// 			return true;
// 		}
// 	}

// 	return false;
// }

// static OSStatus	syfala_IsControlPropertySettable(
// 	AudioObjectID const inObjectID,
// 	AudioObjectPropertyAddress const* const inAddress,
// 	Boolean* const outIsSettable
// ) {
// 	switch (inAddress->mSelector) {
// 		case kAudioObjectPropertyBaseClass:
// 		case kAudioObjectPropertyClass:
// 		case kAudioObjectPropertyOwner:
// 		case kAudioObjectPropertyOwnedObjects:
// 		case kAudioControlPropertyScope:
// 		case kAudioControlPropertyElement:
// 			*outIsSettable = false;
// 			return kAudioHardwareNoError;
// 	}

// 	if (control_is_volume(inObjectID)) {
// 		switch (inAddress->mSelector) {
// 			case kAudioLevelControlPropertyDecibelRange:
// 			case kAudioLevelControlPropertyConvertScalarToDecibels:
// 			case kAudioLevelControlPropertyConvertDecibelsToScalar:
// 				*outIsSettable = false;
// 				return kAudioHardwareNoError;

// 			case kAudioLevelControlPropertyScalarValue:
// 			case kAudioLevelControlPropertyDecibelValue:
// 				*outIsSettable = true;
// 				return kAudioHardwareNoError;
// 		}
// 	} else {
// 		if (inAddress->mSelector == kAudioBooleanControlPropertyValue) {
// 			*outIsSettable = true;
// 			return kAudioHardwareNoError;
// 		}
// 	}

// 	return kAudioHardwareUnknownPropertyError;
// }

// static OSStatus	syfala_GetControlPropertyDataSize(
// 	AudioObjectID const inObjectID,
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32* const outDataSize
// ) {
// 	switch (inAddress->mSelector) {
// 		case kAudioObjectPropertyBaseClass:
// 			*outDataSize = sizeof(AudioClassID);
// 			return kAudioHardwareNoError;

// 		case kAudioObjectPropertyClass:
// 			*outDataSize = sizeof(AudioClassID);
// 			return kAudioHardwareNoError;

// 		case kAudioObjectPropertyOwner:
// 			*outDataSize = sizeof(AudioObjectID);
// 			return kAudioHardwareNoError;

// 		case kAudioObjectPropertyOwnedObjects:
// 			*outDataSize = 0 * sizeof(AudioObjectID);
// 			return kAudioHardwareNoError;

// 		case kAudioControlPropertyScope:
// 			*outDataSize = sizeof(AudioObjectPropertyScope);
// 			return kAudioHardwareNoError;

// 		case kAudioControlPropertyElement:
// 			*outDataSize = sizeof(AudioObjectPropertyElement);
// 			return kAudioHardwareNoError;
// 	}

// 	if (control_is_volume(inObjectID)) {
// 		switch (inAddress->mSelector) {
// 			case kAudioLevelControlPropertyScalarValue:
// 				*outDataSize = sizeof(Float32);
// 				return kAudioHardwareNoError;

// 			case kAudioLevelControlPropertyDecibelValue:
// 				*outDataSize = sizeof(Float32);
// 				return kAudioHardwareNoError;

// 			case kAudioLevelControlPropertyDecibelRange:
// 				*outDataSize = sizeof(AudioValueRange);
// 				return kAudioHardwareNoError;

// 			case kAudioLevelControlPropertyConvertScalarToDecibels:
// 				*outDataSize = sizeof(Float32);
// 				return kAudioHardwareNoError;

// 			case kAudioLevelControlPropertyConvertDecibelsToScalar:
// 				*outDataSize = sizeof(Float32);
// 				return kAudioHardwareNoError;
// 		}
// 	} else {
// 		if (inAddress->mSelector == kAudioBooleanControlPropertyValue) {
// 			*outDataSize = sizeof(UInt32);
// 			return kAudioHardwareNoError;
// 		}
// 	}

// 	return kAudioHardwareUnknownPropertyError;
// }

// static OSStatus	syfala_GetControlPropertyData(
// 	AudioObjectID const inObjectID,
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32 const inDataSize,
// 	UInt32* const outDataSize,
// 	void* const outData
// ) {
// 	AudioObjectPropertyElement const ctrl_index = control_channel_index(inObjectID);

// 	switch (inAddress->mSelector)
// 	{
// 		case kAudioObjectPropertyOwner:
// 			//	The control's owner is the device object
// 			DoIfFailed(inDataSize < sizeof(AudioObjectID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyOwner for the volume control");
// 			*((AudioObjectID*)outData) = DEVICE_ID;
// 			*outDataSize = sizeof(AudioObjectID);
// 			return kAudioHardwareNoError;

// 		case kAudioObjectPropertyOwnedObjects:
// 			//	Controls do not own any objects
// 			*outDataSize = 0 * sizeof(AudioObjectID);
// 			return kAudioHardwareNoError;

// 		case kAudioControlPropertyScope:
// 			//	This property returns the scope that the control is attached to.
// 			DoIfFailed(inDataSize < sizeof(AudioObjectPropertyScope), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioControlPropertyScope for the volume control");
// 			*((AudioObjectPropertyScope*)outData) = kAudioObjectPropertyScopeOutput;
// 			*outDataSize = sizeof(AudioObjectPropertyScope);
// 			return kAudioHardwareNoError;

// 		case kAudioControlPropertyElement:
// 			//	This property returns the element that the control is attached to.
// 			DoIfFailed(inDataSize < sizeof(AudioObjectPropertyElement), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioControlPropertyElement for the volume control");
// 			*((AudioObjectPropertyElement*)outData) = ctrl_index;
// 			*outDataSize = sizeof(AudioObjectPropertyElement);
// 			return kAudioHardwareNoError;

// 	}

// 	//	Note that for each object, this driver implements all the required properties plus a few
// 	//	extras that are useful but not required.
// 	//
// 	//	Also, since most of the data that will get returned is static, there are few instances where
// 	//	it is necessary to lock the state mutex.
// 	if (control_is_volume(inObjectID)) {
// 		// we are a volume control
// 		switch(inAddress->mSelector)
// 		{
// 			case kAudioObjectPropertyBaseClass:
// 				//	The base class for kAudioVolumeControlClassID is kAudioLevelControlClassID
// 				DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the volume control");
// 				*((AudioClassID*)outData) = kAudioLevelControlClassID;
// 				*outDataSize = sizeof(AudioClassID);
// 				return kAudioHardwareNoError;

// 			case kAudioObjectPropertyClass:
// 				//	Volume controls are of the class, kAudioVolumeControlClassID
// 				DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyClass for the volume control");
// 				*((AudioClassID*)outData) = kAudioVolumeControlClassID;
// 				*outDataSize = sizeof(AudioClassID);
// 				return kAudioHardwareNoError;

// 			case kAudioLevelControlPropertyScalarValue: {
// 				//	This returns the value of the control in the normalized range of 0 to 1.
// 				DoIfFailed(inDataSize < sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyScalarValue for the volume control");
// 				Float32 const gain = atomic_load_explicit(&OUTPUT_VOLUME[ctrl_index], __ATOMIC_RELAXED);
// 				Float32 const db = gain_to_db(gain);
// 				Float32 const db_clamped = fminf(fmaxf(db, VOL_MIN_DB), VOL_MAX_DB);
// 				Float32 const norm = linear_remap(db, VOL_MIN_DB, VOL_DB_RANGE, 0.0f, 1.0f);
// 				*((Float32*)outData) = sq(norm);
// 				*outDataSize = sizeof(Float32);
// 				return kAudioHardwareNoError;
// 			}

// 			case kAudioLevelControlPropertyDecibelValue: {
// 				//	This returns the dB value of the control.
// 				DoIfFailed(inDataSize < sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyDecibelValue for the volume control");

// 				Float32 const gain = atomic_load_explicit(&OUTPUT_VOLUME[ctrl_index], __ATOMIC_RELAXED);
// 				Float32 const db = gain_to_db(gain);
// 				Float32 const db_clamped = fminf(fmaxf(db, VOL_MIN_DB), VOL_MAX_DB);
// 				*((Float32*)outData) = db_clamped;

// 				//	report how much we wrote
// 				*outDataSize = sizeof(Float32);
// 				return kAudioHardwareNoError;
// 			}

// 			case kAudioLevelControlPropertyDecibelRange: {
// 				//	This returns the dB range of the control.
// 				DoIfFailed(inDataSize < sizeof(AudioValueRange), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyDecibelRange for the volume control");

// 				AudioValueRange* const data = (AudioValueRange*) outData;

// 				data->mMinimum = VOL_MIN_DB;
// 				data->mMaximum = VOL_MAX_DB;
// 				*outDataSize = sizeof(AudioValueRange);
// 				return kAudioHardwareNoError;
// 			}

// 			case kAudioLevelControlPropertyConvertScalarToDecibels: {
// 				//	This takes the scalar value in outData and converts it to dB.
// 				DoIfFailed(inDataSize < sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyDecibelValue for the volume control");

// 				Float32* const data = (Float32*) outData;

// 				Float32 const val = sqrtf(fminf(fmaxf(*data, 0.0f), 1.0f));

// 				*data = linear_remap(val, 0.0f, 1.0f, VOL_MIN_DB, VOL_DB_RANGE);

// 				//	report how much we wrote
// 				*outDataSize = sizeof(Float32);
// 				return kAudioHardwareNoError;
// 			}

// 			case kAudioLevelControlPropertyConvertDecibelsToScalar: {
// 				DoIfFailed(inDataSize < sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioLevelControlPropertyScalarValue for the volume control");
// 				//	This takes the dB value in outData and converts it to scalar.
// 				Float32* const data = (Float32*) outData;

// 				Float32 const val = fminf(fmaxf(*data, VOL_MIN_DB), VOL_MAX_DB);

// 				*data = sq(linear_remap(val, VOL_MIN_DB, VOL_DB_RANGE, 0.0f, 1.0f));

// 				//	report how much we wrote
// 				*outDataSize = sizeof(Float32);
// 				return kAudioHardwareNoError;
// 			}
// 		}
// 	} else {
// 		// we are a mute control
// 		switch(inAddress->mSelector)
// 		{
// 			case kAudioObjectPropertyBaseClass:
// 				//	The base class for kAudioMuteControlClassID is kAudioBooleanControlClassID
// 				DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyBaseClass for the mute control");
// 				*((AudioClassID*)outData) = kAudioBooleanControlClassID;
// 				*outDataSize = sizeof(AudioClassID);
// 				return kAudioHardwareNoError;

// 			case kAudioObjectPropertyClass:
// 				//	Mute controls are of the class, kAudioMuteControlClassID
// 				DoIfFailed(inDataSize < sizeof(AudioClassID), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioObjectPropertyClass for the mute control");
// 				*((AudioClassID*)outData) = kAudioMuteControlClassID;
// 				*outDataSize = sizeof(AudioClassID);
// 				return kAudioHardwareNoError;

// 			case kAudioBooleanControlPropertyValue:
// 				//	This returns the value of the mute control where 0 means that mute is off
// 				//	and audio can be heard and 1 means that mute is on and audio cannot be heard.
// 				//	Note that we need to take the state lock to examine this value.
// 				DoIfFailed(inDataSize < sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_GetControlPropertyData: not enough space for the return value of kAudioBooleanControlPropertyValue for the mute control");
// 				*((UInt32*)outData) = (UInt32) atomic_load_explicit(&OUTPUT_MUTE[ctrl_index], __ATOMIC_RELAXED);
// 				*outDataSize = sizeof(UInt32);
// 				return kAudioHardwareNoError;
// 		}
// 	}

// 	return kAudioHardwareUnknownPropertyError;
// }

// static OSStatus	syfala_SetControlPropertyData(
// 	AudioObjectID const inObjectID,
// 	AudioObjectPropertyAddress const* const inAddress,
// 	UInt32 const inDataSize,
// 	void const* const inData,
// 	UInt32* const outNumberPropertiesChanged,
// 	AudioObjectPropertyAddress outChangedAddresses[2]
// ) {
// 	//	initialize the returned number of changed properties
// 	*outNumberPropertiesChanged = 0;

// 	AudioObjectPropertyElement const ctrl_index = control_channel_index(inObjectID);

// 	if (control_is_volume(inObjectID)) {
// 		if (inAddress->mSelector == kAudioLevelControlPropertyScalarValue) {
// 			// Note that if this value changes, it is implied that the dB value changed too.
// 			DoIfFailed(inDataSize != sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_SetControlPropertyData: wrong size for the data for kAudioLevelControlPropertyScalarValue");

// 			Float32 const new_vol_norm = sqrtf(fminf(fmaxf(*((Float32 const*)inData), 0.0f), 1.0f));
// 			Float32 const new_vol_db = linear_remap(new_vol_norm, 0.0f, 1.0f, VOL_MIN_DB, VOL_DB_RANGE);
// 			Float32 const new_vol_gain = db_to_gain(new_vol_db);

// 			atomic_store_explicit(&OUTPUT_VOLUME[ctrl_index], new_vol_gain, __ATOMIC_RELAXED);

// 			*outNumberPropertiesChanged = 2;

// 			AudioObjectPropertyAddress* const first_addr = &outChangedAddresses[0];
// 			first_addr->mSelector = kAudioLevelControlPropertyScalarValue;
// 			first_addr->mScope = kAudioObjectPropertyScopeOutput;
// 			first_addr->mElement = ctrl_index;

// 			AudioObjectPropertyAddress* const second_addr = &outChangedAddresses[1];
// 			second_addr->mSelector = kAudioLevelControlPropertyDecibelValue;
// 			second_addr->mScope = kAudioObjectPropertyScopeOutput;
// 			second_addr->mElement = ctrl_index;

// 			return kAudioHardwareNoError;

// 		} else if (inAddress->mSelector == kAudioLevelControlPropertyDecibelValue) {
// 			//	Note that if this value changes, it is implied that the scalar value changes as well.
// 			DoIfFailed(inDataSize != sizeof(Float32), return kAudioHardwareBadPropertySizeError, "syfala_SetControlPropertyData: wrong size for the data for kAudioLevelControlPropertyScalarValue");

// 			Float32 const new_vol_db = fminf(fmaxf(*((const Float32*)inData), VOL_MIN_DB), VOL_MAX_DB);
// 			Float32 const new_vol_gain = db_to_gain(new_vol_db);

// 			atomic_store_explicit(&OUTPUT_VOLUME[ctrl_index], new_vol_gain, __ATOMIC_RELAXED);

// 			*outNumberPropertiesChanged = 2;

// 			AudioObjectPropertyAddress* const first_addr = &outChangedAddresses[0];
// 			first_addr->mSelector = kAudioLevelControlPropertyDecibelValue;
// 			first_addr->mScope = kAudioObjectPropertyScopeOutput;
// 			first_addr->mElement = ctrl_index;

// 			AudioObjectPropertyAddress* const second_addr = &outChangedAddresses[1];
// 			second_addr->mSelector = kAudioLevelControlPropertyScalarValue;
// 			second_addr->mScope = kAudioObjectPropertyScopeOutput;
// 			second_addr->mElement = ctrl_index;

// 			return kAudioHardwareNoError;
// 		}

// 		return kAudioHardwareUnknownPropertyError;
// 	} else {
// 		if (inAddress->mSelector ==  kAudioBooleanControlPropertyValue) {
// 			DoIfFailed(inDataSize != sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "syfala_SetControlPropertyData: wrong size for the data for kAudioBooleanControlPropertyValue");

// 			atomic_store_explicit(&OUTPUT_MUTE[ctrl_index], *((const UInt32*)inData) != 0, __ATOMIC_RELAXED);

// 			*outNumberPropertiesChanged = 1;

// 			AudioObjectPropertyAddress* const addr = &outChangedAddresses[0];
// 			addr->mSelector = kAudioBooleanControlPropertyValue;
// 			addr->mScope = kAudioObjectPropertyScopeOutput;
// 			addr->mElement = kAudioObjectPropertyElementMain;

// 			return kAudioHardwareNoError;
// 		}

// 		return kAudioHardwareUnknownPropertyError;
// 	}
// }

// #pragma mark IO Operations

// static OSStatus	syfala_StartIO(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inDeviceObjectID,
// 	UInt32 const inClientID
// ) {
// 	DebugMsg("StartIO");

// 	//	This call tells the device that IO is starting for the given client. When this routine
// 	//	returns, the device's clock is running and it is ready to have data read/written. It is
// 	//	important to note that multiple clients can have IO running on the device at the same time.
// 	//	So, work only needs to be done when the first client starts. All subsequent starts simply
// 	//	increment the counter.

// 	#pragma unused(inClientID)

// 	// check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_StartIO: bad driver reference");
// 	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_StartIO: bad device ID");

// 	//	we need to hold the state lock
// 	pthread_mutex_lock(&gDevice_IOMutex);

// 	//	figure out what we need to do
// 	if(gDevice_IOIsRunning == UINT32_MAX) {
// 		//	overflowing is an error
// 		return kAudioHardwareIllegalOperationError;
// 	} else if(gDevice_IOIsRunning == 0) {
// 		//	We need to start the hardware, which in this case is just anchoring the time line.
// 		gDevice_IOIsRunning = 1;
// 		gDevice_NumberTimeStamps = 0;
// 		gDevice_AnchorSampleTime = 0;
// 		gDevice_AnchorHostTime = mach_absolute_time();
// 	} else {
// 		//	IO is already running, so just bump the counter
// 		++gDevice_IOIsRunning;
// 	}

// 	//	unlock the state lock
// 	pthread_mutex_unlock(&gDevice_IOMutex);

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_StopIO(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inDeviceObjectID,
// 	UInt32 const inClientID
// ) {
// 	DebugMsg("StopIO");

// 	//	This call tells the device that the client has stopped IO. The driver can stop the hardware
// 	//	once all clients have stopped.

// 	#pragma unused(inClientID)

// 	// check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_StopIO: bad driver reference");
// 	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_StopIO: bad device ID");

// 	//	we need to hold the state lock
// 	pthread_mutex_lock(&gDevice_IOMutex);

// 	//	figure out what we need to do
// 	if(gDevice_IOIsRunning == 0) {
// 		//	underflowing is an error
// 		return kAudioHardwareIllegalOperationError;
// 	} else if(gDevice_IOIsRunning == 1) {
// 		//	We need to stop the hardware, which in this case means that there's nothing to do.
// 		gDevice_IOIsRunning = 0;
// 	} else {
// 		//	IO is still running, so just bump the counter
// 		--gDevice_IOIsRunning;
// 	}

// 	//	unlock the state lock
// 	pthread_mutex_unlock(&gDevice_IOMutex);

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_GetZeroTimeStamp(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inDeviceObjectID,
// 	UInt32 const inClientID,
// 	Float64* const outSampleTime,
// 	UInt64* const outHostTime,
// 	UInt64* const outSeed
// ) {
// 	DebugMsg("GetZeroTimeStamp");

// 	//	This method returns the current zero time stamp for the device. The HAL models the timing of
// 	//	a device as a series of time stamps that relate the sample time to a host time. The zero
// 	//	time stamps are spaced such that the sample times are the value of
// 	//	kAudioDevicePropertyZeroTimeStampPeriod apart. This is often modeled using a ring buffer
// 	//	where the zero time stamp is updated when wrapping around the ring buffer.
// 	//
// 	//	For this device, the zero time stamps' sample time increments every kDevice_RingBufferSize
// 	//	frames and the host time increments by kDevice_RingBufferSize * gDevice_HostTicksPerFrame.

// 	#pragma unused(inClientID)

// 	// check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_GetZeroTimeStamp: bad driver reference");
// 	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_GetZeroTimeStamp: bad device ID");

// 	//	we need to hold the locks
// 	pthread_mutex_lock(&gDevice_IOMutex);

// 	//	get the current host time
// 	UInt64 const theCurrentHostTime = mach_absolute_time();

// 	//	calculate the next host time
// 	Float64 const theHostTicksPerRingBuffer = gDevice_HostTicksPerFrame * ((Float64)kDevice_RingBufferSize);
// 	Float64 const theHostTickOffset = ((Float64)(gDevice_NumberTimeStamps + 1)) * theHostTicksPerRingBuffer;
// 	UInt64 const theNextHostTime = gDevice_AnchorHostTime + ((UInt64)theHostTickOffset);

// 	//	go to the next time if the next host time is less than the current time
// 	if(theNextHostTime <= theCurrentHostTime) ++gDevice_NumberTimeStamps;

// 	//	set the return values
// 	*outSampleTime = gDevice_NumberTimeStamps * kDevice_RingBufferSize;
// 	*outHostTime = gDevice_AnchorHostTime + (((Float64)gDevice_NumberTimeStamps) * theHostTicksPerRingBuffer);
// 	*outSeed = 1;

// 	//	unlock the state lock
// 	pthread_mutex_unlock(&gDevice_IOMutex);

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_WillDoIOOperation(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inDeviceObjectID,
// 	UInt32 const inClientID,
// 	UInt32 const inOperationID,
// 	Boolean* const outWillDo,
// 	Boolean* const outWillDoInPlace
// ) {
// 	DebugMsg("WillDoIOOperation");

// 	//	This method returns whether or not the device will do a given IO operation. For this device,
// 	//	we only support reading input data and writing output data.

// 	#pragma unused(inClientID)

// 	// check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_WillDoIOOperation: bad driver reference");
// 	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_WillDoIOOperation: bad device ID");

// 	//	figure out if we support the operation
// 	bool willDo = false;
// 	bool willDoInPlace = true;

// 	if (inOperationID == kAudioServerPlugInIOOperationWriteMix) {
// 		willDo = true;
// 		willDoInPlace = true;
// 	}

// 	//	fill out the return values
// 	if(outWillDo != NULL) *outWillDo = willDo;
// 	if(outWillDoInPlace != NULL) *outWillDoInPlace = willDoInPlace;
// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_BeginIOOperation(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inDeviceObjectID,
// 	UInt32 const inClientID,
// 	UInt32 const inOperationID,
// 	UInt32 const inIOBufferFrameSize,
// 	AudioServerPlugInIOCycleInfo const* const inIOCycleInfo
// ) {
// 	//This is called at the beginning of an IO operation. This device doesn't do anything, so just
// 	// check args and return.

// 	DebugMsg("BeginIOOperation");

// 	#pragma unused(inClientID, inOperationID, inIOBufferFrameSize, inIOCycleInfo)

// 	// check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_BeginIOOperation: bad driver reference");
// 	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_BeginIOOperation: bad device ID");

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_DoIOOperation(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inDeviceObjectID,
// 	AudioObjectID const inStreamObjectID,
// 	UInt32 const inClientID,
// 	UInt32 const inOperationID,
// 	UInt32 const inIOBufferFrameSize,
// 	AudioServerPlugInIOCycleInfo const* const inIOCycleInfo,
// 	void* const ioMainBuffer,
// 	void* const ioSecondaryBuffer
// ) {
// 	//	This is called to actually perform a given operation. For this device, all we need to do is
// 	//	clear the buffer for the ReadInput operation.

// 	DebugMsg("DoIOOperation");

// 	#pragma unused(inClientID, inIOCycleInfo, ioSecondaryBuffer)

// 	// check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_DoIOOperation: bad driver reference");
// 	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_DoIOOperation: bad device ID");
// 	DoIfFailed(inStreamObjectID != STREAM_ID, return kAudioHardwareBadObjectError, "syfala_DoIOOperation: bad stream ID");

// 	if(inOperationID == kAudioServerPlugInIOOperationWriteMix) {}

// 	return kAudioHardwareNoError;
// }

// static OSStatus	syfala_EndIOOperation(
// 	AudioServerPlugInDriverRef const inDriver,
// 	AudioObjectID const inDeviceObjectID,
// 	UInt32 const inClientID,
// 	UInt32 const inOperationID,
// 	UInt32 const inIOBufferFrameSize,
// 	AudioServerPlugInIOCycleInfo const* const inIOCycleInfo
// ) {
// 	//	This is called at the end of an IO operation. This device doesn't do anything, so just check
// 	//	the arguments and return.
// 	DebugMsg("EndIOOperation");

// 	#pragma unused(inClientID, inOperationID, inIOBufferFrameSize, inIOCycleInfo)

// 	// check args
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "syfala_EndIOOperation: bad driver reference");
// 	DoIfFailed(inDeviceObjectID != DEVICE_ID, return kAudioHardwareBadObjectError, "syfala_EndIOOperation: bad device ID");

// 	return kAudioHardwareNoError;
// }
