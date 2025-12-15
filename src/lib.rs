use core::{
    iter, mem, num, ops::RangeInclusive, ptr::{self, NonNull}, sync::atomic::{AtomicBool, AtomicU32, Ordering}
};
use std::{panic, sync::{LazyLock, Mutex}};

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
    (FIRST_CTRL_ID..=LAST_CTRL_ID).contains(&id)
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
    f32::mul_add(x - x_start, y_len / x_len, y_start)
}

#[inline(always)]
fn volume_control_db_val(index: usize) -> Option<Float32> {
    OUTPUT_VOLUME.get(index).map(|v| {
        gain_to_db(v.load(Ordering::Relaxed))
            .min(VOL_MAX_DB)
            .max(VOL_MAX_DB)
    })
}

#[inline(always)]
fn volume_control_norm_val(index: usize) -> Option<Float32> {
    volume_control_db_val(index).map(|db| sq(linear_remap(db, VOL_MIN_DB, VOL_DB_RANGE, 0., 1.)))
}

#[inline(always)]
fn norm_to_db(v: Float32) -> Float32 {
    let clamped = v.min(1.).max(0.);
    let skewed = Float32::sqrt(clamped);
    linear_remap(skewed, 0., 1., VOL_MIN_DB, VOL_DB_RANGE)
}

#[inline(always)]
fn norm_to_gain(v: Float32) -> Float32 {
    db_to_gain(norm_to_db(v))
}

#[inline(always)]
fn db_to_norm(db: Float32) -> Float32 {
    let clamped = db.min(VOL_MIN_DB).max(VOL_MAX_DB);
    let gain = linear_remap(clamped, VOL_MIN_DB, VOL_DB_RANGE, 0., 1.);
    sq(gain)
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
// it and `unsafe impl Sync {}`

// All of the plugin's methods are exposed through this struct, which we return a reference to in the factory function
static DRIVER_INTERFACE: AudioServerPlugInDriverInterface = AudioServerPlugInDriverInterface {
    _reserved: core::ptr::null_mut(),
    QueryInterface: Some(query_interface),
    AddRef: Some(add_ref),
    Release: Some(release),
    Initialize: Some(initialize),
    CreateDevice: Some(create_device),
    DestroyDevice: Some(destroy_device),
    AddDeviceClient: Some(add_device_client),
    RemoveDeviceClient: Some(remove_device_client),
    PerformDeviceConfigurationChange: Some(perform_device_configuration_change),
    AbortDeviceConfigurationChange: Some(abort_device_configuration_change),
    HasProperty: Some(has_property),
    IsPropertySettable: Some(is_property_settable),
    GetPropertyDataSize: Some(get_property_data_size),
    GetPropertyData: Some(get_property_data),
    SetPropertyData: Some(set_property_data),
    StartIO: Some(start_io),
    StopIO: Some(stop_io),
    GetZeroTimeStamp: Some(get_zero_timestamp),
    WillDoIOOperation: Some(will_do_io_operation),
    BeginIOOperation: Some(begin_io_operation),
    DoIOOperation: Some(do_io_operation),
    EndIOOperation: Some(end_io_operation),
};

static DRIVER_OBJECT: &AudioServerPlugInDriverInterface = &DRIVER_INTERFACE;

const PLUGIN_BUNDLE_ID: &str = "com.emeraude.coreaudio";

static PLUGIN_REF_COUNT: AtomicU32 = AtomicU32::new(0);
static IS_OUTPUT_STREAM_ACTIVE: AtomicBool = AtomicBool::new(true);

const DEVICE_UID: &str = "UID";

const DEVICE_MODEL_UID: &str = "ModelUID";

// How often should CoreAudio query timestamps (and, if necessary, apply drift correction)?
const TIMESTAMP_PERIOD_NANOSECS: u64 = 500_000_000;
const FRAMES_PER_TIMESTAMP: u32 = (TIMESTAMP_PERIOD_NANOSECS as f64 * SAMPLE_RATE / 1e9 + 0.5) as u32;

struct IOInfo {
    n_active_clients: Option<num::NonZeroU32>,
    current_frame_timestamp: UInt64,
    current_host_timestamp: UInt64,
}

static DEVICE_IO: Mutex<IOInfo> = Mutex::new(IOInfo {
    n_active_clients: None,
    current_frame_timestamp: 0,
    current_host_timestamp: 0,
});

const VOL_MIN_DB: Float32 = -80.;
const VOL_MAX_DB: Float32 = 0.;
const VOL_DB_RANGE: Float32 = VOL_MAX_DB - VOL_MIN_DB;

static OUTPUT_VOLUME: [AtomicF32; N_CHANNELS.strict_add(1) as usize] = {
    let mut out = [const { AtomicF32::new(1.) }; _];
    // mute the master initially for safety
    out[0] = AtomicF32::new(0.);
    out
};
    
static OUTPUT_MUTE: [AtomicBool; N_CHANNELS.strict_add(1) as usize] = 
    [const { AtomicBool::new(false) }; _];

// These IDs are defined as macros in the coreaudio headers, bindgen can't generate them so we do
// it ourselves. We also can't use constants sincee they are created using extern functions can't

// kAudioServerPlugInTypeUUID
#[inline(always)]
#[rustfmt::skip]
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
#[rustfmt::skip]
fn i_unknown_uuid_macro() -> CFUUIDRef {
    // SAFETY: arguments are valid
    unsafe {
        CFUUIDGetConstantUUIDWithBytes(kCFAllocatorSystemDefault, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46)
    }
}

// kAudioServerPlugInDriverInterfaceUUID
#[inline(always)]
#[rustfmt::skip]
fn asp_driver_interface_type_uuid_macro() -> CFUUIDRef {
    // SAFETY: arguments are valid
    unsafe {
        CFUUIDGetConstantUUIDWithBytes(core::ptr::null_mut(), 0xEE, 0xA5, 0x77, 0x3D, 0xCC, 0x43, 0x49, 0xF1, 0x8E, 0x00, 0x8F, 0x96, 0xE7, 0xD2, 0x3B, 0x17)
    }
}

#[inline(always)]
fn cfstr(str: &str) -> CFStringRef {
    unsafe {
        CFStringCreateWithBytes(
            ptr::null(),
            str.as_bytes().as_ptr(),
            str.len().try_into().unwrap(),
            kCFStringEncodingUTF8,
            Boolean::from(false),
        )
    }
}

#[inline(always)]
const fn raw_driver_ptr() -> *mut std::os::raw::c_void {
    (&DRIVER_OBJECT as *const &AudioServerPlugInDriverInterface)
        .cast_mut()
        .cast()
}

// ----
// coreaudio_sys doesn't define these, so we do it ourselves

#[repr(C)]
#[allow(non_camel_case_types)]
pub struct mach_timebase_info_data_t {
    pub numer: u32,
    pub denom: u32,
}

unsafe extern "C" {
    pub safe fn mach_timebase_info(info: *mut mach_timebase_info_data_t) -> i32;
}

unsafe extern "C" {
    pub safe fn mach_absolute_time() -> UInt64;
}

// ----

#[inline(always)]
fn get_timebase_info() -> mach_timebase_info_data_t {
    let mut info = mach_timebase_info_data_t { numer: 0, denom: 0 };
    mach_timebase_info(&mut info);
    info
}

// This is practically a system-wide constant. It sucks that we can't just use a static
#[inline(always)]
fn get_ticks_per_timestamp() -> num::NonZeroU64 {
    let mach_timebase_info_data_t { numer, denom } = get_timebase_info();
    num::NonZeroU64::new(TIMESTAMP_PERIOD_NANOSECS.strict_mul(UInt64::from(numer)).strict_div(u64::from(denom))).unwrap()
}

static TICKS_PER_TIMESTAMP: LazyLock<num::NonZeroU64> = LazyLock::new(get_ticks_per_timestamp);

// This is the only function exported as a symbol by the library, It's name must be indicated in
// the CFPlugInFactories field the bundle's Info.plist file.
#[unsafe(no_mangle)] // <-- important
unsafe extern "C" fn create(
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

    // Well.. there's nothing really we can do if we fail setting up logging
    let _ = log::set_boxed_logger(Box::new(OsLogger::new("com.emeraude.syfala_coreaudio")));

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
const E_NOINTERFACE: HRESULT = 0x80000004u32 as i32;

unsafe extern "C" fn query_interface(
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
        log::error!("query_interface: bad driver reference");
        return kAudioHardwareBadObjectError as HRESULT;
    }

    if out_interface.is_null() {
        log::error!("query_interface: no place to store the returned interface");
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

    kAudioHardwareNoError as HRESULT
}

unsafe extern "C" fn add_ref(driver: *mut std::os::raw::c_void) -> ULONG {
    //	This call returns the resulting reference count after the increment.

    log::debug!("AddRef");

    // check args
    if !eq_driver_ptr(driver) {
        log::error!("add_ref: bad driver reference");
        return 0 as ULONG;
    }

    //	increment the refcount, return the new value
    saturating_refount_inc(&PLUGIN_REF_COUNT).saturating_add(1)
}

unsafe extern "C" fn release(driver: *mut std::os::raw::c_void) -> ULONG {
    //	This call returns the resulting reference count after the decrement.
    log::debug!("Release");

    // check args
    if !eq_driver_ptr(driver) {
        log::error!("add_ref: bad driver reference");
        return 0 as ULONG;
    }

    // decrement the refcount, return the new value
    saturating_refount_dec(&PLUGIN_REF_COUNT).saturating_sub(1)
}

unsafe extern "C" fn initialize(
    driver: AudioServerPlugInDriverRef,
    host: AudioServerPlugInHostRef,
) -> OSStatus {
    //	The job of this method is, as the name implies, to get the driver initialized. Note that when this call returns, the HAL will scan the various lists the driver
    //	maintains (such as the device list) to get the inital set of objects the driver is
    //	publishing. So, there is no need to notifiy the HAL about any objects created as part of the
    //	execution of this method.

    log::debug!("Initialize");

    if !eq_driver_ptr(driver.cast()) {
        log::error!("initialize: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    // One specific thing that needs to be done is to store the AudioServerPlugInHostRef
    // so that it can be used later. This is the object used by our plugin to notify the host
    // of property changes etc.
    // SAFETY: the HAL guarantees this isn't called concurrently, and that `host` outlives
    // our whole plugin ('static lifetime)
    unsafe { HOST = host.as_ref() };

    kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn create_device(
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
        log::error!("create_device: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    kAudioHardwareUnsupportedOperationError as OSStatus
}

unsafe extern "C" fn destroy_device(
    driver: AudioServerPlugInDriverRef,
    _device_object_id: AudioObjectID,
) -> OSStatus {
    //	This method is used to tell a driver that implements the Transport Manager semantics to
    //	destroy an AudioEndpointDevice. Since this driver is not a Transport Manager, we just check
    //	the arguments and return kAudioHardwareUnsupportedOperationError.

    log::debug!("DestroyDevice");

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("destroy_device: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    kAudioHardwareUnsupportedOperationError as OSStatus
}

unsafe extern "C" fn add_device_client(
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
        log::error!("add_device_client: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_object_id != DEVICE_ID {
        log::error!("add_device_client: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn remove_device_client(
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
        log::error!("remove_device_client: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_object_id != DEVICE_ID {
        log::error!("remove_device_client: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn perform_device_configuration_change(
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
        log::error!("perform_device_configuration_change: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_object_id != DEVICE_ID {
        log::error!("perform_device_configuration_change: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn abort_device_configuration_change(
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
        log::error!("abort_device_configuration_change: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_object_id != DEVICE_ID {
        log::error!("abort_device_configuration_change: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn has_property(
    driver: AudioServerPlugInDriverRef,
    object_id: AudioObjectID,
    _client_pid: pid_t,
    addr: *const AudioObjectPropertyAddress,
) -> Boolean {
    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("has_property: bad driver reference");
        return Boolean::from(false);
    }

    let Some(addr) = NonNull::new(addr.cast_mut()) else {
        log::error!("has_property: no address");
        return Boolean::from(false);
    };

    let Ok(res) =
        panic::catch_unwind(|| find_property_data(object_id, unsafe { addr.as_ref() }).is_ok())
    else {
        log::error!("has_property: internal function panicked!!!");
        return Boolean::from(false);
    };

    Boolean::from(res)
}

unsafe extern "C" fn is_property_settable(
    driver: AudioServerPlugInDriverRef,
    object_id: AudioObjectID,
    _client_pid: pid_t,
    addr: *const AudioObjectPropertyAddress,
    is_settable: *mut Boolean,
) -> OSStatus {
    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("is_property_settable: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    let Some(addr) = NonNull::new(addr.cast_mut()) else {
        log::error!("is_property_settable: no address");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    let Some(is_settable) = NonNull::new(is_settable) else {
        log::error!("is_property_settable: no place to put the return value");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    let addr = unsafe { addr.as_ref() };

    let Ok(res) =
        panic::catch_unwind(|| find_property_data(object_id, addr).map(|data| data.read_data.is_some()))
    else {
        log::error!("has_property: internal function panicked!!!");
        return kAudioHardwareUnspecifiedError as OSStatus;
    };

    let output = match res {
        Ok(b) => Boolean::from(b),
        Err(b) => return if b {
            kAudioHardwareUnknownPropertyError
        } else {
            kAudioHardwareBadObjectError
        } as OSStatus,
    };

    unsafe { is_settable.write(output) }

    kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn get_property_data_size(
    driver: AudioServerPlugInDriverRef,
    object_id: AudioObjectID,
    _client_pid: pid_t,
    addr: *const AudioObjectPropertyAddress,
    _qual_size: UInt32,
    _qual_data: *const std::os::raw::c_void,
    data_size: *mut UInt32,
) -> OSStatus {
    // This method returns the byte size of the property's data.

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("get_property_data_size: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    let Some(addr) = NonNull::new(addr.cast_mut()) else {
        log::error!("get_property_data_size: no address");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    let Some(data_size) = NonNull::new(data_size) else {
        log::error!("get_property_data_size: no place to put the return value");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    let addr = unsafe { addr.as_ref() };

    let Ok(res) = panic::catch_unwind(|| {
        find_property_data(object_id, addr).map(|data| (data.get_data_size)(object_id, addr).into_inner().1)
    }) else {
        log::error!("has_property: internal function panicked!!!");
        return kAudioHardwareUnspecifiedError as OSStatus;
    };

    let output = match res {
        Ok(size) => size,
        Err(b) => return if b {
            kAudioHardwareUnknownPropertyError
        } else {
            kAudioHardwareBadObjectError
        } as OSStatus,
    };

    unsafe { data_size.write(output) }

    kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn get_property_data(
    driver: AudioServerPlugInDriverRef,
    object_id: AudioObjectID,
    _client_pid: pid_t,
    addr: *const AudioObjectPropertyAddress,
    qual_size: UInt32,
    qual_data: *const std::os::raw::c_void,
    available_data_size: UInt32,
    written_data_size: *mut UInt32,
    out_data: *mut std::os::raw::c_void,
) -> OSStatus {
    // This method returns the byte size of the property's data.

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("get_property_data: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    let Some(addr) = NonNull::new(addr.cast_mut()) else {
        log::error!("get_property_data: no address");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    let Some(written_data_size) = NonNull::new(written_data_size) else {
        log::error!("get_property_data: no place to put the return value size");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    let Some(out_data) = NonNull::new(out_data) else {
        log::error!("get_property_data: no place to put the return value");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    let addr = unsafe { addr.as_ref() };

    let data = match find_property_data(object_id, addr) {
        Ok(data) => data,
        Err(b) => return if b {
            kAudioHardwareUnknownPropertyError
        } else {
            kAudioHardwareBadObjectError
        } as OSStatus,
    };

    if available_data_size < *(data.get_data_size)(object_id, addr).start() {
        return kAudioHardwareBadPropertySizeError as OSStatus;
    };

    let size = match data.write_data {
        Qualifier::Needed((get_qual_size, write_data)) => {
            let Some(qual_data) = NonNull::new(qual_data.cast_mut()) else {
                return kAudioHardwareIllegalOperationError as OSStatus;
            };

            if qual_size < *get_qual_size(object_id, addr).start() {
                return kAudioHardwareBadPropertySizeError as OSStatus;
            }

            unsafe {
                write_data(
                    object_id,
                    addr,
                    qual_size,
                    qual_data,
                    available_data_size,
                    out_data,
                )
            }
        }
        Qualifier::Unneeded(f) => unsafe {
            f(
                object_id,
                addr,
                qual_size,
                qual_data,
                available_data_size,
                out_data,
            )
        },
    };

    unsafe { written_data_size.write(size) }

    kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn set_property_data(
    driver: AudioServerPlugInDriverRef,
    object_id: AudioObjectID,
    _client_pid: pid_t,
    addr: *const AudioObjectPropertyAddress,
    qual_size: UInt32,
    qual_data: *const std::os::raw::c_void,
    available_data_size: UInt32,
    out_data: *const std::os::raw::c_void,
) -> OSStatus {
    // This method returns the byte size of the property's data.

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("set_property_data: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    let Some(addr) = NonNull::new(addr.cast_mut()) else {
        log::error!("set_property_data: no address");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    let addr = unsafe { addr.as_ref() };

    let data = match find_property_data(object_id, addr) {
        Ok(data) => data,
        Err(b) => return if b {
            kAudioHardwareUnknownPropertyError
        } else {
            kAudioHardwareBadObjectError
        } as OSStatus,
    };

    let Some(out_data) = NonNull::new(out_data.cast_mut()) else {
        log::error!("set_property_data: no place to put the return value");
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    if available_data_size < *(data.get_data_size)(object_id, addr).start() {
        return kAudioHardwareBadPropertySizeError as OSStatus;
    };    

    let Some(read_data) = &data.read_data else {
        return kAudioHardwareUnknownPropertyError as OSStatus; 
    };

    match read_data {
        Qualifier::Needed(f) => {
            if let Some(qual_data) = NonNull::new(qual_data.cast_mut()) {
                unsafe { f(
                    object_id,
                    addr,
                    qual_size,
                    qual_data,
                    available_data_size,
                    out_data,
                ) }
            }
        },
        Qualifier::Unneeded(f) => unsafe {
            f(
                object_id,
                addr,
                qual_size,
                qual_data,
                available_data_size,
                out_data,
            )
        },
    };

    // unsafe { written_data_size.write(size) }

    kAudioHardwareNoError as OSStatus
}

// static OSStatus	SetPropertyData(
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
// 	DoIfFailed(inDriver != DRIVER_REF, return kAudioHardwareBadObjectError, "SetPropertyData: bad driver reference");
// 	DoIfFailed(inAddress == NULL, return kAudioHardwareIllegalOperationError, "SetPropertyData: no address");

// 	// Only 2 * (N_CHANNELS + 1) + 1 properties are settable in our plugin:
// 	//		- Whether the output stream is active.
// 	// 		- The values of the volume and mule controls for each channel + the master controls

// 	if (inObjectID == STREAM_ID) {
// 		if (inAddress->mSelector == kAudioStreamPropertyIsActive) {
// 			//	Changing the active state of a stream doesn't affect IO or change the structure
// 			//	so we can just save the state and send the notification.
// 			DoIfFailed(inDataSize != sizeof(UInt32), return kAudioHardwareBadPropertySizeError, "SetStreamPropertyData: wrong size for the data for kAudioDevicePropertyNominalSampleRate");
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

// 		OSStatus const status = SetControlPropertyData(
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

#[inline(always)]
const fn s<T: Copy>(v: T) -> RangeInclusive<T> {
    v..=v
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Qualifier<T, U> {
    Needed(T),
    Unneeded(U),
}

#[derive(Debug, Clone, Copy, Hash)]
struct PropertyData {
    get_data_size: fn(id: AudioObjectID, &AudioObjectPropertyAddress) -> RangeInclusive<UInt32>,
    write_data: Qualifier<
        (
            fn(id: AudioObjectID, &AudioObjectPropertyAddress) -> RangeInclusive<UInt32>,
            unsafe fn(
                id: AudioObjectID,
                address: &AudioObjectPropertyAddress,
                qual_size_bytes: UInt32,
                // read_only
                qual_data: NonNull<std::os::raw::c_void>,
                output_size_bytes: UInt32,
                output_data: NonNull<std::os::raw::c_void>,
            ) -> UInt32,
        ),
        unsafe fn(
            id: AudioObjectID,
            address: &AudioObjectPropertyAddress,
            qual_size_bytes: UInt32,
            qual_data: *const std::os::raw::c_void,
            output_size_bytes: UInt32,
            output_data: NonNull<std::os::raw::c_void>,
        ) -> UInt32,
    >,
    read_data: Option<
        Qualifier<
            unsafe fn(
                id: AudioObjectID,
                address: &AudioObjectPropertyAddress,
                qual_size_bytes: UInt32,
                // read-only
                qual_data: NonNull<std::os::raw::c_void>,
                input_size_bytes: UInt32,
                // read-only
                input_data: NonNull<std::os::raw::c_void>,
            ),
            unsafe fn(
                id: AudioObjectID,
                address: &AudioObjectPropertyAddress,
                qual_size_bytes: UInt32,
                qual_data: *const std::os::raw::c_void,
                input_size_bytes: UInt32,
                // read-only
                input_data: NonNull<std::os::raw::c_void>,
            ),
        >,
    >,
}

const CLASS_ID_SIZE: UInt32 = size_of::<AudioClassID>() as UInt32;
const OBJECT_ID_SIZE: UInt32 = size_of::<AudioObjectID>() as UInt32;
const CFSTRING_SIZE: UInt32 = size_of::<CFStringRef>() as UInt32;

#[inline(always)]
unsafe fn write_get_size<I: IntoIterator>(
    ptr: NonNull<std::os::raw::c_void>,
    bytes_available: usize,
    values: I,
) -> UInt32 {
    let item_size = size_of::<I::Item>();
    values
        .into_iter()
        .map(|v| unsafe { NonNull::write(ptr.cast(), v) })
        .take(bytes_available / item_size)
        .count()
        .strict_mul(item_size)
        .try_into()
        .unwrap()
}

#[inline(always)]
unsafe fn write_once_get_size<T>(ptr: NonNull<std::os::raw::c_void>, value: T) -> UInt32 {
    unsafe { write_get_size(ptr, size_of::<T>(), iter::once(value)) }
}

// Ok(...) no error
// Ok(false) = kAudioHardwareBadObjectError
// OK(true) = kAudioHardwareUnknownPropertyError
fn find_property_data(
    id: AudioObjectID,
    address: &AudioObjectPropertyAddress,
) -> Result<&'static PropertyData, bool> {
    match id {
        PLUGIN_ID => find_plugin_property_data(address).ok_or(true),
        DEVICE_ID => find_device_property_data(address).ok_or(true),
        STREAM_ID => find_stream_property_data(address).ok_or(true),
        ctrl if audio_object_is_control(ctrl) => find_control_property_data(
            control_is_volume(ctrl),
            control_channel_index(id).try_into().unwrap(),
            address,
        ).ok_or(true),
        _ => Err(false),
    }
}

#[allow(non_upper_case_globals)]
fn find_plugin_property_data(
    address: &AudioObjectPropertyAddress,
) -> Option<&'static PropertyData> {
    if address.mScope != kAudioObjectPropertyScopeGlobal
        || address.mElement != kAudioObjectPropertyElementMain
    {
        return None;
    }

    match address.mSelector {
        kAudioObjectPropertyBaseClass => Some(&PropertyData {
            get_data_size: |_, _| s(CLASS_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioObjectClassID)
            }),
            read_data: None,
        }),
        kAudioObjectPropertyClass => Some(&PropertyData {
            get_data_size: |_, _| s(CLASS_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioPlugInClassID)
            }),
            read_data: None,
        }),
        kAudioObjectPropertyOwner => Some(&PropertyData {
            get_data_size: |_, _| s(OBJECT_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioObjectUnknown)
            }),
            read_data: None,
        }),
        kAudioObjectPropertyManufacturer => Some(&PropertyData {
            get_data_size: |_, _| s(CFSTRING_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, cfstr("GRAME CNCM"))
            }),
            read_data: None,
        }),
        kAudioObjectPropertyOwnedObjects | kAudioPlugInPropertyDeviceList => Some(&PropertyData {
            get_data_size: |_, _| s(OBJECT_ID_SIZE.strict_mul(1)),
            write_data: Qualifier::Unneeded(|_, _, _, _, size, ptr| unsafe {
                write_get_size(ptr, size.try_into().unwrap(), [DEVICE_ID])
            }),
            read_data: None,
        }),
        kAudioPlugInPropertyTranslateUIDToDevice => Some(&PropertyData {
            get_data_size: |_, _| s(OBJECT_ID_SIZE),
            write_data: Qualifier::Needed((
                |_, _| s(CFSTRING_SIZE),
                |_, _, _, qual_data, _, out_data| {
                    let compare_res: CFComparisonResult =
                        unsafe { CFStringCompare(qual_data.cast().read(), cfstr(DEVICE_UID), 0) };
                    let equal: CFComparisonResult = kCFCompareEqualTo;

                    let id = if compare_res == equal {
                        DEVICE_ID
                    } else {
                        kAudioObjectUnknown
                    };

                    unsafe { out_data.cast().write(id) };
                    OBJECT_ID_SIZE
                },
            )),
            read_data: None,
        }),
        kAudioPlugInPropertyResourceBundle => Some(&PropertyData {
            get_data_size: |_, _| s(CFSTRING_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, cfstr(""))
            }),
            read_data: None,
        }),
        _ => None,
    }
}

#[allow(non_upper_case_globals)]
fn find_device_property_data(
    address: &AudioObjectPropertyAddress,
) -> Option<&'static PropertyData> {
    // NOTE: For some properties we must check scope or element. We perform those checks
    // on a per-case basis to match the C implementation.

    let AudioObjectPropertyAddress {
        mSelector: selector,
        mScope: scope,
        mElement: elem,
    } = *address;

    if let kAudioObjectPropertyElementName = selector
        && (0..=N_CHANNELS).contains(&elem)
    {
        return Some(&PropertyData {
            get_data_size: |_, _| s(CFSTRING_SIZE),
            write_data: Qualifier::Unneeded(|_, addr, _, _, _, out_ptr| {
                let string = match addr.mElement {
                    kAudioObjectPropertyElementMain /* AKA 0 */ => String::from("Master"),
                    i @ 1..N_CHANNELS => i.to_string(),
                    // unreachable but we do that to not panic
                    _ => String::from("<Unknwon>"),
                };

                let cfstring = cfstr(string.as_str());

                unsafe { write_once_get_size(out_ptr, cfstring) }
            }),
            read_data: None,
        });
    }

    if [
        kAudioObjectPropertyScopeOutput,
        kAudioObjectPropertyScopeGlobal,
    ]
    .contains(&scope)
    {
        match selector {
            kAudioDevicePropertyDeviceCanBeDefaultDevice => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                        write_once_get_size(ptr, 1u32)
                    }),
                    read_data: None,
                });
            }

            kAudioDevicePropertyDeviceCanBeDefaultSystemDevice => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                        write_once_get_size(ptr, 1u32)
                    }),
                    read_data: None,
                });
            }

            kAudioDevicePropertyLatency => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                        write_once_get_size(ptr, 0u32)
                    }),
                    read_data: None,
                });
            }

            kAudioDevicePropertySafetyOffset => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                        write_once_get_size(ptr, 0u32)
                    }),
                    read_data: None,
                });
            }

            kAudioDevicePropertyPreferredChannelLayout => {
                return Some(&PropertyData {
                    get_data_size: |_, _| {
                        s(size_of::<AudioChannelDescription>()
                            .strict_mul(1)
                            .strict_add(mem::offset_of!(AudioChannelLayout, mChannelDescriptions))
                            .try_into()
                            .unwrap())
                    },
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, out_ptr| {
                        let layout = unsafe { out_ptr.cast::<AudioChannelLayout>().as_mut() };

                        // fill layout as in C
                        layout.mChannelLayoutTag = kAudioChannelLayoutTag_UseChannelDescriptions;
                        layout.mChannelBitmap = 0;
                        layout.mNumberChannelDescriptions = 1;

                        let chann_desc = unsafe {
                            core::slice::from_raw_parts_mut(
                                layout.mChannelDescriptions.as_mut_ptr(),
                                layout.mNumberChannelDescriptions.try_into().unwrap(),
                            )
                        };

                        chann_desc[0] = AudioChannelDescription {
                            mChannelLabel: 1,
                            mChannelFlags: 0,
                            mCoordinates: [0., 0., 0.],
                        };

                        size_of::<AudioChannelDescription>()
                            .strict_mul(1)
                            .strict_add(mem::offset_of!(AudioChannelLayout, mChannelDescriptions))
                            .try_into()
                            .unwrap()
                    }),
                    read_data: None,
                });
            }
            _ => (),
        }
    }

    match address.mSelector {
        kAudioObjectPropertyBaseClass => Some(&PropertyData {
            get_data_size: |_, _| s(CLASS_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioObjectClassID)
            }),
            read_data: None,
        }),

        kAudioObjectPropertyClass => Some(&PropertyData {
            get_data_size: |_, _| s(CLASS_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioDeviceClassID)
            }),
            read_data: None,
        }),

        kAudioObjectPropertyOwner => Some(&PropertyData {
            get_data_size: |_, _| s(OBJECT_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, PLUGIN_ID)
            }),
            read_data: None,
        }),

        kAudioObjectPropertyName => Some(&PropertyData {
            get_data_size: |_, _| s(CFSTRING_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, cfstr("Space Bar"))
            }),
            read_data: None,
        }),

        kAudioObjectPropertyManufacturer => Some(&PropertyData {
            get_data_size: |_, _| s(CFSTRING_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, cfstr("GRAME"))
            }),
            read_data: None,
        }),

        kAudioObjectPropertyOwnedObjects => Some(&PropertyData {
            get_data_size: |_, _| s(N_DEVICE_OWNED_OBJS.strict_mul(OBJECT_ID_SIZE)),
            write_data: Qualifier::Unneeded(|_, addr, _, _, out_size, out_ptr| {
                if [
                    kAudioObjectPropertyScopeGlobal,
                    kAudioObjectPropertyScopeOutput,
                ]
                .contains(&addr.mScope)
                {
                    unsafe {
                        write_get_size(
                            out_ptr,
                            out_size.try_into().unwrap(),
                            iter::chain(iter::once(STREAM_ID), FIRST_CTRL_ID..=LAST_CTRL_ID),
                        )
                    }
                } else {
                    0
                }
            }),
            read_data: None,
        }),

        kAudioObjectPropertyControlList => Some(&PropertyData {
            get_data_size: |_, _| s(N_CONTROLS.strict_mul(OBJECT_ID_SIZE)),
            write_data: Qualifier::Unneeded(|_, _, _, _, out_size, out_ptr| unsafe {
                write_get_size(
                    out_ptr,
                    out_size.try_into().unwrap(),
                    FIRST_CTRL_ID..=LAST_CTRL_ID,
                )
            }),
            read_data: None,
        }),

        kAudioDevicePropertyDeviceUID => Some(&PropertyData {
            get_data_size: |_, _| s(CFSTRING_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, cfstr(DEVICE_UID))
            }),
            read_data: None,
        }),

        kAudioDevicePropertyModelUID => Some(&PropertyData {
            get_data_size: |_, _| s(CFSTRING_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, cfstr(DEVICE_MODEL_UID))
            }),
            read_data: None,
        }),

        kAudioDevicePropertyTransportType => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioDeviceTransportTypeAVB)
            }),
            read_data: None,
        }),

        kAudioDevicePropertyRelatedDevices => Some(&PropertyData {
            get_data_size: |_, _| s(OBJECT_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, out_size, out_ptr| unsafe {
                write_get_size(out_ptr, out_size.try_into().unwrap(), [DEVICE_ID])
            }),
            read_data: None,
        }),

        kAudioDevicePropertyClockDomain => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, 0u32)
            }),
            read_data: None,
        }),

        kAudioDevicePropertyDeviceIsAlive => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, 1u32)
            }),
            read_data: None,
        }),

        kAudioDevicePropertyDeviceIsRunning => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, DEVICE_IO.lock().unwrap().n_active_clients)
            }),
            read_data: None,
        }),

        kAudioDevicePropertyNominalSampleRate => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<Float64>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, SAMPLE_RATE)
            }),
            read_data: None,
        }),

        kAudioDevicePropertyAvailableNominalSampleRates => Some(&PropertyData {
            get_data_size: |_, _| {
                s(size_of::<AudioValueRange>()
                    .strict_mul(1)
                    .try_into()
                    .unwrap())
            },
            write_data: Qualifier::Unneeded(|_, _, _, _, out_size, out_ptr| unsafe {
                write_get_size(
                    out_ptr,
                    out_size.try_into().unwrap(),
                    [AudioValueRange {
                        mMinimum: SAMPLE_RATE,
                        mMaximum: SAMPLE_RATE,
                    }],
                )
            }),
            read_data: None,
        }),

        kAudioDevicePropertyIsHidden => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, 0u32)
            }),
            read_data: None,
        }),

        kAudioDevicePropertyZeroTimeStampPeriod => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, FRAMES_PER_TIMESTAMP)
            }),
            read_data: None,
        }),

        // Icon (CFURLRef) — needs bundle lookup
        kAudioDevicePropertyIcon => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<CFURLRef>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| {
                // Follow the same logic as C: get bundle, copy resource URL
                let bundle = unsafe { CFBundleGetBundleWithIdentifier(cfstr(PLUGIN_BUNDLE_ID)) };
                if bundle.is_null() {
                    // No bundle — write NULL to output (C code returned error; here we return 0)
                    unsafe { write_once_get_size(ptr, ptr::null_mut::<()>() as CFURLRef) }
                } else {
                    let url = unsafe {
                        CFBundleCopyResourceURL(
                            bundle,
                            cfstr("DeviceIcon.icns"),
                            ptr::null(),
                            ptr::null(),
                        )
                    };
                    unsafe { write_once_get_size(ptr, url) }
                }
            }),
            read_data: None,
        }),

        // Streams (scope-aware)
        kAudioDevicePropertyStreams => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<AudioObjectID>().strict_mul(1).try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, addr, _, _, out_size, out_ptr| {
                if [
                    kAudioObjectPropertyScopeGlobal,
                    kAudioObjectPropertyScopeOutput,
                ]
                .contains(&addr.mScope)
                {
                    unsafe { write_get_size(out_ptr, out_size.try_into().unwrap(), [STREAM_ID]) }
                } else {
                    0
                }
            }),
            read_data: None,
        }),

        _ => None,
    }
}

#[allow(non_upper_case_globals)]
fn find_stream_property_data(
    address: &AudioObjectPropertyAddress,
) -> Option<&'static PropertyData> {
    match address.mSelector {
        kAudioObjectPropertyBaseClass => Some(&PropertyData {
            get_data_size: |_, _| s(CLASS_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioObjectClassID)
            }),
            read_data: None,
        }),

        kAudioObjectPropertyClass => Some(&PropertyData {
            get_data_size: |_, _| s(CLASS_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioStreamClassID)
            }),
            read_data: None,
        }),

        kAudioObjectPropertyOwner => Some(&PropertyData {
            get_data_size: |_, _| s(OBJECT_ID_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, DEVICE_ID)
            }),
            read_data: None,
        }),

        kAudioObjectPropertyOwnedObjects => Some(&PropertyData {
            // streams don't own any objects
            get_data_size: |_, _| 0..=OBJECT_ID_SIZE.strict_mul(0),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, _| 0),
            read_data: None,
        }),

        kAudioObjectPropertyName => Some(&PropertyData {
            get_data_size: |_, _| s(CFSTRING_SIZE),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, cfstr("Output Stream"))
            }),
            read_data: None,
        }),

        kAudioStreamPropertyDirection => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, 0)
            }),
            read_data: None,
        }),

        kAudioStreamPropertyTerminalType => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, kAudioStreamTerminalTypeUnknown)
            }),
            read_data: None,
        }),

        kAudioStreamPropertyStartingChannel => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, 1)
            }),
            read_data: None,
        }),

        kAudioStreamPropertyLatency => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(ptr, 0)
            }),
            read_data: None,
        }),

        kAudioStreamPropertyAvailableVirtualFormats
        | kAudioStreamPropertyAvailablePhysicalFormats => Some(&PropertyData {
            get_data_size: |_, _| {
                s(size_of::<AudioStreamRangedDescription>()
                    .strict_mul(1)
                    .try_into()
                    .unwrap())
            },
            write_data: Qualifier::Unneeded(|_, _, _, _, data_size, ptr| unsafe {
                write_get_size(ptr, data_size.try_into().unwrap(), [FORMAT])
            }),
            read_data: None,
        }),

        kAudioStreamPropertyVirtualFormat | kAudioStreamPropertyPhysicalFormat => {
            Some(&PropertyData {
                get_data_size: |_, _| {
                    s(size_of::<AudioStreamBasicDescription>().try_into().unwrap())
                },
                write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                    write_once_get_size(ptr, FORMAT.mFormat)
                }),
                read_data: None,
            })
        }

        kAudioStreamPropertyIsActive => Some(&PropertyData {
            get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
            write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                write_once_get_size(
                    ptr,
                    UInt32::from(IS_OUTPUT_STREAM_ACTIVE.load(Ordering::Relaxed)),
                )
            }),
            read_data: Some(Qualifier::Unneeded(|obj_id, addr, _, _, _, data_ptr| {
                let data_ptr = data_ptr.cast::<UInt32>();
                let new_state = unsafe { data_ptr.read() } != 0;

                if IS_OUTPUT_STREAM_ACTIVE.swap(new_state, Ordering::Relaxed) != new_state
                    && let Some(host) = unsafe { HOST }
                        && let Some(properties_changed_callback) = host.PropertiesChanged {
                            unsafe { properties_changed_callback(host, obj_id, 1, addr) };
                        }
            })),
        }),
        _ => None,
    }
}

#[allow(non_upper_case_globals)]
fn find_control_property_data(
    is_volume: bool,
    _ctrl_index: usize,
    address: &AudioObjectPropertyAddress,
) -> Option<&'static PropertyData> {
    let selector = address.mSelector;

    match selector {
        kAudioObjectPropertyOwner => {
            return Some(&PropertyData {
                get_data_size: |_, _| s(OBJECT_ID_SIZE),
                write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                    write_once_get_size(ptr, DEVICE_ID)
                }),
                read_data: None,
            });
        }

        kAudioObjectPropertyOwnedObjects => {
            return Some(&PropertyData {
                // controls don't own any objects
                get_data_size: |_, _| 0..=OBJECT_ID_SIZE.strict_mul(0),
                write_data: Qualifier::Unneeded(|_, _, _, _, _, _| 0),
                read_data: None,
            });
        }

        //	This property returns the scope that the control is attached to.
        kAudioControlPropertyScope => {
            return Some(&PropertyData {
                get_data_size: |_, _| s(size_of::<AudioObjectPropertyScope>().try_into().unwrap()),
                write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                    write_once_get_size(ptr, kAudioObjectPropertyScopeOutput)
                }),
                read_data: None,
            });
        }

        //	This property returns the element that the control is attached to.
        kAudioControlPropertyElement => {
            return Some(&PropertyData {
                get_data_size: |_, _| {
                    s(size_of::<AudioObjectPropertyElement>().try_into().unwrap())
                },
                write_data: Qualifier::Unneeded(|id, _, _, _, _, ptr| {
                    let ctrl_idx: AudioObjectPropertyElement = control_channel_index(id);
                    unsafe { write_once_get_size(ptr, ctrl_idx) }
                }),
                read_data: None,
            });
        }

        _ => (),
    }

    if is_volume {
        // we are a volume control
        match selector {
            kAudioObjectPropertyBaseClass => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(CLASS_ID_SIZE),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                        write_once_get_size(ptr, kAudioLevelControlClassID)
                    }),
                    read_data: None,
                });
            }

            kAudioObjectPropertyClass => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(CLASS_ID_SIZE),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                        write_once_get_size(ptr, kAudioVolumeControlClassID)
                    }),
                    read_data: None,
                });
            }

            kAudioLevelControlPropertyScalarValue => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<Float32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|id, _, _, _, _, ptr| unsafe {
                        let ctrl_idx = usize::try_from(control_channel_index(id)).unwrap();
                        write_once_get_size(ptr, volume_control_norm_val(ctrl_idx).unwrap())
                    }),
                    read_data: Some(Qualifier::Unneeded(|id, addr, _, _, _, data_ptr| {
                        let ctrl_elem = control_channel_index(id);
                        let ctrl_idx: usize = ctrl_elem.try_into().unwrap();

                        let data_ptr = data_ptr.cast::<Float32>();
                        let new_norm_val = unsafe { data_ptr.read() };

                        let new_gain = norm_to_gain(new_norm_val);

                        OUTPUT_VOLUME
                            .get(ctrl_idx)
                            .unwrap()
                            .store(new_gain, Ordering::Relaxed);

                        if let Some(host) = unsafe { HOST }
                            && let Some(properties_changed_callback) = host.PropertiesChanged {
                                // Note that if this value changes, it is implied
                                // that the decibel value changed as well
                                let changed_prop_addrs = [
                                    *addr,
                                    AudioObjectPropertyAddress {
                                        mSelector: kAudioLevelControlPropertyDecibelValue,
                                        ..*addr
                                    },
                                ];

                                unsafe {
                                    properties_changed_callback(
                                        host,
                                        id,
                                        changed_prop_addrs.len().try_into().unwrap(),
                                        changed_prop_addrs.as_ptr(),
                                    )
                                };
                            }
                    })),
                });
            }

            kAudioLevelControlPropertyDecibelValue => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<Float32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|id, _, _, _, _, ptr| unsafe {
                        let ctrl_idx = usize::try_from(control_channel_index(id)).unwrap();
                        write_once_get_size(ptr, volume_control_db_val(ctrl_idx).unwrap())
                    }),
                    read_data: Some(Qualifier::Unneeded(|id, addr, _, _, _, data_ptr| {
                        let ctrl_elem = control_channel_index(id);
                        let ctrl_idx: usize = ctrl_elem.try_into().unwrap();

                        let data_ptr = data_ptr.cast::<Float32>();
                        let new_db_val = unsafe { data_ptr.read() };

                        let new_gain = db_to_gain(new_db_val);

                        OUTPUT_VOLUME
                            .get(ctrl_idx)
                            .unwrap()
                            .store(new_gain, Ordering::Relaxed);

                        if let Some(host) = unsafe { HOST }
                            && let Some(properties_changed_callback) = host.PropertiesChanged {
                                // Note that if this value changes, it is implied
                                // that the scalar value changed as well
                                let changed_prop_addrs = [
                                    *addr,
                                    AudioObjectPropertyAddress {
                                        mSelector: kAudioLevelControlPropertyScalarValue,
                                        ..*addr
                                    },
                                ];

                                unsafe {
                                    properties_changed_callback(
                                        host,
                                        id,
                                        changed_prop_addrs.len().try_into().unwrap(),
                                        changed_prop_addrs.as_ptr(),
                                    )
                                };
                            }
                    })),
                });
            }

            kAudioLevelControlPropertyDecibelRange => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<AudioValueRange>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| {
                        const RANGE: AudioValueRange = AudioValueRange {
                            mMaximum: VOL_MAX_DB as Float64,
                            mMinimum: VOL_MIN_DB as Float64,
                        };
                        unsafe { write_once_get_size(ptr, RANGE) }
                    }),
                    read_data: None,
                });
            }

            kAudioLevelControlPropertyConvertScalarToDecibels => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<Float32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| {
                        let val_ptr = ptr.cast::<Float32>();
                        let norm_val = unsafe { val_ptr.read() };
                        unsafe { write_once_get_size(ptr, norm_to_db(norm_val)) }
                    }),
                    read_data: None,
                });
            }

            kAudioLevelControlPropertyConvertDecibelsToScalar => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<Float32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| {
                        let val_ptr = ptr.cast::<Float32>();
                        let db_val = unsafe { val_ptr.read() };
                        unsafe { write_once_get_size(ptr, db_to_norm(db_val)) }
                    }),
                    read_data: None,
                });
            }

            _ => (),
        }
    } else {
        match selector {
            kAudioObjectPropertyBaseClass => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(CLASS_ID_SIZE),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                        write_once_get_size(ptr, kAudioBooleanControlClassID)
                    }),
                    read_data: None,
                });
            }

            kAudioObjectPropertyClass => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(CLASS_ID_SIZE),
                    write_data: Qualifier::Unneeded(|_, _, _, _, _, ptr| unsafe {
                        write_once_get_size(ptr, kAudioMuteControlClassID)
                    }),
                    read_data: None,
                });
            }

            kAudioBooleanControlPropertyValue => {
                return Some(&PropertyData {
                    get_data_size: |_, _| s(size_of::<UInt32>().try_into().unwrap()),
                    write_data: Qualifier::Unneeded(|id, _, _, _, _, ptr| {
                        let ctrl_index: usize = usize::try_from(control_channel_index(id)).unwrap();
                        let mute_control = OUTPUT_MUTE.get(ctrl_index).unwrap();
                        let output = UInt32::from(mute_control.load(Ordering::Relaxed));
                        unsafe { write_once_get_size(ptr, output) }
                    }),
                    read_data: Some(Qualifier::Unneeded(|id, addr, _, _, _, data_ptr| {
                        let ctrl_elem = control_channel_index(id);
                        let ctrl_idx: usize = ctrl_elem.try_into().unwrap();

                        let data_ptr = data_ptr.cast::<UInt32>();
                        let new_val = unsafe { data_ptr.read() };

                        OUTPUT_MUTE
                            .get(ctrl_idx)
                            .unwrap()
                            .store(new_val != 0, Ordering::Relaxed);

                        if let Some(host) = unsafe { HOST }
                            && let Some(properties_changed_callback) = host.PropertiesChanged {
                                unsafe { properties_changed_callback(host, id, 1, addr) };
                            }
                    })),
                });
            }

            _ => (),
        }
    }

    None
}

unsafe extern "C" fn start_io(
	driver: AudioServerPlugInDriverRef,
	device_id: AudioObjectID,
	_client_id: UInt32,
) -> OSStatus {
	log::debug!("StartIO");

	//	This call tells the device that IO is starting for the given client. When this routine
	//	returns, the device's clock is running and it is ready to have data read/written. It is
	//	important to note that multiple clients can have IO running on the device at the same time.
	//	So, work only needs to be done when the first client starts. All subsequent starts simply
	//	increment the counter

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("start_io: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_id != DEVICE_ID {
        log::error!("start_io: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    let mut device_io = DEVICE_IO.lock().unwrap();

    let new_n_clients = if let Some(n_clients) = device_io.n_active_clients {
        // we have already started io, just increment the counter
        if let Some(n) = n_clients.checked_add(1) {
            n
        } else {
            // That would be a bit too many clients innit
            return kAudioHardwareIllegalOperationError as OSStatus;
        }
    } else {
        device_io.current_host_timestamp = mach_absolute_time();
        device_io.current_frame_timestamp = 0;
        num::NonZeroU32::MIN
    };

    device_io.n_active_clients = Some(new_n_clients);

	kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn stop_io(
	driver: AudioServerPlugInDriverRef,
	device_id: AudioObjectID,
    _client_id: UInt32,
) -> OSStatus {
	log::debug!("StopIO");

	//	This call tells the device that the client has stopped IO. The driver can stop the hardware
	//	once all clients have stopped.

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("stop_io: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_id != DEVICE_ID {
        log::error!("stop_io: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    let n_active_clients = &mut DEVICE_IO.lock().unwrap().n_active_clients;

    let Some(n_clients) = n_active_clients else {
        // a client has stopped io while we think there are, in fact, none doing io, error out
        return kAudioHardwareIllegalOperationError as OSStatus;
    };

    // no checked_sub for nonzeros? me very sad
    if let Some(new_n_clients) = num::NonZeroU32::new(n_clients.get().strict_sub(1)) {
        *n_clients = new_n_clients;
    } else {
        // we are the last client, here is where we would stop the hardware
        // but in our case that doesn't mean anything
        *n_active_clients = None;
    }

	kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn get_zero_timestamp(
	driver: AudioServerPlugInDriverRef,
	device_id: AudioObjectID,
	_client_id: UInt32,
	sample_time: *mut Float64,
	host_time: *mut UInt64,
	seed: *mut UInt64,
) -> OSStatus {

	//	This method returns the current zero time stamp for the device. The HAL models the timing of
	//	a device as a series of time stamps that relate the sample time to a host time. The zero
	//	time stamps are spaced such that the sample times are the value of
	//	kAudioDevicePropertyZeroTimeStampPeriod apart. This is often modeled using a ring buffer
	//	where the zero time stamp is updated when wrapping around the ring buffer.
	//
	//	For this device, the zero time stamps' sample time increments every kDevice_RingBufferSize
	//	frames and the host time increments by kDevice_RingBufferSize * gDevice_HostTicksPerFrame.

    // Addendum: this method is how coreaudio determines when/how to apply drift correction.

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("get_zero_timestamp: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_id != DEVICE_ID {
        log::error!("get_zero_timestamp: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    //	get the current host time
    let current_time = mach_absolute_time();
    let n_ticks_per_timestamp = *TICKS_PER_TIMESTAMP;

    let mut device_io = DEVICE_IO.lock().unwrap();

    let elapsed_time_ticks = current_time.strict_sub(device_io.current_host_timestamp);
    let n_timestamps_advanced = elapsed_time_ticks / n_ticks_per_timestamp; 
    device_io.current_host_timestamp = device_io.current_host_timestamp.strict_add(n_ticks_per_timestamp.get().strict_mul(n_timestamps_advanced));
    device_io.current_frame_timestamp = device_io.current_frame_timestamp.strict_add(u64::from(FRAMES_PER_TIMESTAMP).strict_mul(n_timestamps_advanced));
    
	//	set the return values
	unsafe { sample_time.write(device_io.current_frame_timestamp as f64) };
	unsafe { host_time.write(device_io.current_host_timestamp) };
	unsafe { seed.write(1) };

	kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn will_do_io_operation(
	driver: AudioServerPlugInDriverRef,
	device_id: AudioObjectID,
	_client_id: UInt32,
	operation_id: UInt32,
	will_do: *mut Boolean,
	will_do_in_place: *mut Boolean,
) -> OSStatus {
	//	This method returns whether or not the device will do a given IO operation. For this device,
	//	we only support reading input data and writing output data.

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("will_do_io_operation: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_id != DEVICE_ID {
        log::error!("will_do_io_operation: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

	//	figure out if we support the operation
	let mut will = false;
	let mut will_in_place = true;

	if operation_id == kAudioServerPlugInIOOperationWriteMix {
		will = true;
		will_in_place = true;
	}

    if let Some(will_do) = NonNull::new(will_do) {
        unsafe { will_do.write(Boolean::from(will)) };
    }
    
    if let Some(will_do_in_place) = NonNull::new(will_do_in_place) {
        unsafe { will_do_in_place.write(Boolean::from(will_in_place)) };
    }

	kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn begin_io_operation(
	driver: AudioServerPlugInDriverRef,
	device_id: AudioObjectID,
	_client_id: UInt32,
	_operation_id: UInt32,
	_io_buffer_frame_size: UInt32,
    _io_cycle_info: *const AudioServerPlugInIOCycleInfo,
) -> OSStatus {
	// This is called at the beginning of an IO operation. This device doesn't do anything, so just
	// check args and return.

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("begin_io_operation: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_id != DEVICE_ID {
        log::error!("begin_io_operation: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

	kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn do_io_operation(
	driver: AudioServerPlugInDriverRef,
	device_id: AudioObjectID,
    stream_id: AudioObjectID,
	_client_id: UInt32,
	operation_id: UInt32,
	_io_buffer_frame_size: UInt32,
    _io_cycle_info: *const AudioServerPlugInIOCycleInfo,
	_io_main_buffer: *mut std::os::raw::c_void,
	_io_secondary_buffer: *mut std::os::raw::c_void,
) -> OSStatus {
	//	This is called to actually perform a given operation. For this device, all we need to do is
	//	clear the buffer for the ReadInput operation.

	// check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("do_io_operation: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_id != DEVICE_ID {
        log::error!("do_io_operation: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if stream_id != STREAM_ID {
        log::error!("do_io_operation: bad stream ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

	if operation_id == kAudioServerPlugInIOOperationWriteMix {}

	kAudioHardwareNoError as OSStatus
}

unsafe extern "C" fn end_io_operation(
	driver: AudioServerPlugInDriverRef,
	device_id: AudioObjectID,
	_client_id: UInt32,
	_operation_id: UInt32,
	_io_buffer_frame_size: UInt32,
    _io_cycle_info: *const AudioServerPlugInIOCycleInfo,
) -> OSStatus {
	// This is called at the beginning of an IO operation. This device doesn't do anything, so just
	// check args and return.

    // check args
    if !eq_driver_ptr(driver.cast()) {
        log::error!("end_io_operation: bad driver reference");
        return kAudioHardwareBadObjectError as OSStatus;
    }

    if device_id != DEVICE_ID {
        log::error!("end_io_operation: bad device ID");
        return kAudioHardwareBadObjectError as OSStatus;
    }

	kAudioHardwareNoError as OSStatus
}