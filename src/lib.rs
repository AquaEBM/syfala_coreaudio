#[unsafe(no_mangle)]
pub static N_CHANNELS: u32 = 16;

#[unsafe(no_mangle)]
pub static SAMPLE_RATE: f64 = 48000.0;

// use core::{
//     ffi::{CStr, c_char, c_void},
//     net::{IpAddr, Ipv4Addr},
//     num::NonZeroU32,
//     ptr::NonNull,
// };
// use std::thread::{self, Thread};
// fn as_bytes<T>(data: &[T]) -> &[u8] {
//     // SAFETY: all bit patterns for u8 are valid, references have same lifetime and location
//     unsafe { core::slice::from_raw_parts(data.as_ptr().cast(), size_of::<T>() * data.len()) }
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn rust_fat_sine(x: f32) -> f32 {
//     f32::copysign(1.0, x.fract() - 0.5)
// }

// #[repr(C)]
// struct NetworkTx {
//     tx: rtrb::Producer<f32>,
//     thread: Thread,
// }

// impl NetworkTx {
//     #[inline]
//     pub fn new(tx: rtrb::Producer<f32>, thread: Thread) -> Self {
//         return Self { tx, thread };
//     }
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn launch_network_thread(
//     n_channels: u32,
//     frames_per_packet: u32,
//     rb_size_packets: u32,
//     ip_addr: *const c_char,
//     port: u16,
// ) -> Option<NonNull<c_void>> {
//     let addr_str = unsafe { CStr::from_ptr(ip_addr) }.to_str().ok()?;

//     let addr = core::net::SocketAddr::new(
//         addr_str.parse().unwrap_or(IpAddr::V4(Ipv4Addr::LOCALHOST)),
//         port,
//     );

//     let socket = std::net::UdpSocket::bind("0.0.0.0:0").ok()?;

//     let (n_channels, (frames_per_chunk, rb_size_packets)) = NonZeroU32::new(n_channels)
//         .zip(NonZeroU32::new(frames_per_packet).zip(NonZeroU32::new(rb_size_packets)))?;

//     let net_chunk_size_spls = frames_per_chunk.checked_mul(n_channels)?;
//     let rb_size_samples = net_chunk_size_spls.checked_mul(rb_size_packets)?;
//     let (tx, mut rx) = rtrb::RingBuffer::<f32>::new(rb_size_samples.get() as usize);

//     let handle = thread::Builder::new()
//         .name(String::from("SYFALA UDP NETWORK"))
//         .spawn(move || {
//             loop {
//                 let Ok(read_chunk) = rx.read_chunk(net_chunk_size_spls.get() as usize) else {
//                     std::thread::park();
//                     continue;
//                 };

//                 let (slice, _) = read_chunk.as_slices();
//                 let slice = as_bytes(slice);

//                 socket.send_to(slice, &addr).unwrap();

//                 read_chunk.commit_all();
//             }
//         }).ok()?;

//     NonNull::new(
//         Box::into_raw(Box::new(NetworkTx::new(tx, handle.thread().clone()))) as *mut c_void,
//     )
// }

// #[unsafe(no_mangle)]
// pub extern "C" fn send_buffer(
//     buffer: *const f32,
//     len: u32,
// ) {}

// let num_ports = args
//     .next()
//     .as_deref()
//     .map(str::parse)
//     .unwrap_or(Ok(DEFAULT_NUM_PORTS))?;

// let addr = core::net::SocketAddr::new(
//     args.next()
//         .as_deref()
//         .map(str::parse)
//         .unwrap_or(Ok(IpAddr::V4(Ipv4Addr::LOCALHOST)))?,
//     PORT,
// );

//

// let (jack_client, _status) =
//     jack::Client::new("CLIENT", jack::ClientOptions::NO_START_SERVER).unwrap();

// let ports = Box::from_iter((0..num_ports.get()).map(|i| {
//     jack_client
//         .register_port(&format!("input_{i}"), jack::AudioIn::default())
//         .unwrap()
// }));

// let mut port_buf_ptrs =
//     Box::from_iter(iter::repeat_with(JackBufPtr::dangling).take(num_ports.get()));

// let writer_async_client = jack::contrib::ClosureProcessHandler::new(move |_client, scope| {
//     let mut remaining_frames = scope.n_frames() as usize;

//     for (port, ptr) in ports.iter().zip(&mut port_buf_ptrs) {
//         *ptr = JackBufPtr::from_slice(port.as_slice(scope));
//     }

//     while let Some(rem) = NonZeroUsize::new(remaining_frames) {
//         let frames = CHUNK_SIZE_FRAMES.min(rem);
//         remaining_frames -= frames.get();

//         let Ok(mut write_chunk) =
//             tx.write_chunk_uninit(num_ports.checked_mul(frames).unwrap().get())
//         else {
//             continue;
//         };

//         let (start, end) = write_chunk.as_mut_slices();

//         let mut rb_chunk_iter = start.iter_mut().chain(end);

//         for _i in 0..frames.get() {
//             // deinterleave chunk contents

//             for buf in &mut port_buf_ptrs {
//                 // SAFETY: buf is valid, and within the actual buffer's bounds
//                 let sample = unsafe { buf.read() };

//                 // SAFETY: this happens at most `frames` times, guaranteeing this stays within
//                 // the buffer's bounds
//                 *buf = unsafe { buf.increment() };

//                 rb_chunk_iter.next().unwrap().write(sample);
//             }
//         }

//         unsafe { write_chunk.commit_all() };

//         if rb_size_samples.get() - tx.slots() >= net_chunk_size_spls.get() {
//             network_thread.unpark();
//         }
//     }

//     jack::Control::Continue
// });

// let _active_client = jack_client.activate_async((), writer_async_client).unwrap();
