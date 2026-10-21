//! FTDI's D2XX library, simulated: longmynd's D2XX backend
//! (longmynd::d2xx) driven against the same simulated MiniTiouner as the
//! libusb path, so a scenario run through D2XX can be compared with the C
//! (tests/d2xx.rs). The device model is the Sim's; this is only D2XX's
//! call surface over it (sim.rs, "D2XX").

use longmynd::d2xx::*;

use crate::sim;

pub struct SimD2xx;

impl D2xxApi for SimD2xx {
    fn create_device_info_list(&self) -> Result<u32, FtStatus> {
        Ok(sim::with(|s| s.d2xx_list()).len() as u32)
    }
    fn get_device_info_detail(&self, index: u32) -> Result<FtDeviceInfo, FtStatus> {
        // the list is regenerated per call, as D2XX's is per
        // FT_CreateDeviceInfoList; without logging each entry
        sim::with(|s| {
            let t = std::mem::take(&mut s.trace);
            let v = s.d2xx_list();
            s.trace = t;
            v.get(index as usize).cloned().ok_or(FT_DEVICE_NOT_FOUND)
        })
    }
    fn open_ex(&self, by: &OpenBy) -> Result<FtHandle, FtStatus> {
        match by {
            OpenBy::Location(loc) => sim::with(|s| s.d2xx_open(*loc)),
            _ => Err(FT_DEVICE_NOT_FOUND),
        }
    }
    fn close(&self, h: FtHandle) -> FtStatus {
        sim::with(|s| s.d2xx_setup(h, "FT_Close"))
    }
    fn reset_device(&self, h: FtHandle) -> FtStatus {
        sim::with(|s| s.d2xx_control(h, "FT_ResetDevice", false))
    }
    fn purge(&self, h: FtHandle, mask: u32) -> FtStatus {
        sim::with(|s| s.d2xx_control(h, &format!("FT_Purge {}", mask), mask & FT_PURGE_RX != 0))
    }
    fn set_latency_timer(&self, h: FtHandle, ms: u8) -> FtStatus {
        sim::with(|s| s.d2xx_control(h, &format!("FT_SetLatencyTimer {}", ms), false))
    }
    fn set_bit_mode(&self, h: FtHandle, mask: u8, mode: u8) -> FtStatus {
        sim::with(|s| s.d2xx_control(h, &format!("FT_SetBitMode {:02x} {:02x}", mask, mode), false))
    }
    fn set_usb_parameters(&self, h: FtHandle, in_size: u32, out_size: u32) -> FtStatus {
        sim::with(|s| s.d2xx_setup(h, &format!("FT_SetUSBParameters {} {}", in_size, out_size)))
    }
    fn set_timeouts(&self, h: FtHandle, read_ms: u32, write_ms: u32) -> FtStatus {
        sim::with(|s| s.d2xx_setup(h, &format!("FT_SetTimeouts {} {}", read_ms, write_ms)))
    }
    fn get_queue_status(&self, h: FtHandle) -> Result<u32, FtStatus> {
        Ok(sim::with(|s| s.d2xx_queue(h)))
    }
    fn read(&self, h: FtHandle, buf: &mut [u8]) -> (FtStatus, usize) {
        if sim::with(|s| s.d2xx_starts_ts_read(h)) {
            crate::rsdrive::run_ts_hook();
        }
        sim::with(|s| s.d2xx_read(h, buf))
    }
    fn write(&self, h: FtHandle, buf: &[u8]) -> (FtStatus, usize) {
        sim::with(|s| s.d2xx_write(h, buf))
    }
}
