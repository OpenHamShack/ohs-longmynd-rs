//! longmynd's error codes (errors.h), kept as the same numbers because they
//! are visible: main() returns its error as the process exit status, and
//! callers such as the OpenHamShack daemon read it.
#![allow(dead_code)]

pub const ERROR_NONE: u8 = 0;
pub const ERROR_ARGS: u8 = 1;
pub const ERROR_ARGS_INPUT: u8 = 2;
pub const ERROR_STATE: u8 = 3;
pub const ERROR_DEMOD_STATE: u8 = 4;
pub const ERROR_FTDI_READ_HIGH: u8 = 5;
pub const ERROR_READ_DEMOD: u8 = 6;
pub const ERROR_WRITE_DEMOD: u8 = 7;
pub const ERROR_READ_OTHER: u8 = 8;
pub const ERROR_WRITE_OTHER: u8 = 9;
pub const ERROR_FTDI_USB_INIT_LIBUSB: u8 = 101;
pub const ERROR_FTDI_USB_VID_PID: u8 = 102;
pub const ERROR_FTDI_USB_CLAIM: u8 = 103;
pub const ERROR_FTDI_USB_I2C_WRITE: u8 = 104;
pub const ERROR_FTDI_USB_I2C_READ: u8 = 105;
pub const ERROR_FTDI_USB_CMD: u8 = 106;
pub const ERROR_FTDI_USB_INIT: u8 = 107;
pub const ERROR_FTDI_SYNC_AA: u8 = 10;
pub const ERROR_FTDI_SYNC_AB: u8 = 11;
pub const ERROR_NIM_INIT: u8 = 12;
pub const ERROR_LNA_ID: u8 = 13;
pub const ERROR_TUNER_ID: u8 = 14;
pub const ERROR_TUNER_LOCK_TIMEOUT: u8 = 15;
pub const ERROR_TUNER_CAL_TIMEOUT: u8 = 16;
pub const ERROR_TUNER_CAL_LOWPASS_TIMEOUT: u8 = 17;
pub const ERROR_I2C_NO_ACK: u8 = 18;
pub const ERROR_FTDI_I2C_WRITE_LEN: u8 = 19;
pub const ERROR_FTDI_I2C_READ_LEN: u8 = 20;
pub const ERROR_MPSSE: u8 = 21;
pub const ERROR_DEMOD_INIT: u8 = 22;
pub const ERROR_BAD_DEMOD_HUNT_STATE: u8 = 23;
pub const ERROR_TS_FIFO_WRITE: u8 = 24;
pub const ERROR_OPEN_TS_FIFO: u8 = 25;
pub const ERROR_TS_FIFO_CREATE: u8 = 26;
pub const ERROR_STATUS_FIFO_CREATE: u8 = 27;
pub const ERROR_OPEN_STATUS_FIFO: u8 = 28;
pub const ERROR_TS_FIFO_CLOSE: u8 = 29;
pub const ERROR_STATUS_FIFO_CLOSE: u8 = 30;
pub const ERROR_USB_TS_READ: u8 = 31;
pub const ERROR_LNA_AGC_TIMEOUT: u8 = 32;
pub const ERROR_DEMOD_PLL_TIMEOUT: u8 = 33;
pub const ERROR_FTDI_USB_DEVICE_LIST: u8 = 34;
pub const ERROR_FTDI_USB_BAD_DEVICE_NUM: u8 = 35;
pub const ERROR_FTDI_USB_DEVICE_NUM_OPEN: u8 = 36;
pub const ERROR_UDP_WRITE: u8 = 37;
pub const ERROR_UDP_SOCKET_OPEN: u8 = 38;
pub const ERROR_UDP_CLOSE: u8 = 39;
pub const ERROR_VITERBI_PUNCTURE_RATE: u8 = 40;
pub const ERROR_TS_BUFFER_MALLOC: u8 = 41;
pub const ERROR_THREAD_ERROR: u8 = 42;
pub const ERROR_SIGNAL_TERMINATE: u8 = 43;
