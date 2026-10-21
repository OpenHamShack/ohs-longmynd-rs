/* The part of libusb-1.0's API that longmynd's ftdi_usb.c uses, declared
 * here so the reference build links against the simulator
 * (src/bin/c_reference.rs) instead of a real libusb. The declarations and
 * the descriptor layout are libusb's own; nothing here reaches hardware. */
#ifndef LMSIM_FAKE_LIBUSB_H
#define LMSIM_FAKE_LIBUSB_H

#include <stdint.h>
#include <sys/types.h>

#define LIBUSB_API_VERSION 0x01000109

typedef struct libusb_context libusb_context;
typedef struct libusb_device libusb_device;
typedef struct libusb_device_handle libusb_device_handle;

struct libusb_device_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
};

enum libusb_request_type {
    LIBUSB_REQUEST_TYPE_STANDARD = (0x00 << 5),
    LIBUSB_REQUEST_TYPE_CLASS = (0x01 << 5),
    LIBUSB_REQUEST_TYPE_VENDOR = (0x02 << 5),
    LIBUSB_REQUEST_TYPE_RESERVED = (0x03 << 5)
};
enum libusb_request_recipient {
    LIBUSB_RECIPIENT_DEVICE = 0x00,
    LIBUSB_RECIPIENT_INTERFACE = 0x01,
    LIBUSB_RECIPIENT_ENDPOINT = 0x02,
    LIBUSB_RECIPIENT_OTHER = 0x03
};
enum libusb_endpoint_direction {
    LIBUSB_ENDPOINT_OUT = 0x00,
    LIBUSB_ENDPOINT_IN = 0x80
};
enum libusb_option {
    LIBUSB_OPTION_LOG_LEVEL = 0,
    LIBUSB_OPTION_USE_USBDK = 1
};
enum libusb_log_level {
    LIBUSB_LOG_LEVEL_NONE = 0,
    LIBUSB_LOG_LEVEL_ERROR = 1,
    LIBUSB_LOG_LEVEL_WARNING = 2,
    LIBUSB_LOG_LEVEL_INFO = 3,
    LIBUSB_LOG_LEVEL_DEBUG = 4
};

int libusb_init(libusb_context **ctx);
void libusb_exit(libusb_context *ctx);
int libusb_set_option(libusb_context *ctx, enum libusb_option option, ...);
libusb_device_handle *libusb_open_device_with_vid_pid(libusb_context *ctx, uint16_t vendor_id, uint16_t product_id);
ssize_t libusb_get_device_list(libusb_context *ctx, libusb_device ***list);
void libusb_free_device_list(libusb_device **list, int unref_devices);
uint8_t libusb_get_bus_number(libusb_device *dev);
uint8_t libusb_get_device_address(libusb_device *dev);
int libusb_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc);
int libusb_open(libusb_device *dev, libusb_device_handle **dev_handle);
void libusb_close(libusb_device_handle *dev_handle);
libusb_device *libusb_get_device(libusb_device_handle *dev_handle);
int libusb_kernel_driver_active(libusb_device_handle *dev_handle, int interface_number);
int libusb_detach_kernel_driver(libusb_device_handle *dev_handle, int interface_number);
int libusb_claim_interface(libusb_device_handle *dev_handle, int interface_number);
int libusb_bulk_transfer(libusb_device_handle *dev_handle, unsigned char endpoint, unsigned char *data,
                         int length, int *actual_length, unsigned int timeout);
int libusb_control_transfer(libusb_device_handle *dev_handle, uint8_t request_type, uint8_t bRequest,
                            uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength,
                            unsigned int timeout);
const char *libusb_error_name(int errcode);

#endif
