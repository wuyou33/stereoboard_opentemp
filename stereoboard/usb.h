

#ifndef MY_USB_HEADER
#define MY_USB_HEADER

// Initialize the USB Virtual COM Port
void usb_init(void);

// Send a packet
void usb_send(char *buff, int len);

#endif
