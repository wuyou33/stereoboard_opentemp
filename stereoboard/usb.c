

#include "usb.h"



#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


void usb_init(void)
{
  char message[64] = " --- Stereo Camera System --- \r\n";
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);
  VCP_send_str(message);
}

void usb_send(char *buff, int len)
{
  int j = 0;
  int size = len;
  for (j = 0; j < size; j += CDC_DATA_MAX_PACKET_SIZE) {
    int s = size - j;
    if (s > CDC_DATA_MAX_PACKET_SIZE) {
      s = CDC_DATA_MAX_PACKET_SIZE;
    }
    VCP_DataTx(buff + j, s);
    Delay(0x1FFFFF);
  }
}
