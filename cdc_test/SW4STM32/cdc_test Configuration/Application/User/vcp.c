/*
 * vcp.c
 *
 *  Created on: 2015/09/20
 */

#include "vcp.h"
#include "usb_device.h"

/**
 * thanks http://www.mcu.by/tag/usb-vcp/
 */
int VCP_read(void *pBuffer, int size)
{
    if (!RxBuffer.ReadDone)
        return 0;

    int remaining = RxBuffer.Size - RxBuffer.Position;
    int todo = MIN(remaining, size);
    if (todo <= 0)
        return 0;

    memcpy(pBuffer, RxBuffer.Buffer + RxBuffer.Position, todo);
    RxBuffer.Position += todo;
    if (RxBuffer.Position >= RxBuffer.Size)
    {
        RxBuffer.ReadDone = 0;
        if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
              return 0;
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }

    return todo;
}

int VCP_write(const void *pBuffer, int size)
{
    if (size > CDC_DATA_HS_OUT_PACKET_SIZE)
    {
        int offset;
        for (offset = 0; offset < size; offset++)
        {
            int todo = MIN(CDC_DATA_HS_OUT_PACKET_SIZE,
                           size - offset);
            int done = VCP_write(((char *)pBuffer) + offset, todo);
            if (done != todo)
                return offset + done;
        }

        return size;
    }

    USBD_CDC_HandleTypeDef *pCDC =
            (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    while(pCDC->TxState) { } //Wait for previous transfer

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
      return 0;
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)pBuffer, size);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
        return 0;

    while(pCDC->TxState) { } //Wait until transfer is done
    return size;
}

