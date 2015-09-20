/*
 * vcp.h
 *
 *  Created on: 2015/09/20
 */

#ifndef APPLICATION_USER_VCP_H_
#define APPLICATION_USER_VCP_H_

#include "usbd_cdc.h"

extern struct rxBufStruct
{
  uint8_t Buffer[CDC_DATA_HS_OUT_PACKET_SIZE];
  int Position, Size;
  char ReadDone;
} RxBuffer;

extern struct rxBufStruct RxBuffer;

int VCP_read(void *pBuffer, int size);
int VCP_write(const void *pBuffer, int size);


#endif /* APPLICATION_USER_VCP_H_ */
