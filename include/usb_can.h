/*
  Linux CAN Bridge
  Copyright (C) 2025 Gonzalo José Carracedo Carballal
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, version 3.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program.  If not, see
  <http://www.gnu.org/licenses/>

*/

#ifndef _USB_CAN_H
#define _USB_CAN_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>
#include <unistd.h>

#define USB_CAN_SERIAL_SPEED     B1000000
#define USB_CAN_BUS_MAX_SPEED      125000

#define USB_CAN_FRAME_WAIT_SYNC  0x55415541
#define USB_CAN_FRAME_DATA_SYNC  0xf00fbeeb
#define USB_CAN_FRAME_ERROR_SYNC 0xffaaffee

#define USB_CAN_PARTIAL_FRAME_SIZE (sizeof(usb_can_frame_t) - 8)

enum usb_can_frame_search_state {
  Searching,
  Reading,
  Complete
};

#pragma pack(push, 1)
struct usb_can_frame {
  uint32_t sync;
  uint32_t seq;

  struct {
    bool     remote:1;
    bool     extended:1;
    uint8_t  dlc:4;
    uint16_t id:11;
  } info;

  uint8_t data[8];
};
#pragma pack(pop)

typedef struct usb_can_frame usb_can_frame_t;

typedef void (*usb_can_statistics_cb_t) (
    uint32_t can2usb,
    uint64_t can2usb_bytes,
    uint32_t usb2can,
    uint64_t usb2can_bytes,
    uint32_t lost,
    void    *userdata);

struct usb_can_ctx {
  int usbcan_fd;
  int vcan_fd;

  uint8_t           p;
  uint8_t           size;
  uint8_t           target;
  uint32_t          last_rx_data_seq;
  uint32_t          last_tx_data_seq;
  
  struct timeval    last_statistics;
  struct timeval    next_tx;

  enum usb_can_frame_search_state state;
  union {
    uint8_t         as_bytes[0];
    usb_can_frame_t as_frame;
  };

  uint32_t can2usb;
  uint32_t can2usb_bytes;

  uint32_t usb2can;
  uint32_t usb2can_bytes;
  uint32_t usb_lost;
  bool     have_first_seq;

  void *userdata;
  usb_can_statistics_cb_t on_statistics;
};

typedef struct usb_can_ctx usb_can_ctx_t;

usb_can_ctx_t *usb_can_ctx_open(const char *path, const char *vcan);
void usb_can_ctx_close(usb_can_ctx_t *);

uint32_t usb_can_ctx_last_seq(const usb_can_ctx_t *);
size_t   usb_can_ctx_feed(usb_can_ctx_t *, const uint8_t *, size_t);
bool     usb_can_ctx_consume_usb(usb_can_ctx_t *);
bool     usb_can_ctx_consume_vcan(usb_can_ctx_t *);
bool     usb_can_ctx_bridge_work(usb_can_ctx_t *);
void     usb_can_ctx_set_userdata(usb_can_ctx_t *, void *);
void     usb_can_ctx_set_statistics_cb(usb_can_ctx_t *, usb_can_statistics_cb_t);
const    usb_can_frame_t *usb_can_ctx_get_frame(usb_can_ctx_t *);

#endif /* _USB_CAN_H */
