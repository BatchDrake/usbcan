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

#define _DEFAULT_SOURCE

#include <stdio.h>
#include <usb_can.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <poll.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <net/if.h>

void
usb_can_frame_debug(const usb_can_frame_t *self)
{
  unsigned int i, size;

  fprintf(stderr, 
    "[%9d] [%c%c] [%04x] ",
    self->seq,
    self->info.remote   ? 'R' : '-',
    self->info.extended ? 'E' : '-',
    self->info.id);

  printf("%d,%d,%d,%d,", self->seq, self->info.remote, self->info.extended, self->info.id);

  size = self->info.dlc;
  if (size > 8)
    size = 8;
  
  for (i = 0; i < size; ++i) {
    if (i > 0)
      fputc(' ', stderr);
    fprintf(stderr, "%02x", self->data[i]);
    printf("%02x", self->data[i]);
  }

  fputc(10, stderr);
  putchar(10);
}

static inline size_t
sizemin(size_t a, size_t b)
{
  return a < b ? a : b;
}

static inline size_t
sizemax(size_t a, size_t b)
{
  return a > b ? a : b;
}

static void
usb_can_ctx_init(usb_can_ctx_t *ctx)
{
  memset(ctx, 0, sizeof(usb_can_ctx_t));

  ctx->state     = Searching;
  ctx->usbcan_fd = -1;
  ctx->vcan_fd   = -1;
}

uint32_t
usb_can_ctx_last_seq(const usb_can_ctx_t *self)
{
  return self->last_rx_data_seq;
}

static bool
usb_can_ctx_try_sync(usb_can_ctx_t *self)
{
  unsigned int i;
  uint8_t sync_buf[4];
  
  if (self->size < 4)
    return false;

  for (i = 0; i < 4; ++i) {
    memcpy(sync_buf, self->as_bytes + i, 4 - i);
    memcpy(sync_buf + 4 - i, self->as_bytes, i);
    uint32_t sync = *(const uint32_t *) sync_buf;

    if (sync  == USB_CAN_FRAME_WAIT_SYNC 
      || sync == USB_CAN_FRAME_DATA_SYNC
      || sync == USB_CAN_FRAME_ERROR_SYNC) {
      self->as_frame.sync = sync;
      self->p             = 0;
      self->size          = 4;
      return true;
    }
  }

  return false;
}

static inline size_t
usb_can_ctx_expected_size(const usb_can_ctx_t *self)
{
  size_t pkt_size = USB_CAN_PARTIAL_FRAME_SIZE;

  if (self->size >= USB_CAN_PARTIAL_FRAME_SIZE
    && self->as_frame.sync == USB_CAN_FRAME_DATA_SYNC)
    pkt_size += sizemin(self->as_frame.info.dlc, 8);
  
  return pkt_size;
}

size_t
usb_can_ctx_feed(
  usb_can_ctx_t *self,
  const uint8_t *data,
  size_t size)
{
  size_t i = 0;
  size_t needed = 0;
  size_t read_size;

  while (i < size && self->state != Complete) {
    switch (self->state) {
      case Searching:
        self->as_bytes[self->p++] = data[i++];
        if (self->size < 4)
          ++self->size;
        
        if (self->p == 4)
          self->p = 0;
        
        if (usb_can_ctx_try_sync(self))
          self->state = Reading;

        break;

      case Reading:
        needed = usb_can_ctx_expected_size(self);

        read_size = sizemin(needed - self->size, size - i);

        memcpy(self->as_bytes + self->size, data + i, read_size);
        self->size += read_size;
        i += read_size;

        if (self->size == usb_can_ctx_expected_size(self))
          self->state = Complete;

        break;

      case Complete:
        break;
    }
  }
  
  return i;
}

const usb_can_frame_t *
usb_can_ctx_get_frame(usb_can_ctx_t *self)
{
  if (self->state == Complete) {
    self->state = Searching;
    self->size  = 0;
    self->p     = 0;
    
    if (self->as_frame.sync == USB_CAN_FRAME_DATA_SYNC)
      self->last_rx_data_seq = self->as_frame.seq;

    return &self->as_frame;
  }

  return NULL;
}

static inline bool
usb_can_ctx_deliver_to_vcan(usb_can_ctx_t *self)
{
  struct can_frame linux_frame;
  const usb_can_frame_t *frame = &self->as_frame;

  bzero(&linux_frame, sizeof(struct can_frame));

  linux_frame.can_id = frame->info.id;
  linux_frame.len    = sizemin(frame->info.dlc, CAN_MAX_DLEN);
  
  memcpy(linux_frame.data, frame->data, linux_frame.len);

  if (write(self->vcan_fd, &linux_frame, CAN_MTU) != CAN_MTU)
    return false;

  ++self->usb2can;
  self->usb2can_bytes += CAN_MTU;

  return true;
}

static void
usb_can_ctx_show_statistics(usb_can_ctx_t *self)
{
  if (self->on_statistics != NULL) {
    struct timeval tv, diff;

    gettimeofday(&tv, NULL);
    timersub(&tv, &self->last_statistics, &diff);
    
    if (true || diff.tv_sec > 0 || diff.tv_usec > 100000) {
      (self->on_statistics) (
          self->can2usb,
          self->can2usb_bytes,
          self->usb2can,
          self->usb2can_bytes,
          self->usb_lost,
          self->userdata
        );
      self->last_statistics = tv;
    }
  }
}

bool
usb_can_ctx_consume_usb(usb_can_ctx_t *self)
{
  const usb_can_frame_t *frame;
  char buffer[USB_CAN_PARTIAL_FRAME_SIZE];
  ssize_t got = 0;

  if ((got = read(self->usbcan_fd, buffer, sizeof(buffer))) > 0) {
    ssize_t size = got, p = 0;

    while (p < size) {
      got = usb_can_ctx_feed(self, buffer + p, size - p);

      uint32_t next_seq = usb_can_ctx_last_seq(self) + 1;

      if ((frame = usb_can_ctx_get_frame(self)) != NULL) {
        switch (frame->sync) {
          case USB_CAN_FRAME_DATA_SYNC:
            if (self->have_first_seq && frame->seq != next_seq)
              self->usb_lost += frame->seq - next_seq;
          
            if (!usb_can_ctx_deliver_to_vcan(self))
              fprintf(
                stderr,
                "[%9d] cannot deliver to CAN layer: %s\n",
                next_seq,
                strerror(errno));

            self->have_first_seq = true;
            break;
            
          case USB_CAN_FRAME_WAIT_SYNC:
            break;

          case USB_CAN_FRAME_ERROR_SYNC:
            fprintf(stderr, "[%9d] IN-DEVICE TRANSCEIVER ERROR\n", next_seq);
            break;
        }

        usb_can_ctx_show_statistics(self);
      }

      p += got;
    }
  }

  return got >= 0 || errno == EAGAIN;
}

static void
usb_can_ctx_update_next_tx(usb_can_ctx_t *self, size_t payload_len)
{
  struct timeval tv;
  size_t frame_bits = 21 + payload_len * 8 + 16 + 2 + 7 + 3;
  double frame_t    = frame_bits / (double) USB_CAN_BUS_MAX_SPEED;

  struct timeval diff = {
    .tv_sec = (unsigned) floor(frame_t),
    .tv_usec = (unsigned) ceil(fmod(frame_t / 1e-6, 1e6))
  };

  gettimeofday(&tv, NULL);
  timeradd(&tv, &diff, &self->next_tx);
}

static void
usb_can_ctx_wait_next_tx(const usb_can_ctx_t *self)
{
  struct timeval tv;

  gettimeofday(&tv, NULL);

  while (timercmp(&tv, &self->next_tx, <)) {
    struct timeval diff;
    timersub(&self->next_tx, &tv, &diff);
    
    if (tv.tv_sec > 0) {
      sleep(diff.tv_sec);
    } else if (diff.tv_usec > 0) {
      while (timercmp(&tv, &self->next_tx, <))
        gettimeofday(&tv, NULL);
    }
    gettimeofday(&tv, NULL);
  }
}

bool
usb_can_ctx_consume_vcan(usb_can_ctx_t *self)
{
  ssize_t got;
  struct canfd_frame linux_frame;
  usb_can_frame_t    usb_frame;

  if ((got = read(self->vcan_fd, &linux_frame, CANFD_MTU)) >= (ssize_t) CAN_MTU) {
    if (linux_frame.len > CAN_MAX_DLEN) {
      fprintf(
        stderr,
        "[%9d] vcan: discard extended frame (unsupported)\n",
        self->last_tx_data_seq);
    } else {
      const uint8_t *as_bytes = (const uint8_t *) &usb_frame;
      unsigned int i = 0, frame_len;
      
      bzero(&usb_frame, sizeof(usb_can_frame_t));

      usb_frame.sync     = USB_CAN_FRAME_DATA_SYNC;
      usb_frame.seq      = self->last_tx_data_seq;
      usb_frame.info.dlc = linux_frame.len;
      usb_frame.info.id  = linux_frame.can_id;

      memcpy(usb_frame.data, linux_frame.data, linux_frame.len);
      
      frame_len = USB_CAN_PARTIAL_FRAME_SIZE + usb_frame.info.dlc;

      usb_can_ctx_wait_next_tx(self);
      for (i = 0; i < frame_len; ++i)
        if (write(self->usbcan_fd, as_bytes + i, 1) < 1)
          fprintf(stderr, "[%9d] USB WRITE ERROR: %s\n", self->last_tx_data_seq, strerror(errno));
      
      tcflush(self->usbcan_fd, TCIOFLUSH);
      usb_can_ctx_update_next_tx(self, linux_frame.len);
      usb_can_ctx_show_statistics(self);

      ++self->can2usb;
      self->can2usb_bytes += frame_len;
    }

    ++self->last_tx_data_seq;
  }

  return got >= 0 || errno == EAGAIN;
}

static bool
usb_can_ctx_configure_line(usb_can_ctx_t *self)
{
  struct termios options;
  bool ok = false;

  /* Configure line speed and raw mode */
  if (tcgetattr(self->usbcan_fd, &options) == -1)
    goto done;

  if (cfsetispeed(&options, USB_CAN_SERIAL_SPEED) == -1)
    goto done;

  if (cfsetospeed(&options, USB_CAN_SERIAL_SPEED) == -1)
    goto done;

  cfmakeraw(&options);

  options.c_lflag &= ~ICANON;

  if (tcsetattr(self->usbcan_fd, TCSANOW, &options) == -1)
    goto done;

  ok = true;

done:
  return ok;
}

static bool
usb_can_ctx_configure_vcan(usb_can_ctx_t *self, const char *vcan)
{
  struct sockaddr_can addr;
  struct ifreq ifr;
  int enable_canfd = 1;

  /* open socket */
  if ((self->vcan_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    fprintf(
      stderr,
      "usb_can_ctx_configure_vcan: cannot open CAN socket: %s\n",
      strerror(errno));
    return false;
  }

  addr.can_family = AF_CAN;

  strcpy(ifr.ifr_name, vcan);
  if (ioctl(self->vcan_fd, SIOCGIFINDEX, &ifr) < 0) {
    fprintf(
      stderr,
      "usb_can_ctx_configure_vcan: cannot retrieve index for interface `%s': %s\n",
      vcan,
      strerror(errno));

    return false;
  }

  addr.can_ifindex = ifr.ifr_ifindex;

  if (setsockopt(
    self->vcan_fd,
    SOL_CAN_RAW,
    CAN_RAW_FD_FRAMES,
    &enable_canfd,
    sizeof(enable_canfd))) {
    fprintf(
      stderr,
      "usb_can_ctx_configure_vcan: cannot enable RAW FD CAN mode: %s\n",
      strerror(errno));
    
    return false;
  }

  if (bind(self->vcan_fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    fprintf(
      stderr,
      "usb_can_ctx_configure_vcan: cannot bind CAN interface: %s\n",
      strerror(errno));
    return false;
  }

  return true;
}

usb_can_ctx_t *
usb_can_ctx_open(const char *path, const char *vcan)
{
  usb_can_ctx_t *new = NULL;

  if ((new = calloc(1, sizeof(usb_can_ctx_t))) == NULL) {
    fprintf(stderr, "usb_can_ctx_open: memory exhausted\n");
    goto fail;
  }

  usb_can_ctx_init(new);

  if ((new->usbcan_fd = open(path, O_RDWR)) == -1) {
    fprintf(
      stderr,
      "usb_can_ctx_open: cannot open CAN dongle device `%s': %s\n",
      path,
      strerror(errno));
    goto fail;
  }

  if (!usb_can_ctx_configure_line(new))
    goto fail;

  if (!usb_can_ctx_configure_vcan(new, vcan))
    goto fail;
  
  return new;

fail:
  if (new != NULL)
    usb_can_ctx_close(new);

  return NULL;
}

void
usb_can_ctx_close(usb_can_ctx_t *self)
{
  if (self->usbcan_fd != -1)
    close(self->usbcan_fd);

  if (self->vcan_fd != -1)
    close(self->vcan_fd);

  free(self);
}

void
usb_can_ctx_set_userdata(usb_can_ctx_t *self, void *userdata)
{
  self->userdata = userdata;
}

void
usb_can_ctx_set_statistics_cb(usb_can_ctx_t *self, usb_can_statistics_cb_t cb)
{
  self->on_statistics = cb;
}

bool
usb_can_ctx_bridge_work(usb_can_ctx_t *self)
{
  struct pollfd fds[2];

  fds[0].fd     = self->usbcan_fd;
  fds[0].events = POLLIN;

  fds[1].fd     = self->vcan_fd;
  fds[1].events = POLLIN;

  while (poll(fds, 2, -1) != -1) {
    if (fds[0].revents & POLLHUP) {
      fprintf(stderr, "usb_can_ctx_bridge_work: USB dongle disconnect\n");
      return true;
    }

    if (fds[0].revents & POLLERR) {
      fprintf(stderr, "usb_can_ctx_bridge_work: USB dongle I/O error\n");
      return false;
    }

    if (fds[1].revents & POLLHUP) {
      fprintf(stderr, "usb_can_ctx_bridge_work: VCAN interface vanished\n");
      return true;
    }

    if (fds[1].revents & POLLERR) {
      fprintf(stderr, "usb_can_ctx_bridge_work: VCAN I/O error\n");
      return false;
    }

    if ((fds[0].revents & POLLIN) && !usb_can_ctx_consume_usb(self)) {
      fprintf(stderr, "usb_can_ctx_bridge_work: USB -> VCAN error\n");
      return false;
    }

    if ((fds[1].revents & POLLIN) && !usb_can_ctx_consume_vcan(self)) {
      fprintf(stderr, "usb_can_ctx_bridge_work: VCAN -> USB error\n");
      return false;
    }

    fds[0].revents = fds[1].revents = 0;
  }

  return true;
}
