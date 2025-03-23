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

#include <usb_can.h>
#include <stdio.h>
#include <stdlib.h>

static double
to_human(uint64_t quantity, const char **human_units)
{
  static const char *units[] = {"bytes", "KiB", "MiB", "GiB", "TiB", "PiB", "EiB"};
  unsigned int p = 0;

  while (p < 6 && (quantity >> 10 * p) > 1024)
    ++p;

  *human_units = units[p];
  return (double) quantity / (double) (1ull << 10 * p);
}

static void
usbcanbr_statistics(
    uint32_t can2usb,
    uint64_t can2usb_bytes,
    uint32_t usb2can,
    uint64_t usb2can_bytes,
    uint32_t lost,
    void    *userdata)
{
  const char *can_units;
  double      can_bytes = to_human(can2usb_bytes, &can_units);

  const char *usb_units;
  double      usb_bytes = to_human(usb2can_bytes, &usb_units);

  fprintf(
    stderr,
    "\r\e[2K[usbcanbr] RX: %d frames (%5.1f %s), TX: %d frames (%5.1f %s), LOST: %d",
    usb2can,
    usb_bytes,
    usb_units,
    can2usb,
    can_bytes,
    can_units,
    lost);
}

int
main(int argc, char **argv)
{
  bool ok;

  if (argc != 3) {
    fprintf(stderr, "%s: invalid number of arguments\n", argv[0]);
    fprintf(stderr, "Usage\n");
    fprintf(stderr, "    %s SERIAL-DEVICE VCAN-IF\n\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  usb_can_ctx_t *ctx = usb_can_ctx_open(argv[1], argv[2]);
  if (ctx == NULL)
    exit(EXIT_FAILURE);

  usb_can_ctx_set_statistics_cb(ctx, usbcanbr_statistics);
  ok = usb_can_ctx_bridge_work(ctx);
  usb_can_ctx_close(ctx);

  exit(ok ? EXIT_SUCCESS : EXIT_FAILURE);
}
