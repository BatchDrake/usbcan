/*
  Inject test packets into a CAN interface
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
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <net/if.h>
#include <sys/time.h>

#include <getopt.h>

#define DEFAULT_CAN_ID 0x2AA

static int
open_can(const char *iface)
{
  struct sockaddr_can addr;
  struct ifreq ifr;
  int enable_canfd = 1;
  int canfd = -1;

  /* open socket */
  if ((canfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    fprintf(
      stderr,
      "open_can: cannot open CAN socket: %s\n",
      strerror(errno));
    goto fail;
  }

  addr.can_family = AF_CAN;

  strncpy(ifr.ifr_name, iface, sizeof(ifr.ifr_name));

  if (ioctl(canfd, SIOCGIFINDEX, &ifr) < 0) {
    fprintf(
      stderr,
      "open_can: cannot retrieve index for interface `%s': %s\n",
      iface,
      strerror(errno));
    goto fail;
  }

  addr.can_ifindex = ifr.ifr_ifindex;

  if (setsockopt(
    canfd,
    SOL_CAN_RAW,
    CAN_RAW_FD_FRAMES,
    &enable_canfd,
    sizeof(enable_canfd))) {
    fprintf(
      stderr,
      "open_can: cannot enable RAW FD CAN mode: %s\n",
      strerror(errno));
    goto fail;
  }

  if (bind(canfd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
    fprintf(
      stderr,
      "open_can: cannot bind CAN interface: %s\n",
      strerror(errno));
    goto fail;
  }

  return canfd;

fail:
  if (canfd != -1)
    close(canfd);

  return -1;
}

static bool
write_can(int canfd, uint16_t id, const void *data, size_t size)
{
  if (size > CAN_MTU)
    size = CAN_MTU;
  
  struct can_frame linux_frame;

  bzero(&linux_frame, sizeof(struct can_frame));

  linux_frame.can_id = id;
  linux_frame.len    = size;
  
  memcpy(linux_frame.data, data, linux_frame.len);

  if (write(canfd, &linux_frame, CAN_MTU) != CAN_MTU)
    return false;

  return true;
}

static struct option long_options[] = {
    {"delay", required_argument, 0,  'd' },
    {"id",    required_argument, 0,  'i' },
    {"help",  no_argument,       0,  'h' },
    {NULL,    0,                 0,   0 }
};

static void
help(const char *argv0)
{
  fprintf(stderr, "%s: inject test packets to a CAN interface\n\n", argv0);
  fprintf(stderr, "Usage:\n");
  fprintf(stderr, "\t%s [OPTIONS] CAN-IF\n\n", argv0);
  fprintf(stderr, "Where CAN-IF is a network interface supporting the PF_CAN protocol family\n\n");
  fprintf(stderr, "OPTIONS:\n");
  fprintf(stderr, "  -d, --delay=SECONDS  Delay between consecutive messages.\n");
  fprintf(stderr, "                       Default is 1 (1 message per second)\n");
  fprintf(stderr, "  -i, --id=ID          Value of the CAN_ID field. Default is 0x%x.\n", DEFAULT_CAN_ID);
  fprintf(stderr, "  -h, --help           This help.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "(c) 2025 Gonzalo J. Carracedo <BatchDrake@gmail.com>\n");
  fprintf(stderr, "Copyrighted but free, under the terms of the GPLv3 license\n");
}

static void
wait_until(const struct timeval *when)
{
  struct timeval tv;

  gettimeofday(&tv, NULL);

  while (timercmp(&tv, when, <)) {
    struct timeval diff;
    timersub(when, &tv, &diff);
    
    if (tv.tv_sec > 0) {
      sleep(diff.tv_sec);
    } else if (diff.tv_usec > 0) {
      while (timercmp(&tv, when, <))
        gettimeofday(&tv, NULL);
    }
    gettimeofday(&tv, NULL);
  }
}

static void
do_hello(const char *iface, uint16_t can_id, unsigned int delay_us)
{
  int fd;
  unsigned int i = 0;
  char hellobuf[9];
  struct timeval tv, diff, next;

  if ((fd = open_can(iface)) == -1)
    exit(EXIT_FAILURE);

  diff.tv_sec  = delay_us / 1000000;
  diff.tv_usec = delay_us % 1000000;

  gettimeofday(&next, NULL);
  timeradd(&next, &diff, &next);
  
  for (;;) { 
    snprintf(hellobuf, 9, "HOLO_%03u", i % 1000);

    gettimeofday(&tv, NULL);
    printf("[%ld.%06d] Write \"%s\" to %s\n", tv.tv_sec, tv.tv_usec, hellobuf, iface);
    write_can(fd, can_id, hellobuf, 8);
    ++i;
    wait_until(&next);
    timeradd(&next, &diff, &next);
  }
}

int
main(int argc, char *argv[])
{
  const char *iface = NULL;
  uint16_t can_id = DEFAULT_CAN_ID;
  float delay = 1.0;
  int index = 0;

  int c;

  while ((c = getopt_long(argc, argv, "d:i:h", long_options, &index)) != -1) {
    switch (c) {
      case 'd':
        if (sscanf(optarg, "%f", &delay) < 1 || delay <= 0.0f) {
          fprintf(stderr, "%s: invalid delay time.\n", argv[0]);
          fprintf(stderr, "Run %s -h for help.\n", argv[0]);
          exit(EXIT_FAILURE);
        }
        break;

      case 'i':
        if (sscanf(optarg, "%hu", &can_id) < 1 || can_id > 0x7ff) {
          fprintf(stderr, "%s: invalid CAN id.\n", argv[0]);
          fprintf(stderr, "Run %s -h for help.\n", argv[0]);
          exit(EXIT_FAILURE);
        }
        break;

      case ':':
      case '?':
        fprintf(stderr, "Run %s -h for help.\n", argv[0]);
        exit(EXIT_FAILURE);

      case 'h':
        help(argv[0]);
        exit(EXIT_SUCCESS);
    }
  }

  if (argc - optind != 1) {
    fprintf(stderr, "%s: no CAN interface specified.\n", argv[0]);
    fprintf(stderr, "Run %s -h for help.\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  
  do_hello(argv[optind], can_id, delay * 1e6);

  return 0;
}
