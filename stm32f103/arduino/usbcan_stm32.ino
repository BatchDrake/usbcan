//
//    usbcan_stm32.ino: turn your BluePill+ STM32F103 into a
//    CAN bus modem.
//
//    Copyright (C) 2025 Gonzalo José Carracedo Carballal
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as
//    published by the Free Software Foundation, either version 3 of the
//    License, or (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful, but
//    WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this program.  If not, see
//    <http://www.gnu.org/licenses/>
//

#include "AA_MCP2515.h"

#define min(a, b) ((a) < (b) ? (a) : (b))

#define USB_FRAME_WAIT_SYNC  0x55415541
#define USB_FRAME_DATA_SYNC  0xf00fbeeb
#define USB_FRAME_ERROR_SYNC 0xffaaffee

#define _JOIN(a, b) a ## b
#define JOIN(a, b) _JOIN(a, b)

#define JOIN3(a, b, c) JOIN(a, JOIN(b, c))
#define JOIN4(a, b, c, d) JOIN(JOIN(a, b), JOIN(c, d))
#define _STRINGFY(x)  #x
#define STRINGFY(x) _STRINGFY(x)

#define CAN_BITRATE  500kbps
#define SPISPEED 8MHz
#define USB_BITRATE 1000000

const uint8_t CAN_PIN_CS  = PA4;
const int8_t  CAN_PIN_INT = PB0;

enum FrameSearchState {
  Searching,
  Reading,
  Complete
};

#pragma pack(push, 1)
struct USBCanFrame {
  uint32_t sync;
  uint32_t seq;
  struct {
    bool     remote:1;
    bool     extended:1;
    uint8_t  dlc:4;
    uint16_t id:11;
  } info;

  uint8_t data[8];

  USBCanFrame(uint32_t);

  void setCanFrame(CANFrame &);
  void sendToUART();
};
#pragma pack(pop)

static_assert(sizeof(USBCanFrame) == 19);

USBCanFrame::USBCanFrame(uint32_t syncCode)
{
  sync          = syncCode;
  seq           = 0;

  info.remote   = false;
  info.extended = false;
  info.dlc      = 0;
  info.id       = 0;
}

void
USBCanFrame::setCanFrame(CANFrame &canFrame)
{
  info.remote   = canFrame.isRemoteFrame();
  info.extended = canFrame.isExtendedFrame();
  info.dlc      = canFrame.getDlc();
  info.id       = canFrame.getId();

  memcpy(data, canFrame.getData(), min(8, info.dlc));
}

void
USBCanFrame::sendToUART()
{
  size_t frameSize = sizeof(USBCanFrame) - 8;

  if (sync == USB_FRAME_DATA_SYNC)
    frameSize += min(8, info.dlc);
  
  Serial.write(reinterpret_cast<uint8_t *>(this), frameSize);
  Serial.flush();

  ++seq;
}

static USBCanFrame g_waitFrame(USB_FRAME_WAIT_SYNC);
static USBCanFrame g_dataFrame(USB_FRAME_DATA_SYNC);
static USBCanFrame g_errorFrame(USB_FRAME_ERROR_SYNC);

#define USB_PARTIAL_FRAME_SIZE (sizeof(USBCanFrame) - 8)

class USBFrameCtx {
  enum FrameSearchState m_state;
  union {
    uint8_t     m_asBytes[0];
    USBCanFrame m_asFrame;
  };

  uint8_t               m_p;
  uint8_t               m_size;
  uint8_t               m_target;
  uint32_t              m_lastDataSeq;
  CANController        *m_bus;

  inline bool          trySync();
  inline size_t        expectedSize() const;

public:
  USBFrameCtx(CANController *);
  
  inline uint32_t     lastSeq() const;
  inline size_t       feed(const uint8_t *data, size_t size);
  inline USBCanFrame *frame();
  inline bool         sendToBus() const;
  inline bool         consumeFromUART();
};

USBFrameCtx::USBFrameCtx(CANController *bus)
{
  m_state       = Searching;
  m_bus         = bus;
  m_p           = 0;
  m_size        = 0;
  m_lastDataSeq = 0;
  m_target      = 0;

  memset(m_asBytes, 0, sizeof(USBCanFrame));
}

uint32_t
USBFrameCtx::lastSeq() const
{
  return m_lastDataSeq;
}

bool
USBFrameCtx::trySync()
{
  unsigned int i;
  uint8_t sync_buf[4];
  
  if (m_size < 4)
    return false;

  for (i = 0; i < 4; ++i) {
    memcpy(sync_buf, m_asBytes + i, 4 - i);
    memcpy(sync_buf + 4 - i, m_asBytes, i);
    uint32_t sync = *reinterpret_cast<const uint32_t *>(sync_buf);

    if (sync  == USB_FRAME_WAIT_SYNC 
      || sync == USB_FRAME_DATA_SYNC
      || sync == USB_FRAME_ERROR_SYNC) {
      m_asFrame.sync = sync;
      m_p             = 0;
      m_size          = 4;
      return true;
    }
  }

  return false;
}

size_t
USBFrameCtx::expectedSize() const
{
  size_t pktSize = USB_PARTIAL_FRAME_SIZE;

  if (m_size >= USB_PARTIAL_FRAME_SIZE
    && m_asFrame.sync == USB_FRAME_DATA_SYNC)
    pktSize += min(m_asFrame.info.dlc, 8);
  
  return pktSize;
}

size_t
USBFrameCtx::feed(const uint8_t *data, size_t size)
{
  size_t i = 0;
  size_t needed = 0;
  size_t read_size;

  while (i < size && m_state != Complete) {
    switch (m_state) {
      case Searching:
        m_asBytes[m_p++] = data[i++];
        if (m_size < 4)
          ++m_size;
        
        if (m_p == 4)
          m_p = 0;
        
        if (trySync())
          m_state = Reading;
        
        break;

      case Reading:
        needed = expectedSize();
        read_size = min(needed - m_size, size - i);

        memcpy(m_asBytes + m_size, data + i, read_size);
        m_size += read_size;
        i += read_size;

        if (m_size == expectedSize())
          m_state = Complete;

        break;

      case Complete:
        break;
    }
  }
  
  return i;
}

USBCanFrame *
USBFrameCtx::frame()
{
  if (m_state == Complete) {
    m_state = Searching;
    m_size  = 0;
    m_p     = 0;
    
    if (m_asFrame.sync == USB_FRAME_DATA_SYNC)
      m_lastDataSeq = m_asFrame.seq;

    return &m_asFrame;
  }

  return nullptr;
}

bool
USBFrameCtx::sendToBus() const
{
  if (m_bus->write(m_asFrame.info.id, m_asFrame.data, min(m_asFrame.info.dlc, 8)) == CANController::IOResult::OK)
    return true;

  return false;
}

bool
USBFrameCtx::consumeFromUART()
{
  uint8_t readBuf[sizeof(USBCanFrame)];
  USBCanFrame *txFrame = nullptr;
  size_t avail = Serial.available();

  if (avail > 0) {
    digitalWrite(PB2, HIGH);

    for (size_t i = 0; i < avail; ++i) {
      uint8_t byte = Serial.read();

      size_t got = feed(&byte, 1);

      if ((txFrame = frame()) != nullptr && txFrame->sync == USB_FRAME_DATA_SYNC)
        sendToBus();
    }

    digitalWrite(PB2, LOW);
  }

  return true;
}

CANConfig     g_config(CANBitrate::JOIN4(Config_, SPISPEED, _, CAN_BITRATE), CAN_PIN_CS, CAN_PIN_INT);
CANController g_bus(g_config);
USBFrameCtx   g_txFrameReader(&g_bus);

void setup() {
  Serial.begin(USB_BITRATE);

  pinMode(PB2, OUTPUT);

  while(g_bus.begin(CANController::Mode::Normal) != CANController::OK) {
    g_waitFrame.sendToUART();
    delay(500);
  }

  while(!g_bus.setOneshot(true)) {
    g_errorFrame.sendToUART();
    delay(500);
  }
}

void loop() {
  CANFrame frame;
  CANController::IOResult ret = g_bus.read(frame);
  
  g_txFrameReader.consumeFromUART();

  switch (ret) {
    case CANController::IOResult::OK:
      g_dataFrame.setCanFrame(frame);
      g_dataFrame.sendToUART();
      break;

    case CANController::IOResult::FAIL:
      g_errorFrame.sendToUART();
      break;

    default:
      g_waitFrame.sendToUART();
      break;
  }
}
