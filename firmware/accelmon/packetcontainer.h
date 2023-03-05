#ifndef packet_container_h_
#define packet_container_h_


#define HDR_TYPE_DATA (0x1 << 6)
#define HDR_TYPE_SYNC (0x2 << 6)
#define PACKET_LENGTH 64

struct PacketContainer 
{
  PacketContainer() : pos(0), align_left(true), count(0) {}
  
  uint8_t buf[PACKET_LENGTH];
  uint8_t pos;
  bool align_left;
  uint32_t count;

  void reset_for_write() {
    buf[0] = 0;        // clear count
    pos = 1;           // start at byte 1 (byte 0 is header),
    align_left = true; // first 12 bits align left
  }

  void append(uint16_t val)
  {
    // adc_val is 16 bits with upper 4 bits = 0 (12 bit ADC result)
    uint8_t inc_pos;
    if (align_left) {   // byte 0 is header, byte 1 shift up
      val <<= 4;    // left align
      inc_pos = 1;
    } else {            // right align and store existing 4 bits at top
      val |= (buf[pos] & 0xF0) << 8;
      inc_pos = 2;
    }
    *(uint16_t*)(&buf[pos]) = val;
    pos += inc_pos;
    align_left = !align_left;
    buf[0] += 1;
  }
  
};

static PacketContainer packet;

#endif // packet_container_h_
