#ifndef packet_container_h_
#define packet_container_h_

#define HDR_TYPE_DATA 0x40      // MSB is zero
#define HDR_TYPE_RESP 0x80
#define HDR_TYPE_HALT 0xC0

#define RESP_TYPE_NONE          0x00
#define RESP_TYPE_ID            0x01
#define RESP_TYPE_SAMPLE_COUNT  0x02
#define RESP_TYPE_DROPPED_COUNT 0x03
#define RESP_TYPE_T_MEAN        0x04
#define RESP_TYPE_T_VARIANCE    0x05
#define RESP_TYPE_T_MAX         0x06
#define RESP_TYPE_T_MIN         0x07
#define RESP_TYPE_T_N           0x08


#define MAX_PACKET_LENGTH_BYTES 242   // 2-byte header + 120 words   

class PacketContainer 
{
public:
  
  PacketContainer() : max_packets(0), sample_count(0), pos(1) {}

  uint32_t max_packets;
  uint32_t sample_count;
  uint8_t const* buffer() const { return &buf[0]; }

  template<typename T>
  int8_t write_resp(uint8_t resp_type, T val)
  {
    buf[0] = HDR_TYPE_RESP | resp_type;
    write_to_buf(val, &buf[1]);
    return 5;
  }
  int8_t write_halt() 
  {
    buf[0] = HDR_TYPE_HALT;
    write_to_buf(sample_count, &buf[1]);
    return 5;
  }

  bool append_sample(uint16_t data) 
  {
    write_to_buf(data, &buf[2*pos++]);
    sample_count++;   // accumulate total samples recorded over run
    if (pos < (MAX_PACKET_LENGTH_BYTES >> 1)) {
      return false;
    }
    uint16_t const hdr_word = (HDR_TYPE_DATA << 8) | (0x3FFF & (pos-1));  // -1 for header
    write_to_buf(hdr_word, &buf[0]);
    return true;
  }
  uint8_t byte_count() const { return 2*pos; }
  void reset() { pos = 1; }

private:
  uint16_t pos;   // buffer position in words (16-bit)
  uint8_t buf[MAX_PACKET_LENGTH_BYTES];

  template<typename T>
  void write_to_buf(T v, uint8_t* start) 
  {
    uint8_t const* bvals = reinterpret_cast<uint8_t const*>(&v);
    for (int8_t i = 0; i < sizeof(T); ++i) {
      *start++ = bvals[sizeof(T)-1-i];   // MSB first
    }
  }
};

static PacketContainer packet;

#endif // packet_container_h_
