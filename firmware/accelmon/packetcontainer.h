#ifndef packet_container_h_
#define packet_container_h_


#define HDR_TYPE_DATA 0x1
#define HDR_TYPE_RESP 0x2
#define HDR_TYPE_HALT 0x3

#define RESP_TYPE_RSVD      0x0
#define RESP_TYPE_ID        0x1
#define RESP_TYPE_FCLK      0x2
#define RESP_TYPE_DIV       0x3
#define RESP_TYPE_DIV_MODE  0x4
#define RESP_TYPE_ADC_PRE   0x5
#define RESP_TYPE_SAMPLE_COUNT  0x6
#define RESP_TYPE_ADC_SAMPLEN   0x7

#define MAX_PACKET_LENGTH 64

class PacketContainer 
{
public:
  
 PacketContainer() : max_packets(0), sample_count(0), pos(1) {}

  uint32_t max_packets;
  uint32_t sample_count;
  uint8_t const* buffer() const { return &buf[0]; }

  int8_t write_resp(uint8_t resp_type, uint32_t val)
  {
    buf[0] = (HDR_TYPE_RESP << 6) | (resp_type << 3);
    write_to_buf(val, &buf[1]);
    return 5;
  }
  int8_t write_halt() 
  {
    buf[0] = HDR_TYPE_HALT << 6;
    write_to_buf(sample_count, &buf[1]);
    return 5;
  }
  bool append_sample(uint16_t data) 
  {
    write_to_buf(data, &buf[pos]);
    sample_count++;
    pos += 2;   // sizeof(uint16_t)
    if (pos >= (MAX_PACKET_LENGTH-2)) { 
      buf[0] = (HDR_TYPE_DATA << 6) | pos;
      return true;
    }
    return false;
  }

  uint8_t byte_count() const { return pos; }
  void reset() { pos = 1; }

private:
  uint8_t pos;
  uint8_t buf[MAX_PACKET_LENGTH];

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
