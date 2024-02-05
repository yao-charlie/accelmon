import serial
import struct
import datetime
import threading

class BadHeader(Exception):
    """An error in the packet header"""
    pass

class BadPacket(Exception):
    """A packet has a formatting error"""
    pass

class ClockSettings:
    """Configuration for the on-chip ADC clock and sampling to define
    the sample rate"""

    F_SYS_CLK = 48000000

    def __init__(self, r_sample, N_min=0, N_max=64):
        """Initialize the clock settings for a target sample rate.
        Note: the actual sample rate may differ due to the fixed
        precision of the clock settings."""
        self.D = None
        self.N = None
        self.P = None
        
        if r_sample < 5000 or r_sample > 300000:
            raise ValueError('Sample rate must be in the range 5-300ksps ({}ksps)'.format(r_sample/1000.))

        if N_min < 0 or N_max > 64 or N_max < N_min:
            raise ValueError('Sample clocks N must define an increasing range in [0, 64) (N_min={}, N_max={})'.format(N_min, N_max))

        # apply an extra factor of 2 for odd N
        ratio = (2 * self.F_SYS_CLK) // r_sample

        # Since min(r_sample) = 5000, max(ratio) = 2*48000/5 = 19200
        # 19200 / (4*D_max) = 18.823.. --> force N_max = 5 if required
        P = 0       # ADC Prescaler 2^(P+2)
        rr = ratio // 4    # ADC prescaler

        D_max = 256 # max value for the 8-bit DIV field 0xFF
        if N_min == N_max:
            N = N_min
            D = int(rr / (14.0 + N))
            while D >= D_max:
                rr //= 2
                P += 1
                D = int(rr / (14.0 + N))
            self.D = D
            self.N = N
            self.P = P
        else: 
            N = [i for i in range(N_min, N_max)]
            N.reverse()
            D = [int(rr / (14.0 + Ni)) for Ni in N]
            max_err = ratio
            errs = [abs(ratio - 4.0*(14 + ni)*di) if di < D_max else max_err for ni, di in zip(N,D)]
            minpos = errs.index(min(errs))
            self.D = D[minpos]
            self.N = N[minpos]
            self.P = P
    
    def T_conversion(self):
        """Compute the conversion period T"""
        T_sys = 1.0/self.F_SYS_CLK
        return (7.0 + 0.5*self.N)*2**(self.P+2)*self.D*T_sys

    def f_conversion(self):
        """Compute the conversion frequency (sample rate)"""
        return 1.0/self.T_conversion()

class Controller:
    """A board controller for the Accelerometer board using serial communication"""

    AccelerometerTypeId = {"ADXL1005":1, "KX134":2}

    class PacketType:
        """Packet types in the upper two bits of the header byte"""
        RSVD = 0b00
        DATA = 0b01
        RESP = 0b10
        HALT = 0b11

    class ResponseType:
        """The type of response in RESP packet stored in bits 3-5 of the header byte"""
        # general
        NONE          = 0x00
        ID            = 0x01
        SAMPLE_COUNT  = 0x02
        DROPPED_COUNT = 0X03
        T_MEAN        = 0x04
        T_VARIANCE    = 0x05
        T_MAX         = 0x06
        T_MIN         = 0x07
        T_N           = 0x08
        # ADXL1005 specific
        ADXL_FCLK        = 0x13
        ADXL_DIV         = 0x14
        ADXL_DIV_MODE    = 0x15
        ADXL_ADC_PRE     = 0x16
        ADXL_ADC_SAMPLEN = 0x17
        # KX134 specific
        KX134_ODR   = 0x18
        KX134_GSEL  = 0x19 

    class GClkDividerMode:
        """Mode select for the GCLK divider"""
        DIRECT   = 0
        POW2     = 1

    def __init__(self, port=None, baudrate=921600, sinks=[], accel_type="ADXL1005"):
        """Construct a Controller with a specified port

        :param port: The serial port
        :type port: str
        :param baudrate: The baudrate for the serial connection 
            (default 921600 specified in firmware)
        :type baudrate: int
        """
        self.comm = None
        self.reset_com_port(port=port, baudrate=baudrate)
        self.user_halt = False
        self.sinks = sinks
        if accel_type not in self.AccelerometerTypeId:
            raise RuntimeError(f"Unsupported accelerometer type '{accel_type}'") 
        self.accelerometer_type = accel_type
   
    def set_sinks(self, sinks):
        self.sinks = sinks

    def reset_com_port(self, port, baudrate=921600):
        if self.comm is not None:
            self.comm.close()
        self.comm = serial.Serial()
        self.comm.port = port
        self.comm.baudrate = baudrate

    #
    # General communication with board
    #
    def halt(self):
        """Send a halt command"""
        with self.comm as ser:
            ser.write(bytes("H","utf-8"))

    def board_id(self):
        """Get the ID of the connected board

        :returns: integer board ID
        """
        id_field = self._ask_resp('B',self.ResponseType.ID)
        if id_field is None:
            raise RuntimeError("Failed to get a valid board ID response")
        id_mask = 0x00FFFFFF
        controller_id = id_mask & id_field
        accel_type_id = (~id_mask & id_field) >> 24
        for k,v in self.AccelerometerTypeId.items():
            if v == accel_type_id:
                if k != self.accelerometer_type: self.accelerometer_type = k
                return (controller_id, k)
        return (controller_id, "Unknown")
    
    def set_board_id(self, board_id):
        """Set the board identifier

        :param board_id: The integer ID, only the lower 24 bits
        :type board_id: int

        .. note::
            Setting the board ID will write this ID to Flash storage such
            that it will persist if the board is reset. Flash has a limited
            number of write cycles, so this should be done infrequently
        """
        with self.comm as ser:
            msg = bytes("CB","utf-8")+struct.pack('<I',board_id)
            ser.write(msg)

    def reset_board(self):
        """Trigger a software reset of the board"""
        with self.comm as ser:
            ser.write(bytes("Z","utf-8"))

    def sample_count(self):
        """Request the sample count from the last run

        :returns: the packet.sample_count field
        """
        return self._ask_resp('C',self.ResponseType.SAMPLE_COUNT)
    
    def dropped_count(self):
        """Request the dropped count from the last run

        :returns: the dropped_count result
        """
        return self._ask_resp('X',self.ResponseType.DROPPED_COUNT)
    
    def T_mean(self):
        """Request the mean sample period (us)

        :returns: the mean sample period (us)
        """
        return self._ask_resp('U',self.ResponseType.T_MEAN,data_format='>f')
    
    def T_variance(self):
        """Request the sample period variance (us)

        :returns: the sample period variance (us)
        """
        return self._ask_resp('V',self.ResponseType.T_VARIANCE,data_format='>f')

    def T_max(self):
        return self._ask_resp('R',self.ResponseType.T_MAX)
    def T_min(self):
        return self._ask_resp('S',self.ResponseType.T_MIN)
    def T_N(self):
        return self._ask_resp('N',self.ResponseType.T_N)

    # 
    # ADXL1005
    #
    def load_clk_config(self, cfg):
        """Load the board configuration from ClockSettings"""
        self.clock_divider = cfg.D
        self.clock_divider_mode = 0
        self.adc_prescaler = cfg.P
        self.adc_samplen = cfg.N

    def adc_clk_freq(self):
        """Request the calculated ADC clock frequency 

        :returns: the ADC clock frequency in Hz
        """
        return self._ask_resp('F',self.ResponseType.ADXL_FCLK)

    def adc_sample_rate(self):
        """Request the calculated ADC sample rate

        :returns: the calulated ADC sample rate (ADC clock rate / (6 + 1[Gain] + SAMPLEN))
        """
        L = self._ask_resp('L', self.ResponseType.ADXL_ADC_SAMPLEN)
        if L is None: return L
        return self.adc_clk_freq() / (7.0 + 0.5*L)

    @property
    def clock_divider(self):
        """Request the GCLK divider setting 

        :returns: the GCLK divider (0-255)
        """
        return self._ask_resp('D',self.ResponseType.ADXL_DIV)

    @clock_divider.setter
    def clock_divider(self, val):
        """Set the GCLK divider (0-255)"""
        byte_val = int(val) & 0xFF 
        with self.comm as ser:
            msg = bytes("CD","utf-8")+struct.pack("<I",byte_val)
            ser.write(msg)

    @property
    def clock_divider_mode(self):
        """Request the GCLK divider mode (0=direct, 1=pow2)

        :returns: the GCLK divider mode
        """
        val = self._ask_resp('M',self.ResponseType.ADXL_DIV_MODE)
        if val is None: return val
        return self.GClkDividerMode.DIRECT if val == 0 else self.GClkDividerMode.POW2

    @clock_divider_mode.setter
    def clock_divider_mode(self, val):
        """Set the GCLK divider mode to DIRECT or POW2"""
        ival = int(val)
        if ival != 0 and ival != 1:
            raise ValueError("GCLK divider mode must be 0=DIRECT or 1=POW2")
        with self.comm as ser:
            msg = bytes("CM","utf-8")+struct.pack("<I",ival)
            ser.write(msg)

    @property
    def adc_prescaler(self):
        """Request the ADC prescaler setting

        :returns: the prescaler setting p such that the ADC clock is scaled by 2^(p+2)
        """
        return self._ask_resp('P',self.ResponseType.ADXL_ADC_PRE)

    @adc_prescaler.setter
    def adc_prescaler(self, val):
        """Set the ADC prescaler to p such that the ADC clock is scaled by 2^(p+2)"""
        ival = int(val)
        if ival < 0 or ival > 7:
            raise ValueError("ADC prescaler must be in the range 0-7")
        with self.comm as ser:
            msg = bytes("CP","utf-8")+struct.pack("<I",ival)
            ser.write(msg)

    @property
    def adc_samplen(self):
        """Request the ADC sample length setting

        :returns: the sample length L such that L+1 half-cycles of the ADC clock are used to sample the signal
        """
        return self._ask_resp('L',self.ResponseType.ADXL_ADC_SAMPLEN)

    @adc_samplen.setter
    def adc_samplen(self, val):
        """Set the ADC sample length to L such that L+1 half-cycles of the ADC clockare used to sample the signal"""
        ival = int(val)
        if ival < 0 or ival > 63:
            raise ValueError("ADC sample length must be in the range 0-63")
        with self.comm as ser:
            msg = bytes("CL","utf-8")+struct.pack("<I",ival)
            ser.write(msg)
    
    # 
    # KX134
    #
    @property
    def accel_output_data_rate(self):
        """Request the output data rate (KX134)

        :returns: the output data rate register value"""
        return self._ask_resp('F',self.ResponseType.KX134_ODR)

    @accel_output_data_rate.setter
    def accel_output_data_rate(self, val):
        """Set the KX134 output data rate using the 4-bit field code"""
        byte_val = int(val) & 0x0F 
        with self.comm as ser:
            msg = bytes("CF","utf-8")+struct.pack("<I",byte_val)
            ser.write(msg)
    
    @property
    def accel_g_range(self):
        """Request the g range code (KX134)

        :returns: the GSEL range register value (0-3)"""
        return self._ask_resp('G',self.ResponseType.KX134_GSEL)

    @accel_g_range.setter
    def accel_g_range(self, val):
        """Set the KX134 g range using the 2-bit field code"""
        byte_val = int(val) & 0x03 
        with self.comm as ser:
            msg = bytes("CG","utf-8")+struct.pack("<I",byte_val)
            ser.write(msg)

    #
    # Collect samples
    #
    def stop_collection(self):
        """Request a stop of the collection of samples."""
        self.user_halt = True

    def collect_samples(self, max_samples=0):
        """Start the free-running collection of ADC samples

        :param max_samples: The maximum number of samples to collect. Set to zero
            for free-running collection. The sample rate for the board is approximately 5Hz.
        :type max_samples: int
        
        Collected samples are written to the sink(s).
        """
        sample_count = 0
        writer_thread = None
        write_data = []
        with self.comm as ser:
            msg = bytes("R","utf-8")+ struct.pack('<I',max_samples)
            ser.write(msg)
            while not (self.user_halt 
                    or (max_samples > 0 and sample_count >= max_samples)):

                hdr = ser.read(size=1)
                word_count = 0
                if (hdr[0] & 0xC0) >> 6 == self.PacketType.DATA:
                    hdr_lb = ser.read(size=1)   # lower byte
                    word_count = struct.unpack('>H',hdr + hdr_lb)[0] & 0x3FFF
                    if word_count < 1:
                        raise BadHeader("Unexpected count for data: {}".format(word_count))
                elif (hdr[0] & 0xC0) >> 6 == self.PacketType.HALT:
                    sample_count = struct.unpack('>I',ser.read(size=4))[0]
                    break
                else:
                    raise BadHeader("Unexpected header type (data): 0x{:x}".format(hdr[0]))
                
                raw_data = [struct.unpack('>H',ser.read(size=2))[0] for i in range(word_count)]
                #Not sure what format would be preferable but this is at least consistemt with the filename
                raw_data.append(datetime.now().strftime("%Y-%m-%d_%Hh-%Mm-%Ss"))
                sample_count += len(raw_data)

                for sink in self.sinks:
                    sink.write(raw_data)
        
            if self.user_halt:
                ser.write(bytes("H","utf-8"))
                self.user_halt = False

        return sample_count

    def _ask_resp(self,lbl,resp_type,data_format='>I'):
        """[Internal] Send a packet asking for a response (FCLK, DIV, etc.)

        :param lbl: The response type label ('F'CLK, 'D'IV, DIV_'M'ODE)
        :type lbl: str
        :param resp_type: The response type code from :py:class:`ResponseType`
        :type resp_type: :py:class:`ResponseType` value
        :returns: the value"""
        with self.comm as ser:
            ser.write(bytes("A{}\n".format(lbl),'utf-8'))  
            hdr = ser.read(size=1)
            ok = self._validate_resp_hdr(hdr, resp_type)
            buf = ser.read(size=4)
            if ok is None:
                return None
            elif ok:
                return struct.unpack(data_format,buf)[0]
            raise BadHeader("Unexpected error")
                
    def _validate_resp_hdr(self, hdr, resp_type):
        """[Internal] Validate the header byte for a response packet

        :param hdr: The header byte
        :type hdr: byte
        :param resp_type: The response type code from :py:class:`ResponseType`
        :type resp_type: :py:class:`ResponseType` value
        :raises BadHeader: If the header contains an unexpected pattern
        :returns: True on success
        """
        if (hdr[0] & 0xC0) >> 6 != self.PacketType.RESP:
            raise BadHeader("Unexpected header type: 0x{:x}".format(hdr[0]))
        received_resp_type = (hdr[0] & 0x3F)
        if received_resp_type == self.ResponseType.NONE:
            return None
        if received_resp_type != resp_type:
            raise BadHeader("Unexpected response type: 0x{:x}".format(hdr[0]))
        return True
    
    def _validate_cfg_param(self, data, lbl, range):
        if lbl not in data:
            return None
        v = int(data[lbl])
        if v >= range[0] and v < range[1]: 
            return v
        else:
            raise ValueError("{} out of range [{},{}] ({})".format(lbl, range[0], range[1], v))


