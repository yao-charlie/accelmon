import serial
import struct
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

    class PacketType:
        """Packet types in the upper two bits of the header byte"""
        RSVD = 0b00
        DATA = 0b01
        RESP = 0b10
        HALT = 0b11

    class ResponseType:
        """The type of response in RESP packet stored in bits 3-5 of the header byte"""
        RSVD     = 0b000
        ID       = 0b001
        FCLK     = 0b010
        DIV      = 0b011
        DIV_MODE = 0b100
        ADC_PRE  = 0b101
        SAMPLE_COUNT = 0b110
        ADC_SAMPLEN = 0b111

    class GClkDividerMode:
        """Mode select for the GCLK divider"""
        DIRECT   = 0
        POW2     = 1

    def __init__(self, port=None, baudrate=921600, sinks=[]):
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
   
    def set_sinks(self, sinks):
        self.sinks = sinks

    def reset_com_port(self, port, baudrate=921600):
        if self.comm is not None:
            self.comm.close()
        self.comm = serial.Serial()
        self.comm.port = port
        self.comm.baudrate = baudrate

    def halt(self):
        """Send a halt command"""
        with self.comm as ser:
            ser.write(bytes("H","utf-8"))

    def load_clk_config(self, cfg):
        """Load the board configuration from ClockSettings"""
        self.clock_divider = cfg.D
        self.clock_divider_mode = 0
        self.adc_prescaler = cfg.P
        self.adc_samplen = cfg.N

    def board_id(self):
        """Get the ID of the connected board

        :returns: integer board ID
        """
        return self._ask_resp('B',self.ResponseType.ID)

    def adc_clk_freq(self):
        """Request the calculated ADC clock frequency 

        :returns: the ADC clock frequency in Hz
        """
        return self._ask_resp('F',self.ResponseType.FCLK)

    def adc_sample_rate(self):
        """Request the calculated ADC sample rate

        :returns: the calulated ADC sample rate (ADC clock rate / (6 + 1[Gain] + SAMPLEN))
        """
        L = self._ask_resp('L', self.ResponseType.ADC_SAMPLEN)
        return self.adc_clk_freq() / (7.0 + 0.5*L)

    def sample_count(self):
        """Request the sample count from the last run

        :returns: the packet.sample_count field
        """
        return self._ask_resp('C',self.ResponseType.SAMPLE_COUNT)

    @property
    def clock_divider(self):
        """Request the GCLK divider setting 

        :returns: the GCLK divider (0-255)
        """
        return self._ask_resp('D',self.ResponseType.DIV)

    @clock_divider.setter
    def clock_divider(self, val):
        """Set the GCLK divider (0-255)"""
        byte_val = int(val) & 0xFF 
        with self.comm as ser:
            msg = bytes("CD{}".format(byte_val),"utf-8")
            ser.write(msg)

    @property
    def clock_divider_mode(self):
        """Request the GCLK divider mode (0=direct, 1=pow2)

        :returns: the GCLK divider mode
        """
        val = self._ask_resp('M',self.ResponseType.DIV_MODE)
        return self.GClkDividerMode.DIRECT if val == 0 else self.GClkDividerMode.POW2

    @clock_divider_mode.setter
    def clock_divider_mode(self, val):
        """Set the GCLK divider mode to DIRECT or POW2"""
        ival = int(val)
        if ival != 0 and ival != 1:
            raise ValueError("GCLK divider mode must be 0=DIRECT or 1=POW2")
        with self.comm as ser:
            msg = bytes("CM{}".format(ival),"utf-8")
            ser.write(msg)

    @property
    def adc_prescaler(self):
        """Request the ADC prescaler setting

        :returns: the prescaler setting p such that the ADC clock is scaled by 2^(p+2)
        """
        return self._ask_resp('P',self.ResponseType.ADC_PRE)

    @adc_prescaler.setter
    def adc_prescaler(self, val):
        """Set the ADC prescaler to p such that the ADC clock is scaled by 2^(p+2)"""
        ival = int(val)
        if ival < 0 or ival > 7:
            raise ValueError("ADC prescaler must be in the range 0-7")
        with self.comm as ser:
            msg = bytes("CP{}".format(ival),"utf-8")
            ser.write(msg)

    @property
    def adc_samplen(self):
        """Request the ADC sample length setting

        :returns: the sample length L such that L+1 half-cycles of the ADC clock are used to sample the signal
        """
        return self._ask_resp('L',self.ResponseType.ADC_SAMPLEN)

    @adc_samplen.setter
    def adc_samplen(self, val):
        """Set the ADC sample length to L such that L+1 half-cycles of the ADC clockare used to sample the signal"""
        ival = int(val)
        if ival < 0 or ival > 63:
            raise ValueError("ADC sample length must be in the range 0-63")
        with self.comm as ser:
            msg = bytes("CL{}".format(ival),"utf-8")
            ser.write(msg)


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
                byte_count = 0
                if (hdr[0] & 0xC0) >> 6 == self.PacketType.DATA:
                    byte_count = (1 << (hdr[0] & 0x3F))
                    if byte_count < 2:
                        raise BadHeader("Unexpected byte count for data: {}".format(byte_count))
                    byte_count -= 1
                elif (hdr[0] & 0xC0) >> 6 == self.PacketType.HALT:
                    sample_count = struct.unpack('>I',ser.read(size=4))[0]
                    break
                else:
                    raise BadHeader("Unexpected header type (data): 0x{:x}".format(hdr[0]))
                
                raw_data = [struct.unpack('>H',ser.read(size=2))[0] for i in range(byte_count // 2)]
                sample_count += len(raw_data)

                for sink in self.sinks:
                    sink.write(raw_data)
        
            if self.user_halt:
                ser.write(bytes("H","utf-8"))
                self.user_halt = False

        return sample_count

#    def set_debug_level(self, lvl):
#        """Set the board debug level.
#
#        :param lvl: The debug level, 0=off, 1=on (default 0)
#        :type lvl: int
#        
#        .. warning::
#            Debug levels > 0 will enable string messages sent over the serial port.
#        """
#        ilvl = int(lvl)
#        if ilvl < 0 or ilvl > 2:
#            raise ValueError("Debug level out of range")
#        with self.comm as ser:
#            msg = bytes("CD",'utf-8')+struct.pack('<b',ilvl)
#            ser.write(msg)

    def set_board_id(self, board_id):
        """Set the board identifier

        :param board_id: The integer ID
        :type board_id: int

        .. note::
            Use the :py:meth:`store_board_config` method to save the board ID
            to Flash storage such that it will persist if the board is reset.
        """
        with self.comm as ser:
            msg = bytes("CB","utf-8")+struct.pack('<I',board_id)
            ser.write(msg)

#    def store_board_config(self, confirm):
#        """Store the current configuration (board ID, debug level, polynomial coefficients) to Flash
#
#        :param confirm: boolean flag to indicate confirmation. Flash will not be 
#            written without this flag set.
#        :type confirm: boolean
#
#        .. warning::
#             Excessive writes to Flash may result in loss of storage functionality.
#             Board configuration should only be written during calibration.
#        """
#        if not confirm:
#            raise ValueError("Confirmation must be supplied to write board config to flash")
#        with self.comm as ser:
#            ser.write(bytes("CW","utf-8"))

    def reset_board(self):
        """Trigger a software reset of the board"""
        with self.comm as ser:
            ser.write(bytes("Z","utf-8"))

    def _ask_resp(self,lbl,resp_type):
        """[Internal] Send a packet asking for a response (FCLK, DIV, etc.)

        :param lbl: The response type label ('F'CLK, 'D'IV, DIV_'M'ODE)
        :type lbl: str
        :param resp_type: The response type code from :py:class:`ResponseType`
        :type resp_type: :py:class:`ResponseType` value
        :returns: the value"""
        with self.comm as ser:
            ser.write(bytes("A{}\n".format(lbl),'utf-8'))  
            hdr = ser.read(size=1)
            err_bit = self._validate_resp_hdr(hdr, resp_type)
            buf = ser.read(size=4)
            if err_bit:   
                raise BadHeader("Unexpected error bit")
            else:
                return struct.unpack('>I',buf)[0]

    def _validate_resp_hdr(self, hdr, resp_type):
        """[Internal] Validate the header byte for a response packet

        :param hdr: The header byte
        :type hdr: byte
        :param resp_type: The response type code from :py:class:`ResponseType`
        :type resp_type: :py:class:`ResponseType` value
        :raises BadHeader: If the header contains an unexpected pattern
        :returns: an error flag, True if the error bit is set in the header
        """
        if (hdr[0] & 0xC0) >> 6 != self.PacketType.RESP:
            raise BadHeader("Unexpected header type: 0x{:x}".format(hdr[0]))
        if (hdr[0] & 0x38) >> 3 != resp_type:
            raise BadHeader("Unexpected response type: 0x{:x}".format(hdr[0]))

        return (hdr[0] & 0x01) != 0
    
    def _validate_cfg_param(self, data, lbl, range):
        if lbl not in data:
            return None
        v = int(data[lbl])
        if v >= range[0] and v < range[1]: 
            return v
        else:
            raise ValueError("{} out of range [{},{}] ({})".format(lbl, range[0], range[1], v))


