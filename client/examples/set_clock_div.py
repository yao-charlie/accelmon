import sys
sys.path.append('../src/accelmon')

import argparse
import board
import json

def validate_param(data, lbl, range):
    if lbl not in data:
        return None
    v = int(data[lbl])
    if v >= range[0] and v < range[1]: 
        return v
    else:
        raise ValueError("{} out of range [{},{}] ({})".format(lbl, range[0], range[1], v))

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Set GCLK divider')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',  
            help='Serial port name. Default is /dev/ttyACM0.')
    parser.add_argument('-d', '--divider', type=int, choices=range(256), help='GCLK divider (0-255)')
    parser.add_argument('-m', '--mode', type=int, choices=range(2), help='GCLK divider mode (DIVSEL): 0=direct, 1=pow2')
    parser.add_argument('-s', '--prescaler', type=int, choices=range(8), help='ADC prescaler (0-7): 0=DIV4...7=DIV512')
    parser.add_argument('-l', '--sample-length', type=int, choices=range(64), help='ADC sample length in half-cycles (0-63)')
    parser.add_argument('-f', '--settings-file', help='Load settings from JSON file (overrides command line arguments)')
    args = parser.parse_args()

    if args.settings_file is not None:
        with open(args.settings_file, 'r') as hj:
            jdata = json.load(hj)
            if "adc" in jdata:
                adc = jdata["adc"]
                pv = validate_param(adc, "prescaler", (0,8))
                if pv is not None: args.prescaler = pv
                pv = validate_param(adc, "samplen", (0,64))
                if pv is not None: args.sample_length = pv

                if "gclk" in adc:
                    pv = validate_param(adc["gclk"],"divider",(0,256))
                    if pv is not None: args.divider = pv
                    pv = validate_param(adc["gclk"],"divsel",(0,2))
                    if pv is not None: args.mode = pv
            if "port" in jdata:
                args.port = jdata["port"]

    print("Creating controller on port {}".format(args.port))
    mon = board.Controller(port=args.port)

    id = mon.board_id()
    print("Board ID: {} (0x{:X})".format(id, id))

    if args.mode is not None:
        old_mode = mon.clock_divider_mode
        print("GCLK.DIVSEL = {}, setting to {}".format(old_mode, args.mode))
        mon.clock_divider_mode = args.mode

    if args.divider is not None:
        old_div = mon.clock_divider
        print("GCLK.DIV = {}, setting to {}".format(old_div, args.divider))
        mon.clock_divider = args.divider

    if args.prescaler is not None:
        old_prescaler = mon.adc_prescaler
        print("ADC.PRESCALER = {}, setting to {}".format(old_prescaler, args.prescaler))
        mon.adc_prescaler = args.prescaler

    if args.sample_length is not None:
        old_samplen = mon.adc_samplen
        print("ADC.SAMPLEN = {}, setting to {}".format(old_samplen, args.sample_length))
        mon.adc_samplen = args.sample_length

    print("ADC sample rate: {} ksps".format(mon.adc_sample_rate() / 1000.))



