import sys
sys.path.append('../src/accelmon')

import argparse
import board

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Set GCLK divider')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',  
            help='Serial port name. Default is /dev/ttyACM0.')
    parser.add_argument('-d', '--divider', type=int, choices=range(256))
    parser.add_argument('-m', '--mode', type=int, choices=range(2))
    parser.add_argument('-s', '--prescaler', type=int, choices=range(8))
    args = parser.parse_args()

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
    
    print("ADC sample rate: {} ksps".format(mon.adc_sample_rate() / 1000.))



