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

    print("GCLK frequency: {}Hz".format(mon.fclk()))



