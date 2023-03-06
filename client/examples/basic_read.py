import sys
sys.path.append('../src/accelmon')

import argparse
import board

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Read ADXL-1005 accelerometer over serial')
    parser.add_argument('port', type=str, help='Serial port name, e.g. /dev/ttyACM0')
    args = parser.parse_args()

    pt = board.Controller(port=args.port)

    print("Board ID: {}".format(pt.board_id()))
    print("Last sample count: {}".format(pt.sample_count()))


