import sys
sys.path.append('../src/accelmon')

import argparse
import board
import time

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Reset the board')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',  
            help='Serial port name. Default is /dev/ttyACM0.')
    args = parser.parse_args()

    mon = board.Controller(port=args.port)
    mon.reset_board()
    print("Resetting",end="")
    for i in range(5):
        print(".",end="")
        time.sleep(1)
    print()

    b_id, accel_type = mon.board_id()
    print(f"Reset sent to board ID {b_id} (0x{b_id:08x}) for {accel_type} on {args.port}")

