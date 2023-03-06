import sys
sys.path.append('../src/ptprobe')

import argparse
import board
import random
import time

if __name__ == "__main__":
    
    ai_coeffs = [-97.35308, 920.6867, -14.86687]

    parser = argparse.ArgumentParser(description='Set default board configuration')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',  
            help='Serial port name. Default is /dev/ttyACM0.')
    parser.add_argument('-i', '--id', type=int, default=-1)
    args = parser.parse_args()

    board_id = random.randint(16384,2684355456) if args.id <= 0 else args.id
    
    pt = board.Controller(port=args.port)

    pt.set_board_id(board_id)
    for ich in range(4):
        pt.set_P_poly_coeffs(ich, ai_coeffs) #[1+3*ich,2+3*ich,3+3*ich])
    pt.set_debug_level(0)

    print("Board configuration updated")
    print("  + board ID: {}".format(pt.board_id()))
    print("  + debug level: 0 (off)")
    print("  + pressure transducer coefficients")
    for ich in range(4):
        status = pt.sensor_status_P(ich)
        print("    + ch{}: {}".format(ich, status['ai']))
    choice = input("Store configuration (y/N)? ")

    if choice == 'y' or choice == 'Y':
        print("Writing configuration to flash")
        pt.store_board_config(True)
        time.sleep(1)
        pt.reset_board()
        time.sleep(6)
        print("Done, board reset")
    else:
        print("Done.")






