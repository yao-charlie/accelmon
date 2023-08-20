import sys
sys.path.append('../src/accelmon')

import argparse
import board

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Configure KX134 accelerometer over serial')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',  
            help='Serial port name. Default is /dev/ttyACM0.')
    parser.add_argument('-r', '--output-data-rate', type=int, choices=range(16), help='Output data rate code: 50*2^(r-6) Hz')
    parser.add_argument('-g', '--g-range', type=int, choices=range(4), help='Acceleration range: 8*2^g')
    parser.add_argument('-b', '--board-id', type=int, help='Board ID (only the lower 24 bits)')
    args = parser.parse_args()

    print("Creating controller on port {}".format(args.port))
    mon = board.Controller(port=args.port)

    b_id, accel_type = mon.board_id()
    print(f"Board ID: {b_id} (0x{b_id:08x}), accelerometer type: {accel_type}")

    if accel_type == "KX134":
        if args.output_data_rate is not None:
            old_odr = mon.accel_output_data_rate
            odr_table = [0.781, 0.1563]
            odr_table.extend([3.125*(1 << x) for x in range(14)])
            print(f"ODCNTL.OSA is {old_odr} ({odr_table[old_odr]} Hz), setting to {args.output_data_rate} ({odr_table[args.output_data_rate]} Hz)")
            mon.accel_output_data_rate = args.output_data_rate

        if args.g_range is not None:
            g_range_table = [8*(1 << x) for x in range(4)]
            old_gsel = mon.accel_g_range
            print(f"CNTL1.GSEL is {old_gsel} (+/-{g_range_table[old_gsel]}g), setting to {args.g_range} (+/-{g_range_table[args.g_range]}g)")
            mon.accel_g_range = args.g_range


        if args.board_id is not None:
            print(f"Updating board ID to 0x{(0x00FFFFFF & args.board_id):08x}")
            mon.set_board_id(args.board_id)
        
    else:
        print("!! Board is not configured for KX134 accelerometer")
