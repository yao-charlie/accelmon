import sys
sys.path.append('../src/accelmon')

import argparse
import board

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Read ADXL-1005 accelerometer over serial')
    parser.add_argument('port', type=str, help='Serial port name, e.g. /dev/ttyACM0')
    args = parser.parse_args()

    mon = board.Controller(port=args.port)

    b_id, accel_type = mon.board_id()
    print(f"Board ID: {b_id}, accelerometer type: {accel_type}")
    print("Last sample count: {}".format(mon.sample_count()))

    if accel_type == "KX134":
        odr_table = [0.781, 0.1563]
        odr_table.extend([3.125*(1 << x) for x in range(14)])
        odr = mon.accel_output_data_rate
        print("Output data rate: ",end="")
        if odr < len(odr_table):
            print(f"{odr_table[odr]} Hz")
        else:
            print("undefined")

        gsel = mon.accel_g_range
        max_g = 8*(1 << gsel)
        print(f"Acceleration range: +/-{max_g}g")
            


