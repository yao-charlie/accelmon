import argparse
import json

def calc_r_sample(D, N, P):
    T_sys = 1.0/48000000
    T_conv = (7.0 + 0.5*N)*2**(P+2)*D*T_sys
    return 1.0/T_conv, T_conv

def clk_settings(r_sample, f_sysclk=48000000, N_min=0, N_max=64):
    
    if r_sample < 5000 or r_sample > 300000:
        raise ValueError('Sample rate must be in the range 5-300ksps ({}ksps)'.format(r_sample/1000.))

    # apply an extra factor of 2 for odd N
    ratio = (2 * f_sysclk) // r_sample

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
        return D, N, P

    abserr = rr
    good_pair = None

    N = [i for i in range(N_min, N_max)]
    N.reverse()
    D = [int(rr / (14.0 + Ni)) for Ni in N]
    max_err = ratio
    errs = [abs(ratio - 4.0*(14 + ni)*di) if di < D_max else max_err for ni, di in zip(N,D)]
    minpos = errs.index(min(errs))

    return D[minpos], N[minpos], P


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compute ADC clock settings')
    parser.add_argument('-n','--min-samples',type=int, default=0,
                        help='Minimum additional ADC clock half-cycles for signal sample (>=0)')
    parser.add_argument('-m','--max-samples',type=int, default=32,
                        help='Maximum additional ADC clock half-cycles for signal sample (0-63)')
    parser.add_argument('-o','--output-file',help='JSON output filename')
    parser.add_argument('sample_rate',type=float, help='Target sample rate (5-300ksps)')
    
    args = parser.parse_args() 

    r_in = args.sample_rate*1000
    D,N,P = clk_settings(r_sample=r_in, 
                        N_min=args.min_samples,
                        N_max=args.max_samples)
    r_out, T_conv = calc_r_sample(D,N,P)

    print("ADC Clock Settings ({}ksps target sample rate)".format(args.sample_rate))
    print("  + Sample rate {}ksps".format(round(r_out/1000.,3)))
    print("  + Conversion time {}us".format(round(T_conv*1e6,3)))
    print("  + GENDIV.DIV={}".format(D))
    print("  + SAMPCTRL.SAMPLEN={}".format(N))

    if args.output_file is not None:
        with open(args.output_file, 'w') as hf:
            json.dump({'adc' : {
                'prescaler' : P, 
                'gclk' : {
                    'divider' : D,
                    'divsel' : 0                
                },
                'samplen' : N}}, 
                hf)

    

