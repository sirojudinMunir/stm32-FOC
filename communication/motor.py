from motor_protocol import MotorProtocol
import numpy as np

motor = MotorProtocol(port="COM4", baudrate=115200, timeout=2)

def get_motor_param():
    pole_pairs = motor.get_pole_pairs()
    kv = motor.get_kv()
    rs = motor.get_rs()
    ld = motor.get_ld()
    lq = motor.get_lq()
    flux_linkage = motor.get_flux_linkage()
    print(f'pole pairs:{pole_pairs}, kv:{kv}, rs:{rs}, ld:{ld}, lq:{lq}, flux_linkage:{flux_linkage}')

def set_foc_bandwidth(bw=100):
    rs = motor.get_rs() / 2
    ld = motor.get_ld() / 2
    lq = motor.get_lq() / 2
    omega = 2 * np.pi * bw
    id_kp = ld * omega
    id_ki = rs * omega
    iq_kp = lq * omega
    iq_ki = rs * omega
    print(f'id: kp={id_kp} ki={id_ki}')
    print(f'iq: kp={iq_kp} ki={iq_ki}')
    

