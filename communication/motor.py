from motor_protocol import MotorProtocol

motor = MotorProtocol(port="COM4", baudrate=115200, timeout=2)

def get_motor_param():
    pole_pairs = motor.get_pole_pairs()
    kv = motor.get_kv()
    rs = motor.get_rs()
    ld = motor.get_ld()
    lq = motor.get_lq()
    flux_linkage = motor.get_flux_linkage()
    print(f'pole pairs:{pole_pairs}, kv:{kv}, rs:{rs}, ld:{ld}, lq:{lq}, flux_linkage:{flux_linkage}')


