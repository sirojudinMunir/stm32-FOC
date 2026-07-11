import struct
import serial
import time
import math

class MotorProtocol:

    def __init__(self, port="COM4", baudrate=115200, timeout=2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connect()

    # ================================================================
    # Serial Connection

    def connect(self):
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout
        )

    def reconnect(self):
        print("Reconnecting...")

        while True:
            try:
                if self.ser is not None:
                    if self.ser.is_open:
                        self.ser.close()

                time.sleep(1)

                self.ser = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout
                )

                print("Reconnected!")
                break

            except Exception:
                print(".", end="", flush=True)
                time.sleep(1)

    # ================================================================
    # Communication

    def send_data(self, data):
        try:
            if self.ser.is_open:
                self.ser.write(data)
                response = self.ser.read(1)
                if response:
                    error_code = struct.unpack('<b', response)[0]
                    return error_code
                return None
        except serial.SerialException as e:
            self.reconnect()
            return None

    def recv_float_data(self, cmd, n=1):
        try:
            if self.ser.is_open:
                self.ser.write(cmd)
                response = self.ser.read(4 * n)
                if len(response) != 4 * n:
                    raise RuntimeError("Data float tidak lengkap")
                data = struct.unpack("<" + "f" * n, response)
                if n == 1:
                    return data[0]
                return data
        except serial.SerialException as e:
            self.reconnect()
            return None
        
    def recv_uint8_data(self, cmd, n=1):
        try:
            if self.ser.is_open:
                self.ser.write(cmd)
                response = self.ser.read(n)
                if len(response) != n:
                    raise RuntimeError("Data uint8 tidak lengkap")
                data = struct.unpack("<" + "B" * n, response)
                if n == 1:
                    return data[0]
                return data
        except serial.SerialException as e:
            self.reconnect()
            return None
    
    # ================================================================

    def set_default_config(self):
        data = bytes([9])
        return self.send_data(data)
    
    def save_config(self):
        data = bytes([10])
        return self.send_data(data)

    def set_foc_mode(self, mode):
        data = bytes([11, mode])
        return self.send_data(data)

    def get_foc_mode(self):
        data = bytes([12])
        mode = self.recv_uint8_data(data, 1)
        return mode
    
    def set_foc_motor_mode(self, mode):
        data = bytes([13, mode])
        return self.send_data(data)

    def get_foc_motor_mode(self):
        data = bytes([14])
        mode = self.recv_uint8_data(data, 1)
        return mode
    
    # ================================================================
    def set_pole_pairs(self, value):
        data = bytes([15, value])
        return self.send_data(data)

    def get_pole_pairs(self):
        data = bytes([16])
        return self.recv_uint8_data(data, 1)

    def set_kv(self, value):
        data = bytes([17]) + struct.pack('<f', value)
        return self.send_data(data)

    def get_kv(self):
        data = bytes([18])
        return self.recv_float_data(data, 1)

    def set_rs(self, value):
        data = bytes([19]) + struct.pack('<f', value)
        return self.send_data(data)

    def get_rs(self):
        data = bytes([20])
        return self.recv_float_data(data, 1)

    def set_ld(self, value):
        data = bytes([21]) + struct.pack('<f', value)
        return self.send_data(data)

    def get_ld(self):
        data = bytes([22])
        return self.recv_float_data(data, 1)

    def set_lq(self, value):
        data = bytes([23]) + struct.pack('<f', value)
        return self.send_data(data)

    def get_lq(self):
        data = bytes([24])
        return self.recv_float_data(data, 1)

    def set_flux_linkage(self, value):
        data = bytes([25]) + struct.pack('<f', value)
        return self.send_data(data)

    def get_flux_linkage(self):
        data = bytes([26])
        return self.recv_float_data(data, 1)

    def set_pid_id(self, kp, ki, deadband):
        data = bytes([27]) + struct.pack('<fff', kp, ki, deadband)
        return self.send_data(data)

    def get_pid_id(self):
        data = bytes([28])
        return self.recv_float_data(data, 3)

    def set_pid_iq(self, kp, ki, deadband):
        data = bytes([29]) + struct.pack('<fff', kp, ki, deadband)
        return self.send_data(data)

    def get_pid_iq(self):
        data = bytes([30])
        return self.recv_float_data(data, 3)

    def set_pid_speed(self, kp, ki, out_max, deadband):
        data = bytes([31]) + struct.pack('<ffff', kp, ki, out_max, deadband)
        return self.send_data(data)

    def get_pid_speed(self):
        data = bytes([32])
        return self.recv_float_data(data, 4)

    def set_pid_position(self, kp, ki, kd, out_max, deadband, d_fc):
        data = bytes([33]) + struct.pack('<ffffff', kp, ki, kd, out_max, deadband, d_fc)
        return self.send_data(data)

    def get_pid_position(self):
        data = bytes([34])
        return self.recv_float_data(data, 6)

    def set_field_weakening(self, kp, ki, out_min):
        data = bytes([35]) + struct.pack('<fff', kp, ki, out_min)
        return self.send_data(data)

    def get_field_weakening(self):
        data = bytes([36])
        return self.recv_float_data(data, 3)

    def set_field_weakening_enable(self, enable):
        data = bytes([37, enable])
        return self.send_data(data)

    def get_field_weakening_enable(self):
        data = bytes([38])
        return self.recv_uint8_data(data, 1)

    def set_mtpa_enable(self, enable):
        data = bytes([39, enable])
        return self.send_data(data)

    def get_mtpa_enable(self):
        data = bytes([40])
        return self.recv_uint8_data(data, 1)

    def set_foc_current_set_point(self, value):
        data = bytes([41]) + struct.pack('<f', value)
        return self.send_data(data)


    def start_measure_motor_Rs(self):
        data = bytes([47])
        return self.send_data(data)
    
    def start_measure_motor_Ld(self):
        data = bytes([48])
        return self.send_data(data)
    
    def start_measure_motor_Lq(self):
        data = bytes([49])
        return self.send_data(data)
    

    # def set_ia_offset(self, value):
    #     data = bytes([41]) + struct.pack('<f', value)
    #     return self.send_data(data)

    # def get_ia_offset(self):
    #     data = bytes([42])
    #     return self.recv_float_data(data, 1)

    # def set_ib_offset(self, value):
    #     data = bytes([43]) + struct.pack('<f', value)
    #     return self.send_data(data)

    # def get_ib_offset(self):
    #     data = bytes([44])
    #     return self.recv_float_data(data, 1)

    # def set_ic_offset(self, value):
    #     data = bytes([45]) + struct.pack('<f', value)
    #     return self.send_data(data)

    # def get_ic_offset(self):
    #     data = bytes([46])
    #     return self.recv_float_data(data, 1)

    # def set_gear_ratio(self, value):
    #     data = bytes([47]) + struct.pack('<f', value)
    #     return self.send_data(data)

    # def get_gear_ratio(self):
    #     data = bytes([48])
    #     return self.recv_float_data(data, 1)



