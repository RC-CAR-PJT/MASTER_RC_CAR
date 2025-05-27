import smbus2
import time
import serial
import struct

class MPU6050:
    def __init__(self, address=0x68):
        self.MPU6050_ADDR = address
        self.PWR_MGMT_1 = 0x6B
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        self.bus = None
        self._init_sensor()

    def _init_sensor(self):
        for attempt in range(3):
            try:
                self.bus = smbus2.SMBus(1)
                self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0)
                print(f"[MPU6050] Initialized with address 0x{self.MPU6050_ADDR:02x}")
                return
            except Exception as e:
                print(f"[ERROR] MPU6050 초기화 실패 (시도 {attempt+1}): {e}")
                time.sleep(1)
        print("[ERROR] MPU6050 초기화 3회 실패, 프로그램 종료 필요")

    def read_word(self, reg):
        if self.bus is None:
            print("[ERROR] I2C bus not initialized")
            return 0
        try:
            high = self.bus.read_byte_data(self.MPU6050_ADDR, reg)
            low = self.bus.read_byte_data(self.MPU6050_ADDR, reg + 1)
            val = (high << 8) + low
            if val >= 0x8000:
                val = -((65535 - val) + 1)
            return val
        except Exception as e:
            print(f"[ERROR] MPU6050 읽기 실패: {e}")
            return 0

    def read_accel(self):
        accel_x = self.read_word(self.ACCEL_XOUT_H)
        accel_y = self.read_word(self.ACCEL_XOUT_H + 2)
        accel_z = self.read_word(self.ACCEL_XOUT_H + 4)
        # 감도 변환: 기본 16384 LSB/g (MPU6050 default range ±2g)
        return accel_x / 16384.0, accel_y / 16384.0, accel_z / 16384.0

    def read_gyro(self):
        gyro_x = self.read_word(self.GYRO_XOUT_H)
        gyro_y = self.read_word(self.GYRO_XOUT_H + 2)
        gyro_z = self.read_word(self.GYRO_XOUT_H + 4)
        # 감도 변환: 기본 131 LSB/°/s
        return gyro_x / 131.0, gyro_y / 131.0, gyro_z / 131.0

def send_to_esp32(data, ser):
    """
    ESP32의 SensorPkt_t 구조체와 동일한 포맷으로 바이너리 전송
    struct.pack 포맷: 6 floats + 1 int = 'ffffffi'
    """
    try:
        packet = struct.pack('ffffffi',
                             data['accX'], data['accY'], data['accZ'],
                             data['gyroX'], data['gyroY'], data['gyroZ'],
                             data['dist_cm'])
        ser.write(packet)
        # 필요시 ser.flush() 호출
    except Exception as e:
        print(f"[ERROR] UART 전송 실패: {e}")

if __name__ == "__main__":
    try:
        ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.1)
        print("[SYSTEM] UART 포트 열림")
    except Exception as e:
        print(f"[ERROR] UART 포트 열기 실패: {e}")
        ser = None

    mpu = MPU6050(address=0x68)

    while True:
        accel_x, accel_y, accel_z = mpu.read_accel()
        gyro_x, gyro_y, gyro_z = mpu.read_gyro()

        # 거리 센서 없으면 0으로 임의 세팅 가능
        dist_cm = 0

        data = {
            'accX': accel_x,
            'accY': accel_y,
            'accZ': accel_z,
            'gyroX': gyro_x,
            'gyroY': gyro_y,
            'gyroZ': gyro_z,
            'dist_cm': dist_cm
        }

        if ser is not None:
            send_to_esp32(data, ser)
        else:
            print("[WARN] UART 포트가 열리지 않아 데이터 전송 불가")

        time.sleep(0.05)  # 20Hz 전송