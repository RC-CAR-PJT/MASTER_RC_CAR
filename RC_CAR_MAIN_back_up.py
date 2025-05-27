from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
from Raspi_PWM_Servo_Driver import PWM
import threading
import signal
import sys
import json
import time
from datetime import datetime
import pytz
import serial
import re

from MQTT_PROTOCOL import MQTTClient

# 모터 및 서보 초기화
mh = Raspi_MotorHAT(addr=0x6f)
myMotor = mh.getMotor(2)
carSpeed = 100  # 기본 속도 설정 (0 ~ 255)
carPWM = 210    # 기본 서보 PWM 값 (100 ~ 400 범위로 제한)

servo = PWM(0x6F)
servo.setPWMFreq(60)  # 서보 주파수 60Hz

# UART 초기화
try:
    uart = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.1)
    print("[SYSTEM] UART 포트 열림")
except Exception as e:
    print(f"[ERROR] UART 포트 열기 실패: {e}")
    uart = None

# MQTT 클라이언트 초기화
mqtt_client = MQTTClient(broker="192.168.137.208", port=1883, control_topic="rc_car/command", 
                         sensor_topic="rc_car/sensing")
mqtt_client.setup()

# --- 차량 제어 함수 ---
def GO():
    myMotor.setSpeed(carSpeed)
    myMotor.run(Raspi_MotorHAT.FORWARD)
    print(f"[ACTION] GO with speed {carSpeed}")

def LEFT():
    global carPWM
    carPWM = min(400, carPWM + 10)  # carPWM을 10 증가, 최대 400으로 제한
    servo.setPWM(0, 0, carPWM)
    print(f"Updated carPWM: {carPWM}")
    print("[ACTION] LEFT")

def RIGHT():
    global carPWM
    carPWM = max(100, carPWM - 10)  # carPWM을 10 감소, 최소 100으로 제한
    servo.setPWM(0, 0, carPWM)
    print(f"Updated carPWM: {carPWM}")
    print("[ACTION] RIGHT")

def BACK():
    myMotor.setSpeed(carSpeed)
    myMotor.run(Raspi_MotorHAT.BACKWARD)
    print(f"[ACTION] BACK with speed {carSpeed}")

def STOP():
    myMotor.run(Raspi_MotorHAT.RELEASE)
    servo.setPWM(0, 0, 210)
    print("[ACTION] STOP (motor released & servo centered)")

def MIDDLE():
    global carPWM
    carPWM = 210  # MIDDLE 호출 시 carPWM을 기본값(중앙)으로 재설정
    servo.setPWM(0, 0, carPWM)
    print(f"Updated carPWM: {carPWM}")
    print("[ACTION] MIDDLE (servo centered)")

# MQTT 콜백: 명령 처리
def handle_command(cmd):
    global carSpeed
    global carPWM
    cmd = cmd.strip().upper()
    if cmd == 'GO':
        GO()
    elif cmd == 'LEFT':
        LEFT()
    elif cmd == 'RIGHT':
        RIGHT()
    elif cmd == 'BACK':
        BACK()
    elif cmd == 'STOP':
        STOP()
    elif cmd == 'MIDDLE':
        MIDDLE()
    elif cmd.isdigit():
        speed_val = int(cmd)
        if 0 <= speed_val <= 255:
            carSpeed = speed_val
            myMotor.setSpeed(carSpeed)
            print(f"[INFO] 속도 설정됨: {carSpeed}")
        else:
            print(f"[WARN] 속도 범위 초과: {speed_val} (0~255만 허용)")
    else:
        print(f"[WARN] 알 수 없는 명령: {cmd}")

# 센서 데이터 수신 (슬레이브 코드와 동일한 방식)
def receive_sensor_data(uart):
    """
    UART로부터 ESP32에서 전송된 gyro 데이터를 읽어오는 함수
    예: "GYRO: -0.05,0.00,0.03"
    """
    gyro_x = gyro_y = gyro_z = 0.0
    if uart is None:
        print("[WARN] UART 포트가 열리지 않음")
        return gyro_x, gyro_y, gyro_z

    while uart.in_waiting:
        line = uart.readline().decode('utf-8', errors='ignore').strip()
        # 슬레이브와 동일한 형식: "GYRO: x,y,z"
        m = re.search(r"GYRO:\s*(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)", line)
        if m:
            gyro_x = float(m.group(1))
            gyro_y = float(m.group(2))
            gyro_z = float(m.group(3))
            print(f"[DEBUG] Parsed UART data: GYRO: {gyro_x}, {gyro_y}, {gyro_z}")
    return gyro_x, gyro_y, gyro_z

# 센서 데이터 전송
def send_sensor_data():
    while True:
        try:
            if uart:
                gyro_x, gyro_y, gyro_z = receive_sensor_data(uart)
                kst = pytz.timezone('Asia/Seoul')
                timestamp = datetime.now(kst).strftime('%Y-%m-%d %H:%M:%S')
                sensor_payload = json.dumps({
                    "timestamp": timestamp,
                    "gyro_x": gyro_x,
                    "gyro_y": gyro_y,
                    "gyro_z": gyro_z
                })
                mqtt_client.publish(mqtt_client.sensor_topic, sensor_payload)
                print(f"[SENSOR] Sent: {sensor_payload}")
            time.sleep(0.5)
        except Exception as e:
            print(f"[ERROR] 센서 데이터 전송 중 오류: {e}")
            time.sleep(1)

# 프로그램 종료 핸들러
def signal_handler(sig, frame):
    print('[SYSTEM] Exiting program...')
    STOP()
    mqtt_client.disconnect()
    if uart:
        uart.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# 센서 데이터 전송 스레드 시작
sensor_thread = threading.Thread(target=send_sensor_data, daemon=True)
sensor_thread.start()

# 메인 루프에서 MQTT 메시지 처리
try:
    while True:
        mqtt_client.client.loop(0.01)  # 수동으로 MQTT 루프 실행
        time.sleep(0.01)  # CPU 사용량 최소화
except KeyboardInterrupt:
    print("[SYSTEM] Keyboard interrupt received")
finally:
    STOP()
    mqtt_client.disconnect()
    if uart:
        uart.close()
    print("[SYSTEM] Program exited.")