from Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor
from Raspi_PWM_Servo_Driver import PWM
import threading
import signal
import sys
import json
import time
from datetime import datetime
import pytz

from MQTT_PROTOCOL import MQTTClient
from MPU6050 import MPU6050

# 모터 및 서보 초기화
mh = Raspi_MotorHAT(addr=0x6f)
myMotor = mh.getMotor(2)
carSpeed = 100  # 기본 속도 설정 (0 ~ 255)
carPWM = 210    # 기본 서보 PWM 값 (0 ~ 500 범위로 제한)

servo = PWM(0x6F)
servo.setPWMFreq(60)  # 서보 주파수 60Hz

# 센서 초기화
mpu = MPU6050()

# MQTT 클라이언트 초기화
mqtt_client = MQTTClient(broker="172.20.10.2", port=1883, control_topic="rc_car/command", 
                         sensor_topic="rc_car/sensing")
mqtt_client.setup()

# --- 차량 제어 함수 ---
def GO():
    myMotor.setSpeed(carSpeed)
    myMotor.run(Raspi_MotorHAT.FORWARD)
    print(f"[ACTION] GO with speed {carSpeed}")

def LEFT():
    global carPWM
    carPWM = min(400, carPWM + 10)  # carPWM을 10 증가, 최대 500으로 제한
    servo.setPWM(0, 0, carPWM)
    print(f"Updated carPWM: {carPWM}")
    print("[ACTION] LEFT")

def RIGHT():
    global carPWM
    carPWM = max(100, carPWM - 10)  # carPWM을 10 감소, 최소 0으로 제한
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

# 센서 데이터 전송
def send_sensor_data():
    while True:
        try:
            gyro_data = mpu.read_gyro()
            kst = pytz.timezone('Asia/Seoul')
            timestamp = datetime.now(kst).strftime('%Y-%m-%d %H:%M:%S')
            sensor_payload = json.dumps({
                "timestamp": timestamp,
                "gyro_x": gyro_data[0],
                "gyro_y": gyro_data[1],
                "gyro_z": gyro_data[2]
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
    print("[SYSTEM] Program exited.")
