import paho.mqtt.client as mqtt
import json

class MQTTClient:
    def __init__(self, broker, port, control_topic, sensor_topic):
        self.broker = broker
        self.port = port
        self.control_topic = control_topic
        self.sensor_topic = sensor_topic
        self.client = mqtt.Client()

    def on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] Connected with result code {rc}")
        self.client.subscribe(self.control_topic)
        print(f"[MQTT] Subscribed to {self.control_topic}")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            print(f"[MQTT] Received message: {payload} on topic: {msg.topic}")
            data = json.loads(payload)
            cmd = str(data.get('command')).strip()
            from RC_CAR_MAIN import handle_command
            handle_command(cmd)
        except Exception as e:
            print(f"[ERROR] MQTT 메시지 처리 중 오류: {e}")

    def setup(self):
        self.client.on_connect = self.on_connect  # 올바르게 콜백 설정
        self.client.on_message = self.on_message
        self.client.on_disconnect = lambda *args: print("[MQTT] Disconnected.")
        try:
            self.client.connect(self.broker, self.port, 60)
            # loop_start() 대신 메인 스레드에서 처리하도록 설정
        except Exception as e:
            print(f"[ERROR] MQTT 연결 실패: {e}")

    def publish(self, topic, payload):
        try:
            self.client.publish(topic, payload)
        except Exception as e:
            print(f"[ERROR] MQTT 퍼블리시 실패: {e}")

    def disconnect(self):
        try:
            self.client.disconnect()
            print("[MQTT] Disconnected.")
        except Exception as e:
            print(f"[ERROR] MQTT 연결 해제 실패: {e}")