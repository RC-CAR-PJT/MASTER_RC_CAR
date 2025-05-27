import serial

try:
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    print("[SERIAL] UART port opened successfully")
except serial.SerialException as e:
    print(f"[ERROR] Failed to open UART port: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("[SERIAL] Port closed")
