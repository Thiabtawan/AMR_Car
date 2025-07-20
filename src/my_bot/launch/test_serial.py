#!/usr/bin/env python3
import serial, time

PORT = '/dev/ttyUSB0'
BAUD = 57600

print(f"Opening {PORT} @ {BAUD}bps")
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)                    # รอ Arduino รีบูท/พร้อมรับ
ser.reset_input_buffer()
ser.reset_output_buffer()

while True:
    # ส่ง 'e' + CRLF
    ser.write(b'e\r\n')
    time.sleep(0.05)
    # อ่านทั้งหมดใน buffer
    raw = ser.read_all()
    text = raw.decode(errors='ignore').strip()
    print(f"Raw bytes: {raw!r} → '{text}'")
    time.sleep(0.5)
