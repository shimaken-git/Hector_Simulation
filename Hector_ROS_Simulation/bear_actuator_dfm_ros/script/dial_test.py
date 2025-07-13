#!/usr/bin/env python3

import rospy
import rosparam

import serial

# COMポートとボーレートを指定（適宜変更）
PORT = '/dev/ttyACM0'  # 例: Windowsでは 'COM3', macOS/Linuxでは '/dev/ttyUSB0' など
BAUDRATE = 115200

try:
    # シリアルポートを開く
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        print(f"接続中: {PORT} ({BAUDRATE}bps)")

        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                if line[0:2] == "HD" :
                    print(f"受信: {line}", line[0:2])

except serial.SerialException as e:
    print(f"シリアルポートエラー: {e}")
except KeyboardInterrupt:
    print("\n受信を終了しました。")
