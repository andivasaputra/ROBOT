import serial
import time
import os

PORT = 'COM12'
BAUDRATE = 9600

CMD_SYNC0     = bytes.fromhex('FA FB 03 00 00 00')
CMD_SYNC1     = bytes.fromhex('FA FB 03 01 00 01')
CMD_SYNC2     = bytes.fromhex('FA FB 03 02 00 02')
CMD_OPEN      = bytes.fromhex('FA FB 03 01 00 01')
CMD_SONAR_ON  = bytes.fromhex('FA FB 06 1C 3B 01 00 1D 3B')
CMD_SONAR_OFF = bytes.fromhex('FA FB 06 1C 3B 00 00 1C 3B')
CMD_CLOSE     = bytes.fromhex('FA FB 03 02 00 02')
CMD_PULSE     = bytes.fromhex('FA FB 03 00 00 00')

ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
time.sleep(0.3)

ser.write(CMD_CLOSE)
time.sleep(0.2)
ser.reset_input_buffer()

for cmd in [CMD_SYNC0, CMD_SYNC1, CMD_SYNC2]:
    ser.write(cmd)
    time.sleep(0.25)
    ser.read(ser.in_waiting)

ser.write(CMD_OPEN)
time.sleep(0.2)
ser.write(CMD_SONAR_ON)
time.sleep(0.2)

all_sonars = {}
bumper_bytes = (0, 0)
last_pulse = time.time()
last_print = time.time()

try:
    while True:
        if time.time() - last_pulse >= 1.0:
            ser.write(CMD_PULSE)
            last_pulse = time.time()

        b1 = ser.read(1)
        if b1 == b'\xfa':
            b2 = ser.read(1)
            if b2 == b'\xfb':
                length_byte = ser.read(1)
                if not length_byte:
                    continue
                pkt_len = length_byte[0]
                data = ser.read(pkt_len)
                
                if len(data) == pkt_len:
                    pkt_type = data[0]
                    if pkt_type in (0x32, 0x33) and len(data) > 20:
                        # Ambil 2 byte status bumper/stall (byte ke-12 & ke-13)
                        bumper_bytes = (data[12], data[13])
                        
                        sonar_count = data[19]
                        for i in range(sonar_count):
                            idx = 20 + (i * 3)
                            if idx + 2 < len(data) - 2:
                                s_id = data[idx]
                                dist_mm = data[idx + 1] | (data[idx + 2] << 8)
                                all_sonars[s_id] = dist_mm

        if time.time() - last_print >= 0.25 and all_sonars:
            os.system('cls' if os.name == 'nt' else 'clear')
            b_hi, b_lo = bumper_bytes
            status_bmp = "DITEKAN!" if (b_hi != 0 or b_lo != 0) else "Normal (Tidak ditekan)"
            
            print("=== MONITOR SENSOR & BUMPER PEOPLEBOT ===")
            print(f"Status Bumper : {status_bmp}")
            print(f"HEX Bumper    : {b_hi:02X} {b_lo:02X}")
            print(f"Biner (Bit)   : Byte1[{b_hi:08b}]  Byte2[{b_lo:08b}]")
            print("-" * 45)
            
            for s_id in sorted(all_sonars.keys()):
                jarak = all_sonars[s_id]
                tanda = " <-- DEKAT" if jarak < 400 else ""
                print(f"Sonar ID {s_id:02d} : {jarak:4d} mm{tanda}")
            print("\nTekan Ctrl+C untuk selesai.")
            last_print = time.time()

except KeyboardInterrupt:
    ser.write(CMD_SONAR_OFF)
    time.sleep(0.2)
    ser.write(CMD_CLOSE)
    ser.close()
    print("\nSelesai.")
