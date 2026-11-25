import tkinter as tk
import serial
import threading
import time
import math
import struct

# === KONFIGURASI SERIAL ===
PORT = 'COM12'
BAUDRATE = 9600
MAX_JARAK_MM = 2500  # Jarak maksimal di layar radar (250 cm)

CMD_SYNC0     = bytes.fromhex('FA FB 03 00 00 00')
CMD_SYNC1     = bytes.fromhex('FA FB 03 01 00 01')
CMD_SYNC2     = bytes.fromhex('FA FB 03 02 00 02')
CMD_OPEN      = bytes.fromhex('FA FB 03 01 00 01')
CMD_SONAR_ON  = bytes.fromhex('FA FB 06 1C 3B 01 00 1D 3B')
CMD_SONAR_OFF = bytes.fromhex('FA FB 06 1C 3B 00 00 1C 3B')
CMD_CLOSE     = bytes.fromhex('FA FB 03 02 00 02')
CMD_PULSE     = bytes.fromhex('FA FB 03 00 00 00')

# === PETA SUDUT SENSOR HASIL MAPPING (0° = Depan, Searah Jarum Jam) ===
POSISI_SONAR = {
    # Menara Atas (Menghadap Depan, Kiri ke Kanan: 16 -> 23)
    16: 275,
    17: 305,
    18: 328,
    19: 348,
    20: 12,
    21: 32,
    22: 55,   # Mati (5000 mm)
    23: 85,
    # Bawah Belakang (Kanan ke Kiri: 08 -> 15)
    8:  95,
    9:  125,
    10: 148,
    11: 168,
    12: 192,
    13: 212,
    14: 235,
    15: 265,
}

# === PETA SUDUT 10 PELAT BUMPER ===
BUMPER_DEPAN = [
    (0x02, 288, "02"),
    (0x04, 324, "04"),
    (0x08, 0,   "08"),
    (0x10, 36,  "10"),
    (0x20, 72,  "20"),
]

BUMPER_BELAKANG = [
    (0x02, 108, "02"),
    (0x04, 144, "04"),
    (0x08, 180, "08"),
    (0x10, 216, "10"),
    (0x20, 252, "20"),
]


class RadarPeopleBot:
    def __init__(self, root):
        self.root = root
        self.root.title("Radar 2D Sensor, Bumper & Odometri - PeopleBot")
        self.root.configure(bg="#0f172a")
        self.root.resizable(False, False)

        self.lock = threading.Lock()
        self.all_sonars = {}
        self.bumper_bytes = (0, 0)
        self.voltase = 0.0
        self.raw_odom = (0, 0, 0.0)   # (x_mm, y_mm, theta_deg)
        self.offset_odom = (0, 0, 0.0)
        self.status_koneksi = "Menghubungkan ke COM12..."
        self.running = True
        self.ser = None

        # Area gambar radar (Canvas)
        self.size = 640
        self.cx = self.size // 2
        self.cy = self.size // 2
        self.r_robot = 58
        self.r_max = 265

        self.canvas = tk.Canvas(
            root, width=self.size, height=self.size,
            bg="#0f172a", highlightthickness=0
        )
        self.canvas.pack()

        # Frame bar bawah (Info + Tombol Reset Odometri)
        bottom_bar = tk.Frame(root, bg="#1e293b")
        bottom_bar.pack(fill=tk.X)

        self.label_info = tk.Label(
            bottom_bar, text=self.status_koneksi,
            font=("Consolas", 10, "bold"),
            fg="#38bdf8", bg="#1e293b", pady=8, padx=10
        )
        self.label_info.pack(side=tk.LEFT)

        btn_reset = tk.Button(
            bottom_bar, text="Reset Nol (R)",
            font=("Consolas", 9, "bold"),
            fg="#f8fafc", bg="#334155", activebackground="#475569",
            relief=tk.FLAT, padx=10, pady=4,
            command=self.reset_odometri
        )
        btn_reset.pack(side=tk.RIGHT, padx=10, pady=4)

        # Shortcut tombol keyboard 'R' untuk reset titik nol odometri
        self.root.bind("<r>", lambda e: self.reset_odometri())
        self.root.bind("<R>", lambda e: self.reset_odometri())

        self.thread = threading.Thread(target=self.baca_serial, daemon=True)
        self.thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self.tutup_aplikasi)
        self.gambar_radar()

    def reset_odometri(self):
        """Jadikan posisi dan sudut hadap roda saat ini sebagai titik 0."""
        with self.lock:
            self.offset_odom = self.raw_odom

    def ke_koordinat(self, sudut_deg, radius):
        """Ubah sudut (0° = atas/depan, searah jarum jam) menjadi koordinat X, Y layar."""
        rad = math.radians(sudut_deg)
        x = self.cx + radius * math.sin(rad)
        y = self.cy - radius * math.cos(rad)
        return x, y

    def baca_serial(self):
        try:
            self.ser = serial.Serial(PORT, BAUDRATE, timeout=0.5)
            time.sleep(0.3)

            self.ser.write(CMD_CLOSE)
            time.sleep(0.2)
            self.ser.reset_input_buffer()

            for cmd in [CMD_SYNC0, CMD_SYNC1, CMD_SYNC2]:
                self.ser.write(cmd)
                time.sleep(0.25)
                self.ser.read(self.ser.in_waiting)

            self.ser.write(CMD_OPEN)
            time.sleep(0.2)
            self.ser.write(CMD_SONAR_ON)
            time.sleep(0.2)

            with self.lock:
                self.status_koneksi = "TERHUBUNG | Putar roda pelan untuk uji kompas"

            last_pulse = time.time()
            while self.running:
                if time.time() - last_pulse >= 1.0:
                    self.ser.write(CMD_PULSE)
                    last_pulse = time.time()

                if self.ser.read(1) == b'\xfa' and self.ser.read(1) == b'\xfb':
                    len_b = self.ser.read(1)
                    if not len_b:
                        continue
                    pkt_len = len_b[0]
                    data = self.ser.read(pkt_len)
                    if len(data) == pkt_len and data[0] in (0x32, 0x33) and len(data) > 20:
                        # Baca X, Y, Theta dari byte 1..6 (signed 16-bit little-endian)
                        raw_x, raw_y, raw_th = struct.unpack('<hhh', data[1:7])
                        theta_deg = (raw_th * 360.0) / 4096.0
                        volt = data[11] / 10.0

                        with self.lock:
                            self.raw_odom = (raw_x, raw_y, theta_deg)
                            self.voltase = volt
                            # Byte 13 = Depan, Byte 12 = Belakang
                            self.bumper_bytes = (data[13], data[12])

                            sonar_count = data[19]
                            for i in range(sonar_count):
                                idx = 20 + (i * 3)
                                if idx + 2 < len(data) - 2:
                                    s_id = data[idx]
                                    dist = data[idx + 1] | (data[idx + 2] << 8)
                                    self.all_sonars[s_id] = dist
        except Exception as e:
            with self.lock:
                self.status_koneksi = f"Gagal koneksi {PORT}: {e}"

    def gambar_radar(self):
        self.canvas.delete("all")

        with self.lock:
            sonars = dict(self.all_sonars)
            b_depan, b_belakang = self.bumper_bytes
            status = self.status_koneksi
            volt = self.voltase
            rx, ry, rth = self.raw_odom
            ox, oy, oth = self.offset_odom

        # Hitung odometri relatif terhadap titik reset
        pos_x = rx - ox
        pos_y = ry - oy
        # Balik tanda agar putaran searah jarum jam bernilai positif di radar
        sudut_hadap = (-(rth - oth)) % 360.0

        # 1. Lingkaran Skala Jarak
        for jarak_cm in [50, 100, 150, 200, 250]:
            r = self.r_robot + (jarak_cm * 10 / MAX_JARAK_MM) * (self.r_max - self.r_robot)
            self.canvas.create_oval(
                self.cx - r, self.cy - r, self.cx + r, self.cy + r,
                outline="#1e293b", width=1
            )
            self.canvas.create_text(
                self.cx + 4, self.cy - r + 10,
                text=f"{jarak_cm} cm", fill="#475569",
                font=("Consolas", 8), anchor="w"
            )

        # 2. Garis Sumbu & Label Arah Robot
        self.canvas.create_line(self.cx, 25, self.cx, self.size - 25, fill="#1e293b", dash=(4, 4))
        self.canvas.create_line(25, self.cy, self.size - 25, self.cy, fill="#1e293b", dash=(4, 4))
        self.canvas.create_text(self.cx, 15, text="DEPAN (0°)", fill="#94a3b8", font=("Consolas", 10, "bold"))
        self.canvas.create_text(self.cx, self.size - 15, text="BELAKANG (180°)", fill="#94a3b8", font=("Consolas", 10, "bold"))
        self.canvas.create_text(self.size - 45, self.cy - 12, text="KANAN (90°)", fill="#64748b", font=("Consolas", 8))
        self.canvas.create_text(45, self.cy - 12, text="KIRI (270°)", fill="#64748b", font=("Consolas", 8))

        # 3. HUD Pojok Kiri Atas: Persentase & Tegangan Baterai
        # Aki 12V PeopleBot: 11.0V (0% / batas aman) s.d. 12.7V (100% penuh)
        if volt > 0:
            persen_bat = int(max(0, min(100, ((volt - 11.0) / (12.7 - 11.0)) * 100)))
        else:
            persen_bat = 0

        if persen_bat >= 55:
            warna_bat = "#43c522"   # Hijau
        elif persen_bat >= 25:
            warna_bat = "#facc15"   # Kuning
        else:
            warna_bat = "#ef4444"   # Merah

        self.canvas.create_rectangle(14, 14, 185, 76, fill="#1e293b", outline="#334155")
        self.canvas.create_text(24, 27, text="BATERAI ROBOT", fill="#94a3b8", font=("Consolas", 8, "bold"), anchor="w")

        if volt > 0:
            status_cas = " [CAS]" if volt > 13.2 else ""
            self.canvas.create_text(
                24, 45,
                text=f"{persen_bat}% ({volt:.1f} V){status_cas}",
                fill=warna_bat,
                font=("Consolas", 10, "bold"), anchor="w"
            )
            # Bar indikator baterai
            self.canvas.create_rectangle(24, 58, 174, 67, fill="#0f172a", outline="#475569")
            lebar_isi = int((persen_bat / 100.0) * 148)
            if lebar_isi > 0:
                self.canvas.create_rectangle(25, 59, 25 + lebar_isi, 66, fill=warna_bat, outline="")
        else:
            self.canvas.create_text(24, 46, text="Membaca...", fill="#64748b", font=("Consolas", 10, "bold"), anchor="w")
        

        # 4. HUD Pojok Kanan Atas: Odometri Roda (Kompas & Posisi)
        self.canvas.create_rectangle(self.size - 195, 14, self.size - 14, 78, fill="#1e293b", outline="#334155")
        self.canvas.create_text(self.size - 185, 27, text="ODOMETRI RODA", fill="#94a3b8", font=("Consolas", 8, "bold"), anchor="w")
        self.canvas.create_text(
            self.size - 185, 45,
            text=f"Hadap : {sudut_hadap:5.1f}°",
            fill="#38bdf8", font=("Consolas", 10, "bold"), anchor="w"
        )
        self.canvas.create_text(
            self.size - 185, 63,
            text=f"X:{pos_x:+d}  Y:{pos_y:+d}",
            fill="#cbd5e1", font=("Consolas", 9), anchor="w"
        )

        # 5. Area Blind Spot Samping (Roda Kiri & Kanan)
        self.canvas.create_text(self.cx - 115, self.cy + 14, text="BLIND SPOT\n(Roda Kiri)", fill="#475569", font=("Consolas", 8))
        self.canvas.create_text(self.cx + 115, self.cy + 14, text="BLIND SPOT\n(Roda Kanan)", fill="#475569", font=("Consolas", 8))

        # 6. Bodi Tengah Robot, Roda & Jarum Kompas Berputar
        self.canvas.create_rectangle(self.cx - 56, self.cy - 18, self.cx - 46, self.cy + 18, fill="#334155", outline="#64748b")
        self.canvas.create_rectangle(self.cx + 46, self.cy - 18, self.cx + 56, self.cy + 18, fill="#334155", outline="#64748b")
        self.canvas.create_oval(
            self.cx - 46, self.cy - 46, self.cx + 46, self.cy + 46,
            fill="#1e293b", outline="#64748b", width=2
        )

        # Jarum panah biru mengikuti sudut putaran roda (sudut_hadap)
        ujung_x, ujung_y = self.ke_koordinat(sudut_hadap, 36)
        kiri_x, kiri_y = self.ke_koordinat(sudut_hadap - 145, 22)
        tengah_x, tengah_y = self.ke_koordinat(sudut_hadap + 180, 10)
        kanan_x, kanan_y = self.ke_koordinat(sudut_hadap + 145, 22)
        self.canvas.create_polygon(
            ujung_x, ujung_y,
            kiri_x, kiri_y,
            tengah_x, tengah_y,
            kanan_x, kanan_y,
            fill="#38bdf8", outline="#e0f2fe", width=1
        )

        # Info sonar bawah depan (00-07) yang mati
        self.canvas.create_arc(
            self.cx - 72, self.cy - 72, self.cx + 72, self.cy + 72,
            start=25, extent=130, style="arc", outline="#475569", dash=(3, 3), width=1
        )
        self.canvas.create_text(self.cx, self.cy - 82, text="Sonar Bawah Depan (00-07): Mati", fill="#64748b", font=("Consolas", 7))

        # 7. Gambar 10 Segmen Bumper
        r_bmp = self.r_robot
        for mask, sudut, label in BUMPER_DEPAN:
            aktif = (b_depan & mask) != 0
            warna = "#ef4444" if aktif else "#475569"
            tk_start = (90 - sudut - 15) % 360
            self.canvas.create_arc(
                self.cx - r_bmp, self.cy - r_bmp, self.cx + r_bmp, self.cy + r_bmp,
                start=tk_start, extent=30, style="arc", outline=warna, width=6 if aktif else 4
            )
            lx, ly = self.ke_koordinat(sudut, r_bmp - 12)
            self.canvas.create_text(lx, ly, text=label, fill="#fca5a5" if aktif else "#64748b", font=("Consolas", 7, "bold"))

        for mask, sudut, label in BUMPER_BELAKANG:
            aktif = (b_belakang & mask) != 0
            warna = "#ef4444" if aktif else "#475569"
            tk_start = (90 - sudut - 15) % 360
            self.canvas.create_arc(
                self.cx - r_bmp, self.cy - r_bmp, self.cx + r_bmp, self.cy + r_bmp,
                start=tk_start, extent=30, style="arc", outline=warna, width=6 if aktif else 4
            )
            lx, ly = self.ke_koordinat(sudut, r_bmp - 12)
            self.canvas.create_text(lx, ly, text=label, fill="#fca5a5" if aktif else "#64748b", font=("Consolas", 7, "bold"))

        # 8. Gambar Pancaran & Titik Jarak Sensor Ultrasonik
        for s_id, sudut in POSISI_SONAR.items():
            x_asal, y_asal = self.ke_koordinat(sudut, self.r_robot + 6)

            if s_id == 22:
                self.canvas.create_text(x_asal, y_asal, text="X", fill="#ef4444", font=("Consolas", 10, "bold"))
                lx, ly = self.ke_koordinat(sudut, self.r_robot + 22)
                self.canvas.create_text(lx, ly, text="22(Mati)", fill="#ef4444", font=("Consolas", 8))
                continue

            jarak_mm = sonars.get(s_id, 5000)
            jarak_tampil = min(jarak_mm, MAX_JARAK_MM)
            r_titik = self.r_robot + 8 + (jarak_tampil / MAX_JARAK_MM) * (self.r_max - self.r_robot - 8)
            x_titik, y_titik = self.ke_koordinat(sudut, r_titik)

            if jarak_mm < 500:
                warna = "#ef4444"   # Merah: < 50 cm
            elif jarak_mm < 1000:
                warna = "#facc15"   # Kuning: < 100 cm
            elif jarak_mm < MAX_JARAK_MM:
                warna = "#22c55e"   # Hijau: < 250 cm
            else:
                warna = "#334155"   # Abu gelap: Bebas

            self.canvas.create_line(
                x_asal, y_asal, x_titik, y_titik,
                fill=warna, width=2 if jarak_mm < MAX_JARAK_MM else 1,
                dash=() if jarak_mm < MAX_JARAK_MM else (2, 4)
            )

            ukuran = 5 if jarak_mm < MAX_JARAK_MM else 3
            self.canvas.create_oval(
                x_titik - ukuran, y_titik - ukuran,
                x_titik + ukuran, y_titik + ukuran,
                fill=warna, outline=""
            )

            x_lbl, y_lbl = self.ke_koordinat(sudut, r_titik + 16)
            teks_lbl = f"ID{s_id:02d}:{jarak_mm//10}cm" if jarak_mm < MAX_JARAK_MM else f"{s_id:02d}"
            self.canvas.create_text(
                x_lbl, y_lbl, text=teks_lbl,
                fill="#f8fafc" if jarak_mm < MAX_JARAK_MM else "#64748b",
                font=("Consolas", 8, "bold" if jarak_mm < MAX_JARAK_MM else "normal")
            )

        self.label_info.config(
            text=f"{status} | Bumper: D[{b_depan:02X}] B[{b_belakang:02X}]"
        )

        if self.running:
            self.root.after(50, self.gambar_radar)

    def tutup_aplikasi(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(CMD_SONAR_OFF)
                time.sleep(0.15)
                self.ser.write(CMD_CLOSE)
                self.ser.close()
            except Exception:
                pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = RadarPeopleBot(root)
    root.mainloop()
