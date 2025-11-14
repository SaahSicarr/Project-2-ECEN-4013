import tkinter as tk
from tkinter import ttk, messagebox
import serial, serial.tools.list_ports, threading, time

class MainApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Teensy Project 2 GUI")
        self.root.geometry("600x400")
        self.ser = None
        self.running = False

        self.main_frame = ttk.Frame(root)
        self.data_frame = ttk.Frame(root)

        self.build_main_menu()

    # ---------- Main Menu ----------
    def build_main_menu(self):
        self.clear_frame()
        ttk.Label(self.main_frame, text="Teensy Data Viewer", font=("Arial", 18)).pack(pady=20)
        ttk.Button(self.main_frame, text="Start Display", command=self.start_display).pack(pady=10)
        ttk.Button(self.main_frame, text="Exit", command=self.exit_app).pack(pady=10)
        self.main_frame.pack(fill="both", expand=True)

    # ---------- Data Display Screen ----------
    def build_data_screen(self):
        self.clear_frame()

        # Title
        ttk.Label(self.data_frame, text="Live Data Display", font=("Arial", 16)).grid(row=0, column=0, columnspan=2, pady=10)

        # GPS Section
        ttk.Label(self.data_frame, text="GPS Data", font=("Arial", 14, "underline")).grid(row=1, column=0, sticky="w", padx=10)
        gps_labels = ["Latitude", "Longitude", "Elevation (m)"]
        self.gps_vars = {}
        for i, name in enumerate(gps_labels):
            ttk.Label(self.data_frame, text=f"{name}:").grid(row=i+2, column=0, sticky="e", padx=5)
            var = tk.StringVar(value="--")
            ttk.Label(self.data_frame, textvariable=var).grid(row=i+2, column=1, sticky="w")
            self.gps_vars[name] = var

        # IMU Section
        ttk.Label(self.data_frame, text="IMU Data", font=("Arial", 14, "underline")).grid(row=5, column=0, sticky="w", padx=10, pady=(10,0))
        imu_labels = [
            "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z",
            "Acceleration X", "Acceleration Y", "Acceleration Z",
            "Magnetic Field X", "Magnetic Field Y", "Magnetic Field Z"
        ]
        self.imu_vars = {}
        for i, name in enumerate(imu_labels):
            ttk.Label(self.data_frame, text=f"{name}:").grid(row=i+6, column=0, sticky="e", padx=5)
            var = tk.StringVar(value="--")
            ttk.Label(self.data_frame, textvariable=var).grid(row=i+6, column=1, sticky="w")
            self.imu_vars[name] = var

        ttk.Button(self.data_frame, text="End Display", command=self.stop_display).grid(row=15, column=0, columnspan=2, pady=15)
        self.data_frame.pack(fill="both", expand=True)

    # ---------- Start / Stop ----------
    def start_display(self):
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            if not ports:
                messagebox.showerror("Error", "No USB device detected.")
                return
            port = ports[0]  # auto-select first detected port
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.running = True
            self.build_data_screen()
            threading.Thread(target=self.read_serial_loop, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Connection Error", f"Could not open serial port:\n{e}")
            self.build_main_menu()

    def stop_display(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.build_main_menu()

    def exit_app(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()

    # ---------- Serial Reader ----------
    def read_serial_loop(self):
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                parts = line.split(",")
                if len(parts) < 14:
                    continue  # ignore incomplete lines

                # Format: time,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt,sats
                ax, ay, az = parts[1:4]
                gx, gy, gz = parts[4:7]
                mx, my, mz = parts[7:10]
                lat, lon, alt, sats = parts[10:14]

                # Update IMU display
                self.imu_vars["Acceleration X"].set(ax)
                self.imu_vars["Acceleration Y"].set(ay)
                self.imu_vars["Acceleration Z"].set(az)
                self.imu_vars["Angular Velocity X"].set(gx)
                self.imu_vars["Angular Velocity Y"].set(gy)
                self.imu_vars["Angular Velocity Z"].set(gz)
                self.imu_vars["Magnetic Field X"].set(mx)
                self.imu_vars["Magnetic Field Y"].set(my)
                self.imu_vars["Magnetic Field Z"].set(mz)

                # Update GPS display
                self.gps_vars["Latitude"].set(lat)
                self.gps_vars["Longitude"].set(lon)
                self.gps_vars["Elevation (m)"].set(alt)

            except Exception as e:
                print("Read error:", e)
                break

    # ---------- Helpers ----------
    def clear_frame(self):
        for w in self.root.winfo_children():
            w.pack_forget()
        for f in (self.main_frame, self.data_frame):
            for w in f.winfo_children():
                w.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MainApp(root)
    root.mainloop()
