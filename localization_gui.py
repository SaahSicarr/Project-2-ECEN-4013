import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time


class MainApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Teensy Project 2 GUI")
        self.root.geometry("600x450")

        self.ser = None
        self.running = False

        # Frames
        self.main_frame = ttk.Frame(root)
        self.data_frame = ttk.Frame(root)

        # Serial / ports
        self.port_var = tk.StringVar()
        self.available_ports = []

        self.build_main_menu()

    # ---------- Main Menu ----------
    def build_main_menu(self):
        self.clear_frame()
        self.main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        ttk.Label(
            self.main_frame,
            text="Teensy Data Viewer",
            font=("Arial", 18)
        ).pack(pady=20)

        # Serial port selection section
        port_frame = ttk.Frame(self.main_frame)
        port_frame.pack(pady=10)

        ttk.Label(port_frame, text="Serial Port:").grid(row=0, column=0, padx=5, pady=5)

        self.port_combo = ttk.Combobox(
            port_frame,
            textvariable=self.port_var,
            state="readonly",
            width=15
        )
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(
            port_frame,
            text="Refresh Ports",
            command=self.refresh_ports
        ).grid(row=0, column=2, padx=5, pady=5)

        # Buttons
        ttk.Button(
            self.main_frame,
            text="Start Display",
            command=self.start_display
        ).pack(pady=10)

        ttk.Button(
            self.main_frame,
            text="Exit",
            command=self.exit_app
        ).pack(pady=10)

        # Initial port scan
        self.refresh_ports()

    # ---------- Refresh serial ports ----------
    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        self.available_ports = [p.device for p in ports]

        if not self.available_ports:
            self.port_combo["values"] = []
            self.port_var.set("")
            messagebox.showerror("Error", "No serial ports detected.")
            return

        # Populate combobox
        self.port_combo["values"] = self.available_ports

        # Default to COM9 if available, otherwise first port
        if "COM9" in self.available_ports:
            self.port_var.set("COM9")
        else:
            self.port_var.set(self.available_ports[0])

    # ---------- Data Display Screen ----------
    def build_data_screen(self):
        self.clear_frame()
        self.data_frame.pack(fill="both", expand=True, padx=10, pady=10)

        ttk.Label(
            self.data_frame,
            text="Live Data Display",
            font=("Arial", 16)
        ).grid(row=0, column=0, columnspan=2, pady=10)

        # GPS Section
        ttk.Label(
            self.data_frame,
            text="GPS Data",
            font=("Arial", 14, "underline")
        ).grid(row=1, column=0, sticky="w", padx=10)

        gps_labels = ["Latitude", "Longitude", "Elevation (m)"]
        self.gps_vars = {}

        for i, name in enumerate(gps_labels):
            ttk.Label(self.data_frame, text=f"{name}:").grid(
                row=i + 2, column=0, sticky="e", padx=5
            )
            var = tk.StringVar(value="--")
            ttk.Label(self.data_frame, textvariable=var).grid(
                row=i + 2, column=1, sticky="w"
            )
            self.gps_vars[name] = var

        # IMU Section
        ttk.Label(
            self.data_frame,
            text="IMU Data",
            font=("Arial", 14, "underline")
        ).grid(row=5, column=0, sticky="w", padx=10, pady=(10, 0))

        imu_labels = [
            "Angular Velocity X", "Angular Velocity Y", "Angular Velocity Z",
            "Acceleration X", "Acceleration Y", "Acceleration Z",
            "Magnetic Field X", "Magnetic Field Y", "Magnetic Field Z"
        ]
        self.imu_vars = {}

        for i, name in enumerate(imu_labels):
            ttk.Label(self.data_frame, text=f"{name}:").grid(
                row=i + 6, column=0, sticky="e", padx=5
            )
            var = tk.StringVar(value="--")
            ttk.Label(self.data_frame, textvariable=var).grid(
                row=i + 6, column=1, sticky="w"
            )
            self.imu_vars[name] = var

        ttk.Button(
            self.data_frame,
            text="End Display",
            command=self.stop_display
        ).grid(row=15, column=0, columnspan=2, pady=15)

    # ---------- Start / Stop ----------
    def start_display(self):
        selected_port = self.port_var.get().strip()

        if not selected_port:
            messagebox.showerror("Error", "No serial port selected.")
            return

        try:
            # Close any previous connection
            if self.ser and self.ser.is_open:
                self.ser.close()

            # Open chosen port
            self.ser = serial.Serial(selected_port, 115200, timeout=1)

            # Give Teensy time to reset after opening the port
            time.sleep(2)

            self.running = True
            self.build_data_screen()

            # Start background thread to read serial data
            threading.Thread(
                target=self.read_serial_loop,
                daemon=True
            ).start()

        except Exception as e:
            self.ser = None
            self.running = False
            messagebox.showerror(
                "Connection Error",
                f"Could not open serial port {selected_port}:\n{e}"
            )
            # Go back to main menu just in case
            self.build_main_menu()

    def stop_display(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.build_main_menu()

    def exit_app(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
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

        # If loop exits unexpectedly, make sure connection is closed
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass

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
