import tkinter as tk
from tkinter import ttk, filedialog
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
import numpy as np
import csv
import os
import cv2
import socket
from PIL import Image, ImageTk
import threading
import time
from datetime import datetime

matplotlib.use("TkAgg")

class CombinedDashboard:
    def __init__(self, root):
        self.dark_mode = tk.BooleanVar(value=False)
        self.root = root
        self.root.title("Combined Camera and Sensor Dashboard")
        self.root.geometry("1400x800")  # Increased height to accommodate GPS info

        # Main frames for layout
        self.left_frame = tk.Frame(root)
        self.left_frame.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=5, pady=5)
        
        self.right_frame = tk.Frame(root)
        self.right_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=5, pady=5)
        
        self.gps_frame = tk.Frame(root)
        self.gps_frame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        
        self.control_frame = tk.Frame(root)
        self.control_frame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        
        # Configure grid weights
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)  # Camera side
        self.root.grid_columnconfigure(1, weight=1)  # Sensor charts side

        # Initialize theme
        self.set_theme()

        # Initialize camera stream
        try:
            pi_hostname = "raspberrypi.local"
            pi_ip = socket.gethostbyname(pi_hostname)
            print(f"Discovered Raspberry Pi at {pi_ip}")
            self.video_source = f"http://{pi_ip}:8000/stream.mjpg"
        except socket.gaierror:
            print("Could not find Raspberry Pi on the network.")
            self.video_source = 0  # Default to webcam if Pi not found
        
        self.snapshot_count = 1
        self.stream = None
        self.frame = None

        # Initialize data buffers for sensor data
        self.time_data = list(range(20))
        self.acc_data = {'X': [], 'Y': [], 'Z': []}
        self.gyro_data = {'X': [], 'Y': [], 'Z': []}
        self.mag_data = {'X': [], 'Y': [], 'Z': []}
        self.temp_data = []
        self.humidity_data = []
        self.pressure_data = []
        self.altitude_data = []
        self.heading_data = []
        
        # GPS related data
        self.lat = "No Signal"
        self.long = "No Signal"
        self.num_satellites = 0
        self.hdop = 9999
        
        # Create data log file
        self.create_log_file()

        # Create all widgets
        self.create_camera_widgets()
        self.create_sensor_widgets()
        self.create_gps_widgets()
        self.create_control_widgets()

        # Start data updates
        self.update_camera()
        self.update_sensor_data()

    def create_log_file(self):
        # Create a CSV file for logging all data points
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"sensor_log_{timestamp}.csv"
        
        with open(self.log_filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            # Headers
            headers = [
                'Timestamp', 'Temperature', 'Humidity', 'LAT', 'LONG', 'No_SAT', 
                'HDOP', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ', 
                'Heading', 'Pressure', 'Altitude'
            ]
            writer.writerow(headers)
        
        print(f"Created log file: {self.log_filename}")

    def set_theme(self):
        if self.dark_mode.get():
            plt.style.use("dark_background")
            self.bg = "#2E2E2E"
            self.fg = "white"
            self.button_bg = "#444"
            self.line_colors = {'X': 'cyan', 'Y': 'magenta', 'Z': 'yellow'}
            self.temp_color = 'orange'
            self.humidity_color = 'skyblue'
            self.pressure_color = 'lime'
            self.altitude_color = 'yellow'
            self.heading_color = 'magenta'
        else:
            plt.style.use("default")
            self.bg = "white"
            self.fg = "black"
            self.button_bg = "#ddd"
            self.line_colors = {'X': 'blue', 'Y': 'red', 'Z': 'green'}
            self.temp_color = 'darkorange'
            self.humidity_color = 'royalblue'
            self.pressure_color = 'green'
            self.altitude_color = 'brown'
            self.heading_color = 'purple'

        self.root.configure(bg=self.bg)
        self.left_frame.configure(bg=self.bg)
        self.right_frame.configure(bg=self.bg)
        self.gps_frame.configure(bg=self.bg)
        self.control_frame.configure(bg=self.bg)
        
        # Update all child widgets
        for frame in [self.left_frame, self.right_frame, self.gps_frame, self.control_frame]:
            for widget in frame.winfo_children():
                if isinstance(widget, tk.Button) or isinstance(widget, tk.Checkbutton) or isinstance(widget, tk.Entry):  
                    widget.configure(bg=self.button_bg if isinstance(widget, tk.Button) else self.bg,
                                    fg=self.fg, insertbackground=self.fg if isinstance(widget, tk.Entry) else None)
                elif isinstance(widget, tk.Label):
                    widget.configure(bg=self.bg, fg=self.fg)

    def create_camera_widgets(self):
        # Create canvas for video stream
        self.camera_canvas = tk.Canvas(self.left_frame, width=640, height=480, bg=self.bg)
        self.camera_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Button to take snapshots
        self.btn_snapshot = tk.Button(self.left_frame, text="Take Snapshot", width=20, 
                                      command=self.snapshot, bg="#4CAF50", fg="white", font=("Arial", 10))
        self.btn_snapshot.pack(pady=10)
        
        # Status label for camera
        self.camera_status = tk.Label(self.left_frame, text="Connecting to camera...", 
                                      bd=1, relief=tk.SUNKEN, anchor=tk.W, bg=self.bg, fg=self.fg)
        self.camera_status.pack(fill=tk.X, pady=5)

    def create_sensor_widgets(self):
        self.figures, self.axes, self.canvases = [], [], []

        # Create 3x2 grid for sensor charts in right frame
        for i in range(6):
            frame = tk.Frame(self.right_frame, bg=self.bg)
            frame.grid(row=i // 2, column=i % 2, sticky="nsew", padx=5, pady=5)
            self.right_frame.grid_rowconfigure(i // 2, weight=1)
            self.right_frame.grid_columnconfigure(i % 2, weight=1)

            fig, ax = plt.subplots(figsize=(4, 2.5))
            canvas = FigureCanvasTkAgg(fig, master=frame)
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

            self.figures.append(fig)
            self.axes.append(ax)
            self.canvases.append(canvas)

    def create_gps_widgets(self):
        # Create labels to display GPS and other sensor info
        gps_frame_inner = tk.Frame(self.gps_frame, bg=self.bg)
        gps_frame_inner.pack(fill=tk.X, expand=True)
        
        # Create two rows of data displays
        for i in range(2):
            gps_frame_inner.grid_columnconfigure(i, weight=1)
        
        # First row
        self.lat_label = tk.Label(gps_frame_inner, text="LAT: No Signal", font=("Arial", 12), bg=self.bg, fg=self.fg)
        self.lat_label.grid(row=0, column=0, sticky="w", padx=10, pady=5)
        
        self.long_label = tk.Label(gps_frame_inner, text="LONG: No Signal", font=("Arial", 12), bg=self.bg, fg=self.fg)
        self.long_label.grid(row=0, column=1, sticky="w", padx=10, pady=5)
        
        self.sat_label = tk.Label(gps_frame_inner, text="No. of SAT: 0", font=("Arial", 12), bg=self.bg, fg=self.fg)
        self.sat_label.grid(row=0, column=2, sticky="w", padx=10, pady=5)
        
        # Second row
        self.hdop_label = tk.Label(gps_frame_inner, text="HDOP: 9999", font=("Arial", 12), bg=self.bg, fg=self.fg)
        self.hdop_label.grid(row=1, column=0, sticky="w", padx=10, pady=5)
        
        self.heading_label = tk.Label(gps_frame_inner, text="Heading: 0°", font=("Arial", 12), bg=self.bg, fg=self.fg)
        self.heading_label.grid(row=1, column=1, sticky="w", padx=10, pady=5)
        
        self.altitude_label = tk.Label(gps_frame_inner, text="Altitude: 0m", font=("Arial", 12), bg=self.bg, fg=self.fg)
        self.altitude_label.grid(row=1, column=2, sticky="w", padx=10, pady=5)

    def create_control_widgets(self):
        # Dark mode toggle
        dark_mode_btn = tk.Checkbutton(self.control_frame, text="Dark Mode", variable=self.dark_mode, 
                                       command=self.toggle_theme, bg=self.bg, fg=self.fg, 
                                       selectcolor=self.bg, font=("Arial", 10))
        dark_mode_btn.pack(side=tk.LEFT, padx=10, pady=5)

        # Save buttons
        save_plots_btn = tk.Button(self.control_frame, text="Save Plots", command=self.save_plots,
                            bg="#2196F3", fg="white", font=("Arial", 10))
        save_plots_btn.pack(side=tk.RIGHT, padx=10, pady=5)
        
        save_csv_btn = tk.Button(self.control_frame, text="Save CSV", command=self.save_csv,
                            bg="#2196F3", fg="white", font=("Arial", 10))
        save_csv_btn.pack(side=tk.RIGHT, padx=10, pady=5)

    def update_camera(self):
        # Get a frame from the video source
        try:
            if not self.stream:
                self.stream = cv2.VideoCapture(self.video_source)
                self.camera_status.config(text=f"Connected to camera at {self.video_source}")
            
            ret, self.frame = self.stream.read()
            if ret:
                self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
                # Resize frame to fit canvas if needed
                height, width = self.frame.shape[:2]
                canvas_width = self.camera_canvas.winfo_width()
                canvas_height = self.camera_canvas.winfo_height()
                
                if canvas_width > 1 and canvas_height > 1:  # Ensure canvas is initialized
                    scale = min(canvas_width/width, canvas_height/height)
                    new_width = int(width * scale)
                    new_height = int(height * scale)
                    resized_frame = cv2.resize(self.frame, (new_width, new_height))
                    
                    self.photo = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)))
                    self.camera_canvas.create_image(canvas_width//2, canvas_height//2, image=self.photo, anchor=tk.CENTER)
                else:
                    self.photo = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)))
                    self.camera_canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
            else:
                self.camera_status.config(text="Failed to get frame from camera")
        except Exception as e:
            self.camera_status.config(text=f"Camera error: {str(e)}")
        
        self.root.after(15, self.update_camera)  # Update camera every 15ms

    def update_sensor_data(self):
        # Generate simulated data
        acc_x, acc_y, acc_z = random.uniform(-2, 2), random.uniform(-2, 2), -9.81 + random.uniform(-2, 2)
        gyro_x, gyro_y, gyro_z = random.uniform(0, 50), random.uniform(0, 30), random.uniform(0, 90)
        mag_x, mag_y, mag_z = random.uniform(25, 27), random.uniform(25, 27), random.uniform(25, 27)
        temperature = random.uniform(25, 27)
        humidity = random.uniform(30, 50)
        heading = random.uniform(150, 200)
        pressure = 101325 + random.uniform(-100, 100)  # Sea level pressure in Pascal with randomness
        altitude = random.uniform(1, 7)  # 1-7 meters above sea level

        # Keep last 20 values
        for buffer, new_vals in zip(
            [self.acc_data, self.gyro_data, self.mag_data],
            [[acc_x, acc_y, acc_z], [gyro_x, gyro_y, gyro_z], [mag_x, mag_y, mag_z]]
        ):
            for k, v in zip(['X', 'Y', 'Z'], new_vals):
                buffer[k].append(v)
                if len(buffer[k]) > 20:
                    buffer[k].pop(0)

        # Update other data buffers
        for buffer, new_val in zip(
            [self.temp_data, self.humidity_data, self.heading_data, self.pressure_data, self.altitude_data],
            [temperature, humidity, heading, pressure, altitude]
        ):
            buffer.append(new_val)
            if len(buffer) > 20:
                buffer.pop(0)

        # Update GPS labels
        self.lat_label.config(text=f"LAT: {self.lat}")
        self.long_label.config(text=f"LONG: {self.long}")
        self.sat_label.config(text=f"No. of SAT: {self.num_satellites}")
        self.hdop_label.config(text=f"HDOP: {self.hdop}")
        self.heading_label.config(text=f"Heading: {heading:.1f}°")
        self.altitude_label.config(text=f"Altitude: {altitude:.1f}m")

        # Plot Accelerometer
        self.plot_sensor(0, "Accelerometer (m/s²)", self.acc_data)
        # Plot Gyroscope
        self.plot_sensor(1, "Gyroscope (°)", self.gyro_data)
        # Plot Magnetometer
        self.plot_sensor(2, "Magnetometer (μT)", self.mag_data)

        # Plot Temperature and Humidity
        ax = self.axes[3]
        ax.clear()
        ax.plot(self.temp_data, label="Temp (°C)", color=self.temp_color)
        ax.set_ylabel("Temperature (°C)", color=self.temp_color)
        ax.tick_params(axis='y', labelcolor=self.temp_color)
        
        # Create twin axis for humidity
        ax2 = ax.twinx()
        ax2.plot(self.humidity_data, label="Humidity (%)", color=self.humidity_color)
        ax2.set_ylabel("Humidity (%)", color=self.humidity_color)
        ax2.tick_params(axis='y', labelcolor=self.humidity_color)
        ax2.set_ylim(40, 90)
        
        ax.set_title("Temperature & Humidity", color=self.fg)
        ax.set_ylim(20, 30)
        
        # Create combined legend
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
        
        self.canvases[3].draw()

        # Plot Pressure
        ax = self.axes[4]
        ax.clear()
        ax.plot(self.pressure_data, label="Pressure", color=self.pressure_color)
        ax.set_title("Sea Level Pressure (Pa)", color=self.fg)
        ax.set_ylim(101000, 101650)
        ax.legend()
        self.canvases[4].draw()

        # Plot Heading
        ax = self.axes[5]
        ax.clear()
        ax.plot(self.heading_data, label="Heading", color=self.heading_color)
        ax.set_title("Heading (degrees)", color=self.fg)
        ax.set_ylim(140, 210)
        ax.legend()
        self.canvases[5].draw()

        # Log data to CSV
        self.log_data_point(temperature, humidity, acc_x, acc_y, acc_z, 
                           gyro_x, gyro_y, gyro_z, heading, pressure, altitude)

        self.root.after(1000, self.update_sensor_data)  # Update sensor data every 1000ms

    def log_data_point(self, temperature, humidity, acc_x, acc_y, acc_z, 
                       gyro_x, gyro_y, gyro_z, heading, pressure, altitude):
        # Log every data point to CSV file
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        with open(self.log_filename, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                timestamp, temperature, humidity, self.lat, self.long, 
                self.num_satellites, self.hdop, acc_x, acc_y, acc_z,
                gyro_x, gyro_y, gyro_z, heading, pressure, altitude
            ])

    def plot_sensor(self, index, title, data):
        ax = self.axes[index]
        ax.clear()
        for axis in ['X', 'Y', 'Z']:
            ax.plot(data[axis], label=axis, color=self.line_colors[axis])
        ax.set_title(title, color=self.fg)
        ax.legend()
        self.canvases[index].draw()

    def snapshot(self):
        # Save current frame as a snapshot
        if self.frame is not None:
            cv2.imwrite(f"snapshot_{self.snapshot_count}.jpg", self.frame)
            self.camera_status.config(text=f"Snapshot saved as snapshot_{self.snapshot_count}.jpg")
            self.snapshot_count += 1

    def toggle_theme(self):
        self.set_theme()
        # Redraw all charts
        for ax in self.axes:
            ax.clear()
        self.update_sensor_data()

    def save_plots(self):
        save_dir = filedialog.askdirectory()
        if not save_dir:
            return

        # Save current camera frame if available
        if self.frame is not None:
            cv2.imwrite(os.path.join(save_dir, "camera_capture.jpg"), self.frame)

        # Save plots
        for i, fig in enumerate(self.figures):
            fig.savefig(os.path.join(save_dir, f"plot_{i+1}.png"), dpi=150)
        
        print(f"Camera capture and plots saved to: {save_dir}")

    def save_csv(self):
        save_path = filedialog.asksaveasfilename(defaultextension=".csv", 
                                              filetypes=[("CSV files", "*.csv")])
        if not save_path:
            return
            
        # Copy the current log file to the selected location
        with open(self.log_filename, "r") as source:
            with open(save_path, "w") as dest:
                dest.write(source.read())
                
        print(f"Sensor data saved to: {save_path}")

    def on_closing(self):
        if self.stream:
            self.stream.release()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = CombinedDashboard(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()