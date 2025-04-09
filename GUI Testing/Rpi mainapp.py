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

matplotlib.use("TkAgg")

class CombinedDashboard:
    def __init__(self, root):
        self.dark_mode = tk.BooleanVar(value=False)
        self.root = root
        self.root.title("Combined Camera and Sensor Dashboard")
        self.root.geometry("1400x700")  # Increased width to accommodate both panels

        # Main frames for layout
        self.left_frame = tk.Frame(root)
        self.left_frame.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=5, pady=5)
        
        self.right_frame = tk.Frame(root)
        self.right_frame.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=5, pady=5)
        
        self.input_frame = tk.Frame(root)
        self.input_frame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        
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

        # Create all widgets
        self.create_camera_widgets()
        self.create_sensor_widgets()
        self.create_control_widgets()

        # Start data updates
        self.update_camera()
        self.update_sensor_data()

    def set_theme(self):
        if self.dark_mode.get():
            plt.style.use("dark_background")
            self.bg = "#2E2E2E"
            self.fg = "white"
            self.button_bg = "#444"
            self.line_colors = {'X': 'cyan', 'Y': 'magenta', 'Z': 'yellow'}
            self.temp_color = 'orange'
        else:
            plt.style.use("default")
            self.bg = "white"
            self.fg = "black"
            self.button_bg = "#ddd"
            self.line_colors = {'X': 'blue', 'Y': 'red', 'Z': 'green'}
            self.temp_color = 'darkorange'

        self.root.configure(bg=self.bg)
        self.left_frame.configure(bg=self.bg)
        self.right_frame.configure(bg=self.bg)
        self.input_frame.configure(bg=self.bg)
        
        # Update all child widgets
        for frame in [self.left_frame, self.right_frame, self.input_frame]:
            for widget in frame.winfo_children():
                if isinstance(widget, tk.Button) or isinstance(widget, tk.Checkbutton) or isinstance(widget, tk.Entry):  
                    widget.configure(bg=self.button_bg if isinstance(widget, tk.Button) else self.bg,
                                    fg=self.fg, insertbackground=self.fg if isinstance(widget, tk.Entry) else None)

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

        # Create 2x2 grid for sensor charts in right frame
        for i in range(4):
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

    def create_control_widgets(self):
        # Command input
        self.input_var = tk.StringVar()
        self.input_box = tk.Entry(self.input_frame, textvariable=self.input_var, 
                                 font=("Arial", 12), width=50, bg=self.bg, fg=self.fg)
        self.input_box.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        self.send_button = tk.Button(self.input_frame, text="Send", command=self.send_command, 
                                    bg="#4CAF50", fg="white", font=("Arial", 12))
        self.send_button.grid(row=0, column=1, padx=5)

        # Quick command buttons
        motion_cmds = ["Forward", "Left", "Right", "U-Turn"]
        for idx, cmd in enumerate(motion_cmds):
            btn = tk.Button(self.input_frame, text=cmd, width=10, font=("Arial", 10),
                           command=lambda c=cmd: self.set_input_command(c), bg=self.button_bg, fg=self.fg)
            btn.grid(row=1, column=idx, padx=5, pady=5)

        # Dark mode toggle
        dark_mode_btn = tk.Checkbutton(self.input_frame, text="Dark Mode", variable=self.dark_mode, 
                                       command=self.toggle_theme, bg=self.bg, fg=self.fg, 
                                       selectcolor=self.bg, font=("Arial", 10))
        dark_mode_btn.grid(row=2, column=0, sticky="w", padx=5, pady=5)

        # Save data button
        save_btn = tk.Button(self.input_frame, text="Save Plot & CSV", command=self.save_data,
                            bg="#2196F3", fg="white", font=("Arial", 10))
        save_btn.grid(row=2, column=1, padx=5, pady=5, sticky="e")

    def update_camera(self):
        # Get a frame from the video source
        try:
            if not self.stream:
                self.stream = cv2.VideoCapture(self.video_source)
                self.camera_status.config(text=f"Connected to camera at {self.video_source}")
            
            ret, self.frame = self.stream.read()
            if ret:
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
        gyro_x, gyro_y, gyro_z = random.uniform(0, 90), random.uniform(0, 90), random.uniform(0, 90)
        mag_x, mag_y, mag_z = random.uniform(25, 27), random.uniform(25, 27), random.uniform(25, 27)
        temperature = random.uniform(25, 27)

        # Keep last 20 values
        for buffer, new_vals in zip(
            [self.acc_data, self.gyro_data, self.mag_data],
            [[acc_x, acc_y, acc_z], [gyro_x, gyro_y, gyro_z], [mag_x, mag_y, mag_z]]
        ):
            for k, v in zip(['X', 'Y', 'Z'], new_vals):
                buffer[k].append(v)
                if len(buffer[k]) > 20:
                    buffer[k].pop(0)

        self.temp_data.append(temperature)
        if len(self.temp_data) > 20:
            self.temp_data.pop(0)

        # Plot Accelerometer
        self.plot_sensor(0, "Accelerometer (m/s²)", self.acc_data)
        self.plot_sensor(1, "Gyroscope (°)", self.gyro_data)
        self.plot_sensor(2, "Magnetometer (μT)", self.mag_data)

        # Plot Temperature
        ax = self.axes[3]
        ax.clear()
        ax.plot(self.temp_data, label="Temp", color=self.temp_color)
        ax.set_title("Temperature (°C)", color=self.fg)
        ax.set_ylim(20, 30)
        ax.legend()
        self.canvases[3].draw()

        self.root.after(1000, self.update_sensor_data)  # Update sensor data every 1000ms

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

    def set_input_command(self, cmd):
        self.input_var.set(cmd)

    def send_command(self):
        command = self.input_var.get()
        print("Sent Command:", command)
        # In a real application, you would send this command to the Raspberry Pi
        self.input_var.set("")  # Clear input field

    def toggle_theme(self):
        self.set_theme()
        # Redraw all charts
        for ax in self.axes:
            ax.clear()
        self.update_sensor_data()

    def save_data(self):
        save_dir = filedialog.askdirectory()
        if not save_dir:
            return

        # Save current camera frame if available
        if self.frame is not None:
            cv2.imwrite(os.path.join(save_dir, "camera_capture.jpg"), self.frame)

        # Save plots
        for i, fig in enumerate(self.figures):
            fig.savefig(os.path.join(save_dir, f"plot_{i+1}.png"), dpi=150)

        # Save CSV
        csv_path = os.path.join(save_dir, "sensor_readings.csv")
        with open(csv_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            headers = ['Time'] + [f"Acc_{i}" for i in ['X', 'Y', 'Z']] + \
                      [f"Gyro_{i}" for i in ['X', 'Y', 'Z']] + \
                      [f"Mag_{i}" for i in ['X', 'Y', 'Z']] + ['Temperature']
            writer.writerow(headers)

            for i in range(len(self.temp_data)):
                row = [i]
                row += [self.acc_data[k][i] if i < len(self.acc_data[k]) else '' for k in ['X', 'Y', 'Z']]
                row += [self.gyro_data[k][i] if i < len(self.gyro_data[k]) else '' for k in ['X', 'Y', 'Z']]
                row += [self.mag_data[k][i] if i < len(self.mag_data[k]) else '' for k in ['X', 'Y', 'Z']]
                row += [self.temp_data[i]]
                writer.writerow(row)

        print(f"Camera capture, plots, and CSV saved to: {save_dir}")

    def on_closing(self):
        if self.stream:
            self.stream.release()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = CombinedDashboard(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()