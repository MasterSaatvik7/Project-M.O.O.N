import tkinter as tk
from tkinter import ttk, filedialog
import serial
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
import numpy as np
import csv
import os

matplotlib.use("TkAgg")

class SensorDashboard:
    def __init__(self, root):

        # # Example setup (adjust COM port and baudrate as needed)
        # self.serial_port = serial.Serial('COM3', 9600, timeout=1)

        try:
            self.serial_port = serial.Serial('COM3', 9600, timeout=1)
        except serial.SerialException as e:
            print("Serial port error:", e)
            self.serial_port = None


        self.dark_mode = tk.BooleanVar(value=True)
        self.root = root
        self.root.title("Sensor Data Dashboard")
        self.root.geometry("1000x700")

        self.input_frame = tk.Frame(root)
        self.input_frame.grid(row=2, column=0, columnspan=2, sticky="ew")
        
        self.set_theme()

        # Initialize data buffers
        self.time_data = list(range(20))
        self.acc_data = {'X': [], 'Y': [], 'Z': []}
        self.gyro_data = {'X': [], 'Y': [], 'Z': []}
        self.mag_data = {'X': [], 'Y': [], 'Z': []}
        self.temp_data = []

        self.create_widgets()
        self.update_data()

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
        self.input_frame.configure(bg=self.bg)
        for widget in self.input_frame.winfo_children():
            if isinstance(widget, tk.Button) or isinstance(widget, tk.Checkbutton) or isinstance(widget, tk.Entry):
                widget.configure(bg=self.button_bg if isinstance(widget, tk.Button) else self.bg,
                                fg=self.fg, insertbackground=self.fg if isinstance(widget, tk.Entry) else None)


    def create_widgets(self):
        self.figures, self.axes, self.canvases = [], [], []

        for i in range(4):
            frame = tk.Frame(self.root, bg=self.bg)
            frame.grid(row=i // 2, column=i % 2, sticky="nsew", padx=5, pady=5)
            self.root.grid_rowconfigure(i // 2, weight=1)
            self.root.grid_columnconfigure(i % 2, weight=1)

            fig, ax = plt.subplots(figsize=(5, 3))
            canvas = FigureCanvasTkAgg(fig, master=frame)
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

            self.figures.append(fig)
            self.axes.append(ax)
            self.canvases.append(canvas)

        self.input_frame = tk.Frame(self.root, bg=self.bg)
        self.input_frame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=10)

        self.input_var = tk.StringVar()
        self.input_box = tk.Entry(self.input_frame, textvariable=self.input_var, font=("Arial", 12), width=50)
        self.input_box.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        self.send_button = tk.Button(self.input_frame, text="Send", command=self.send_command, bg="#4CAF50", fg="white", font=("Arial", 12))
        self.send_button.grid(row=0, column=1, padx=5)

        motion_cmds = ["Forward", "Left", "Right", "U-Turn"]
        for idx, cmd in enumerate(motion_cmds):
            btn = tk.Button(self.input_frame, text=cmd, width=10, font=("Arial", 10),
                            command=lambda c=cmd: self.set_input_command(c), bg=self.button_bg, fg=self.fg)
            btn.grid(row=1, column=idx, padx=5, pady=5)

        dark_mode_btn = tk.Checkbutton(self.input_frame, text="Dark Mode", variable=self.dark_mode, command=self.toggle_theme,
                                       bg=self.bg, fg=self.fg, selectcolor=self.bg, font=("Arial", 10))
        dark_mode_btn.grid(row=2, column=0, sticky="w", padx=5, pady=5)

        save_btn = tk.Button(self.input_frame, text="Save Plot & CSV", command=self.save_data,
                             bg="#2196F3", fg="white", font=("Arial", 10))
        save_btn.grid(row=2, column=1, padx=5, pady=5, sticky="e")

    def set_input_command(self, cmd):
        self.input_var.set(cmd)

    def toggle_theme(self):
        self.set_theme()
        for ax in self.axes:
            ax.clear()
        self.update_data()

    def get_serial_data(self):
        try:
            if self.serial_port and self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8').strip()
                parts = [float(x) for x in line.strip("[]").split(",")]
                if len(parts) == 10:
                    gyro = parts[0:3]
                    acc = parts[3:6]
                    mag = parts[6:9]
                    temp = parts[9]
                    return gyro, acc, mag, temp
        except Exception as e:
            print("Serial Read Error:", e)
        return None, None, None, None


    def update_data(self):
        gyro, acc, mag, temp = self.get_serial_data()

        if all(v is not None for v in [gyro, acc, mag, temp]):
            # Append new values
            for buffer, new_vals in zip(
                [self.gyro_data, self.acc_data, self.mag_data],
                [gyro, acc, mag]
            ):
                for axis, value in zip(['X', 'Y', 'Z'], new_vals):
                    buffer[axis].append(value)
                    if len(buffer[axis]) > 20:
                        buffer[axis].pop(0)

            self.temp_data.append(temp)
            if len(self.temp_data) > 20:
                self.temp_data.pop(0)

            # Update Plots
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

        # Call this function again after 1s
        self.root.after(1000, self.update_data)


    def plot_sensor(self, index, title, data):
        ax = self.axes[index]
        ax.clear()
        for axis in ['X', 'Y', 'Z']:
            ax.plot(data[axis], label=axis, color=self.line_colors[axis])
        ax.set_title(title, color=self.fg)
        ax.legend()
        self.canvases[index].draw()

    def send_command(self):
        print("Sent Command:", self.input_var.get())

    def save_data(self):
        save_dir = filedialog.askdirectory()
        if not save_dir:
            return

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

        print("Plots and CSV saved to:", save_dir)

if __name__ == "__main__":
    root = tk.Tk()
    app = SensorDashboard(root)
    root.mainloop()
