# import tkinter as tk
# import serial
# import threading

# # --- Serial Config ---
# # Try COM4 first. If it doesn't work, try COM5
# BLUETOOTH_PORT = "COM4"
# BAUD_RATE = 9600

# # --- Try opening the Bluetooth serial connection ---
# try:
#     ser = serial.Serial(BLUETOOTH_PORT, BAUD_RATE, timeout=1)
# except serial.SerialException:
#     print(f"Could not open {BLUETOOTH_PORT}. Trying COM5...")
#     try:
#         BLUETOOTH_PORT = "COM5"
#         ser = serial.Serial(BLUETOOTH_PORT, BAUD_RATE, timeout=1)
#     except serial.SerialException:
#         print("Failed to connect to Bluetooth device on COM4 or COM5.")
#         ser = None

# # --- GUI ---
# root = tk.Tk()
# root.title("Bluetooth Data Viewer")

# data_label = tk.Label(root, text="Waiting for data...", font=("Helvetica", 16))
# data_label.pack(padx=20, pady=20)

# # Function to update label with data from Bluetooth
# def read_bluetooth():
#     if ser and ser.in_waiting > 0:
#         try:
#             line = ser.readline().decode('utf-8').strip()
#             if line:
#                 data_label.config(text=line)
#         except Exception as e:
#             data_label.config(text=f"Error: {str(e)}")

#     root.after(500, read_bluetooth)  # Repeat every 500ms

# # Start the reading loop
# root.after(1000, read_bluetooth)

# # Start GUI main loop in the main thread
# root.mainloop()


# import tkinter as tk
# import serial
# import threading

# # Set COM port and baud rate
# PORT = "COM4"  # or COM5 if needed
# BAUD = 9600

# # Attempt serial connection
# try:
#     ser = serial.Serial(PORT, BAUD, timeout=1)
# except:
#     print(f"Failed to connect to {PORT}")
#     ser = None

# # --- GUI Setup ---
# root = tk.Tk()
# root.title("Bluetooth Terminal")

# # Output box
# output_label = tk.Label(root, text="Bluetooth Output", font=("Helvetica", 14))
# output_label.pack(pady=5)

# output_text = tk.Text(root, height=10, width=50)
# output_text.pack(pady=5)

# # Input field
# input_entry = tk.Entry(root, width=40, font=("Helvetica", 12))
# input_entry.pack(pady=5)

# # Send button
# def send_message():
#     msg = input_entry.get().strip()
#     if msg and ser:
#         ser.write((msg + "\n").encode())  # send with newline
#         input_entry.delete(0, tk.END)

# send_button = tk.Button(root, text="Send", command=send_message)
# send_button.pack(pady=5)

# # Function to read from Bluetooth
# def read_bluetooth():
#     if ser and ser.in_waiting > 0:
#         try:
#             line = ser.readline().decode('utf-8').strip()
#             if line:
#                 output_text.insert(tk.END, line + "\n")
#                 output_text.see(tk.END)
#         except Exception as e:
#             output_text.insert(tk.END, f"Error: {e}\n")

#     root.after(500, read_bluetooth)  # schedule again

# # Start read loop
# root.after(1000, read_bluetooth)

# # Run main loop
# root.mainloop()


import tkinter as tk
from tkinter import scrolledtext, messagebox
import serial
import threading
import time
from datetime import datetime

class BluetoothController:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Bluetooth Controller")
        self.root.geometry("600x500")
        self.root.resizable(True, True)
        
        # Serial connection
        self.ser = None
        self.connected = False
        self.stop_thread = False
        self.thread = None
        
        # Create GUI elements
        self.create_widgets()
        
        # Try to connect automatically
        self.connect_bluetooth()
        
    def create_widgets(self):
        # Frame for connection controls
        connection_frame = tk.LabelFrame(self.root, text="Connection")
        connection_frame.pack(fill="x", padx=10, pady=5)
        
        self.port_var = tk.StringVar(value="COM4")
        tk.Label(connection_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        tk.Entry(connection_frame, textvariable=self.port_var, width=10).grid(row=0, column=1, padx=5, pady=5)
        
        self.baud_var = tk.StringVar(value="9600")
        tk.Label(connection_frame, text="Baud:").grid(row=0, column=2, padx=5, pady=5)
        tk.Entry(connection_frame, textvariable=self.baud_var, width=10).grid(row=0, column=3, padx=5, pady=5)
        
        self.connect_button = tk.Button(connection_frame, text="Connect", command=self.connect_bluetooth)
        self.connect_button.grid(row=0, column=4, padx=5, pady=5)
        
        self.disconnect_button = tk.Button(connection_frame, text="Disconnect", command=self.disconnect_bluetooth, state=tk.DISABLED)
        self.disconnect_button.grid(row=0, column=5, padx=5, pady=5)
        
        # Status indicator
        self.status_var = tk.StringVar(value="Disconnected")
        self.status_label = tk.Label(connection_frame, textvariable=self.status_var, fg="red")
        self.status_label.grid(row=0, column=6, padx=5, pady=5)
        
        # Frame for message display
        receive_frame = tk.LabelFrame(self.root, text="Received Messages")
        receive_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Scrolled text widget for displaying received messages
        self.receive_text = scrolledtext.ScrolledText(receive_frame, wrap=tk.WORD, height=15)
        self.receive_text.pack(fill="both", expand=True, padx=5, pady=5)
        self.receive_text.config(state=tk.DISABLED)
        
        # Frame for sending messages
        send_frame = tk.LabelFrame(self.root, text="Send Message")
        send_frame.pack(fill="x", padx=10, pady=5)
        
        # Entry for typing messages
        self.message_var = tk.StringVar()
        self.message_entry = tk.Entry(send_frame, textvariable=self.message_var, width=50)
        self.message_entry.grid(row=0, column=0, padx=5, pady=5)
        self.message_entry.bind("<Return>", lambda event: self.send_message())
        
        # Send button
        self.send_button = tk.Button(send_frame, text="Send", command=self.send_message, state=tk.DISABLED)
        self.send_button.grid(row=0, column=1, padx=5, pady=5)
        
        # Clear button
        tk.Button(send_frame, text="Clear Log", command=self.clear_log).grid(row=0, column=2, padx=5, pady=5)
    
    def connect_bluetooth(self):
        # Clear any existing connection
        if self.connected:
            self.disconnect_bluetooth()
        
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self.connected = True
            self.status_var.set(f"Connected to {port}")
            self.status_label.config(fg="green")
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            self.send_button.config(state=tk.NORMAL)
            
            # Start reading thread
            self.stop_thread = False
            self.thread = threading.Thread(target=self.read_from_bluetooth)
            self.thread.daemon = True
            self.thread.start()
            
            self.log_message("System", f"Connected to {port} at {baud} baud")
            
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Could not connect to {port}: {str(e)}")
            
            # Try alternative port
            alt_port = "COM5" if port == "COM4" else "COM4"
            try:
                self.port_var.set(alt_port)
                self.ser = serial.Serial(alt_port, baud, timeout=0.5)
                self.connected = True
                self.status_var.set(f"Connected to {alt_port}")
                self.status_label.config(fg="green")
                self.connect_button.config(state=tk.DISABLED)
                self.disconnect_button.config(state=tk.NORMAL)
                self.send_button.config(state=tk.NORMAL)
                
                # Start reading thread
                self.stop_thread = False
                self.thread = threading.Thread(target=self.read_from_bluetooth)
                self.thread.daemon = True
                self.thread.start()
                
                self.log_message("System", f"Connected to alternative port {alt_port} at {baud} baud")
            except serial.SerialException:
                messagebox.showerror("Connection Error", f"Could not connect to alternative port {alt_port}")
    
    def disconnect_bluetooth(self):
        if self.connected:
            self.stop_thread = True
            if self.thread:
                self.thread.join(1.0)  # Wait for thread to terminate
            
            if self.ser:
                self.ser.close()
            
            self.connected = False
            self.status_var.set("Disconnected")
            self.status_label.config(fg="red")
            self.connect_button.config(state=tk.NORMAL)
            self.disconnect_button.config(state=tk.DISABLED)
            self.send_button.config(state=tk.DISABLED)
            
            self.log_message("System", "Disconnected from device")
    
    def read_from_bluetooth(self):
        while not self.stop_thread and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8').strip()
                    if data:
                        self.log_message("Arduino", data)
            except Exception as e:
                self.log_message("Error", f"Reading error: {str(e)}")
                break
            time.sleep(0.1)
    
    def send_message(self):
        if not self.connected or not self.ser:
            messagebox.showerror("Error", "Not connected to device")
            return
        
        message = self.message_var.get().strip()
        if not message:
            return
        
        try:
            self.ser.write((message + '\n').encode())
            self.log_message("You", message)
            self.message_var.set("")  # Clear input field
        except Exception as e:
            self.log_message("Error", f"Sending error: {str(e)}")
    
    def log_message(self, sender, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {sender}: {message}\n"
        
        self.receive_text.config(state=tk.NORMAL)
        self.receive_text.insert(tk.END, formatted_message)
        self.receive_text.see(tk.END)  # Auto-scroll to the end
        self.receive_text.config(state=tk.DISABLED)
    
    def clear_log(self):
        self.receive_text.config(state=tk.NORMAL)
        self.receive_text.delete(1.0, tk.END)
        self.receive_text.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = BluetoothController(root)
    root.mainloop()