# client.py - Run this on your laptop
import tkinter as tk
import cv2
import numpy as np
from PIL import Image, ImageTk
import threading
import socket

class VideoStreamGUI:
    def __init__(self, window, window_title, video_source):
        self.window = window
        self.window.title(window_title)
        self.video_source = video_source
        
        # Create a canvas that can fit the video source
        self.canvas = tk.Canvas(window, width=640, height=480)
        self.canvas.pack()
        
        # Button to take snapshots
        self.btn_snapshot = tk.Button(window, text="Snapshot", width=50, command=self.snapshot)
        self.btn_snapshot.pack(anchor=tk.CENTER, pady=10)
        
        # Status label
        self.status_label = tk.Label(window, text="Connecting to stream...", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Initialize snapshot counter
        self.snapshot_count = 1
        
        # After it is called once, the update method will be automatically called every delay milliseconds
        self.delay = 15
        self.update()
        
        self.window.mainloop()
    
    def snapshot(self):
        # Save current frame as a snapshot
        if hasattr(self, 'frame'):
            cv2.imwrite(f"snapshot_{self.snapshot_count}.jpg", self.frame)
            self.status_label.config(text=f"Snapshot saved as snapshot_{self.snapshot_count}.jpg")
            self.snapshot_count += 1
    
    def update(self):
        # Get a frame from the video source
        try:
            if not hasattr(self, 'stream'):
                self.stream = cv2.VideoCapture(self.video_source)
                self.status_label.config(text="Connected to stream")
            
            ret, self.frame = self.stream.read()
            if ret:
                self.photo = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)))
                self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
            else:
                self.status_label.config(text="Failed to get frame from stream")
        except Exception as e:
            self.status_label.config(text=f"Error: {str(e)}")
        
        self.window.after(self.delay, self.update)

# Create a window and pass it to the VideoStreamGUI class
if __name__ == "__main__":
    # Replace with your Raspberry Pi's IP address
    # pi_ip = input("Enter Raspberry Pi IP address: ")
    # video_source = f"http://{pi_ip}:8000/stream.mjpg"
    # video_source = "http://raspberrypi.local:8000/stream.mjpg"

    try:
        pi_hostname = "raspberrypi.local"
        pi_ip = socket.gethostbyname(pi_hostname)
        print(f"Discovered Raspberry Pi at {pi_ip}")
        video_source = f"http://{pi_ip}:8000/stream.mjpg"
    except socket.gaierror:
        print("Could not find Raspberry Pi on the network.")
        exit()

    root = tk.Tk()
    app = VideoStreamGUI(root, "Raspberry Pi Camera Stream", video_source)

