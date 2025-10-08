import tkinter as tk
#from tkinter import ttk
import pc_robot_gui_frames
import socket, threading, time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from queue import Queue #https://www.troyfawkes.com/learn-python-multithreading-queues-basics/


PORT_LISTEN = 5006      # Pico → PC
SEND_PORT = 5005        # PC → Pico
PICO_IP = "192.168.1.138"   # set your Pico’s IP here
#PICO_IP = "192.168.5.187"   # bib Neude, via rpi HaHaHo


# -----------------------------
# Message abstraction 
# -----------------------------
class Message:
    def __init__(self, command_raw):
        parts = command_raw.strip().split()
        self.target = parts[0].strip().upper()
        self.command = parts[1].strip().lower()
        if len(parts) > 2:  # Join the rest and split by comma, stripping extra spaces            
            arg_str = " ".join(parts[2:])
            self.args = [a.strip().lower() for a in arg_str.split(",") if a.strip()]
        else:
            self.args = []

    def __repr__(self):
        return f"{self.target} {self.command} " + ",".join(str(x) for x in self.args)
    
    
# -----------------------------
# Run GUI
# -----------------------------
def main():
    app = RobotGUI()
    #app.protocol("WM_DELETE_WINDOW", app.stop)
    app.mainloop()
    app.close()


# -----------------------------
# Main GUI Object
# -----------------------------
class RobotGUI(tk.Tk):            
    def __init__(self):
        super().__init__()  #see: https://www.youtube.com/watch?v=X5yyKZpZ4vU, see also frame constructors; "_event=None"
        self.title("Robot Control GUI")

        # Networking
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)        
        self.pose_data = []  # [ ms,(x, y) ]
        self.running = True

        # State
        self.current_mode = "IDLE"
        self.speed = 0.2

        # --- GUI Layout ---        
        self.frame_console = pc_robot_gui_frames.ConsoleFrame(self)
        self.frame_console.pack(expand=True, fill="both", side="left", padx=10, pady=10)

        self.frame_log = pc_robot_gui_frames.LoggingFrame(self)
        self.frame_log.pack(expand=False, fill="y", side="right", padx=10, pady=10)
        
        self.frame_buttons = pc_robot_gui_frames.ButtonFrame(self)
        self.frame_buttons.pack(expand=False, fill="y", padx=0, pady=0)
        
        #self.frame_robot = pc_robot_gui_frames.RobotFrame(self)
        #self.frame_robot.pack(expand=False, fill="y", side="right", padx=5, pady=10)

        #self.frame_drive = pc_robot_gui_frames.DriveFrame(self)
        #self.frame_drive.pack(expand=False, fill="y", side="right", padx=5, pady=10)

        # Setup plot
# =============================================================================
#         self.fig, self.ax = plt.subplots()
#         self.ax.set_title("Robot Path")
#         self.ax.set_xlabel("X [m]")
#         self.ax.set_ylabel("Y [m]")
#         self.path_line, = self.ax.plot([], [], "b-", lw=2)
# 
#         self.ani = FuncAnimation(self.fig, self.update_plot, interval=200, blit=False)
#         threading.Thread(target=plt.show(block=False), daemon=True).start()
# =============================================================================
        
        # Create thread safe que        
        self.q = Queue(maxsize=100)   
        # Start robot listener
        threading.Thread(target=self.listen_to_robot, daemon=True).start()  
        # schedule next poll
        self.after(100, self.eventhandler)
        
        
    def eventhandler(self):
        while not self.q.empty():
            response = self.q.get()
            if response.startswith("LOG"):
                self.frame_log.add_log(response[4:])

            elif response.startswith("MSG"):
                self.frame_console.add_history("← "+response[4:])            
            else:
                self.frame_console.add_history("? "+response)
            self.q.task_done()
        # schedule next poll
        self.after(50, self.eventhandler)


    # --- Networking ---
    def listen_to_robot(self): #self running thread:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", PORT_LISTEN))           
        while self.running:
            try:
                data, addr = sock.recvfrom(256)
                msg = str(data.decode())
                print(f"sock got message: {msg}.")
                self.q.put(msg)
                time.sleep(0.05)    #generate some time (0.05) before reading next, so queing is not blocked.
            except Exception:
                print (Exception)
                print("? " + msg + " " + str(addr))
                

    def send_command(self, cmd):
        try:
            self.cmd_sock.sendto(cmd.encode(), (PICO_IP, SEND_PORT))
            self.frame_console.add_history("→ " + cmd)
        except OSError:
            pass        
    
    # --- GUI Events ---
    def clear_listboxes(self):
        self.frame_console.clear_list()
        self.frame_log.clear_list()

    def addToClipBoard(self):
        my_list = self.frame_log.log_list.get(0,tk.END)
        self.clipboard_clear()
        self.clipboard_append("\n".join(my_list))
        self.update() # Now it stays on the clipboard after the window is closed

    def on_speed_change(self, val):
        self.speed = float(val)

    def send_velocity(self, v, w):
        """Send velocity command only in manual mode"""
        if self.current_mode == "manual":
            msg = f"vel,{v:.2f},{w:.1f}"
            self.send_command(msg)

    def send_entry_command(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.send_command(cmd)
            self.cmd_entry.delete(0, tk.END)
        
    def on_key_press(self, event):
        if self.current_state != "keyboard":
            return
        if event.keysym == "Up":
            self.send_velocity(self.speed, 0)
        elif event.keysym == "Down":
            self.send_velocity(-self.speed, 0)
        elif event.keysym == "Left":
            self.send_velocity(0, 45)
        elif event.keysym == "Right":
            self.send_velocity(0, -45)

    # --- Plot ---
    def update_plot(self, frame):
        if self.pose_data:
            xs, ys = zip(*self.pose_data)
            self.path_line.set_data(xs, ys)
            self.ax.relim()
            self.ax.autoscale_view()
        return self.path_line,

    # --- Cleanup ---
    def close(self):        
        self.running = False    # kill listening threat
        #self.ani.close()       # close plot animation...
        self.quit()




# --- Run GUI ---
if __name__ == "__main__":
    main()