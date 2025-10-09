# -*- coding: utf-8 -*-
"""
Created on Tue Sep 23 10:51:09 2025

@author: Edge

This module should be used within pc_robot_gui.py.
"""

import tkinter as tk
from tkinter import ttk


# == .pack()  ====================================================================
# anchor: Denotes where the packer is to place each slave in its parcel.
# expand: Boolean, 0 or 1.
# fill: 'x', 'y', 'both', 'none'.
# ipadx and ipady: A distance - designating internal padding on each side of the slave widget.
# padx and pady:   A distance - designating external padding on each side of the slave widget.
# side: 'left', 'right', 'top', 'bottom'.
# =============================================================================


# ----------------------------------
# --- GUI Objects: Console frame ---
# ----------------------------------
class ConsoleFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self._parent = parent
        # --- GUI Frame Layout ---
        self.frame_input = ttk.LabelFrame(self, text="Console:")
        self.frame_input.pack(expand=True, fill="both", padx=0, pady=0)
        
        self.history_list = tk.Listbox(self.frame_input, height=6)
        self.history_list.pack(side="top", fill="both", expand=True, padx=5, pady=5)        

        self.bttn_send = ttk.Button(self.frame_input, text="Clr", width = 3, command=self._parent.clear_listboxes)
        self.bttn_send.pack(side="left", padx=5)

        cmdlist= ["DRV to_pose ", "DRV move_to ", "DRV heading_to ", "DRV rotate_to ", "DRV set_velocity ", "DRV set_wheel_speed ", "DRV set_pwm ",
                  "-----", "RBT run_square 500,CCW,250", "RBT add_cmd ", "RBT stop_cmds ", "RBT start_cmds ", "RBT clear_cmds ",
                  "-----", "DRV set_pose ", "DRV calibrate_drive ", "DRV set_rtc_update_period ", "DRV set_motor_pwm_range ", "DRV set_motor_pid ", "DRV set_goal_seeker ", "DRV set_goal_tolerances "]
        self.combo_cmd = ttk.Combobox(self.frame_input, values=cmdlist, width=50)
        self.combo_cmd.pack(side="left", expand=True, fill="x", padx=5)
        
        #self.text_input = ttk.Entry(self.frame_input, width=50)
        #self.text_input.pack(side="left", expand=True, fill="x", padx=5, pady=5)
        
        self.bttn_send = ttk.Button(self.frame_input, text="Send", command=self.send_command)
        self.bttn_send.pack(side="right", padx=5)

        
    # --- GUI frame functions ---
    def send_command(self):
        #cmd = self.text_input.get().strip()
        cmd = self.combo_cmd.get().strip()
        self._parent.send_command(cmd)
                
    def add_history(self, cmd):
        self.history_list.insert(tk.END, cmd)
        self.history_list.yview(tk.END)
    
    def clear_list(self):
        self.history_list.delete(0,tk.END)


# ----------------------------------
# --- GUI Objects: Logging frame ---
# ----------------------------------
class LoggingFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self._parent = parent
        # --- GUI Frame Layout ---
        self.frame_input = ttk.LabelFrame(self, text="Log:")
        self.frame_input.pack(expand=True, fill="both", padx=0, pady=0)

        self.log_list = tk.Listbox(self.frame_input, height=6, width=40)
        self.log_list.pack(side="top", fill="both", expand=True, padx=5, pady=5) 
        
        self.bttn_send = ttk.Button(self.frame_input, text="Clipboard", command=self.addToClipBoard2)
        self.bttn_send.pack(side="bottom", padx=5)
    
    # --- GUI frame functions ---
    def add_log(self, cmd):        
        self.log_list.insert(tk.END, cmd)
        self.log_list.yview(tk.END)

    def clear_list(self):
        self.log_list.delete(0,tk.END)
    
    def addToClipBoard2(self):
        self._parent.addToClipBoard()

        
        
# ----------------------------------
# --- GUI Objects: BUTTONS frame ---       
# ---------------------------------- 
class ButtonFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self._parent = parent
        # --- GUI Frame Layout ---        
        self.frame_robot = RobotFrame(self)
        self.frame_robot.pack(expand=False, fill="none", side="top", padx=5, pady=10)

        self.frame_drive = DriveFrame(self)
        self.frame_drive.pack(expand=False, fill="none", side="bottom", padx=5, pady=5)

    # --- GUI frame functions ---
    def send_message(self, cmd):
        self._parent.send_command(cmd)


# ----------------------------------
# --- GUI Objects: ROBOT frame ---   
# ----------------------------------     
class RobotFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self._parent = parent
        # --- GUI Frame Layout ---
        self.frame_input = ttk.LabelFrame(self, text="Robot:")
        self.frame_input.pack(expand=False, fill="y", padx=0, pady=0)

        self.bttn_ping = ttk.Button(self.frame_input, text="Ping", command=lambda: self.send_message("RBT ping"))
        self.bttn_ping.pack(side="top", padx=10, pady=10)

        self.bttn_ping = ttk.Button(self.frame_input, text="Info", command=lambda: self.send_message("RBT info"))
        self.bttn_ping.pack(side="top", padx=10, pady=10)

        self.bttn_stop = ttk.Button(self.frame_input, text="STOP", command=lambda: self.send_message("RBT stop"))
        self.bttn_stop.pack(side="top", padx=10, ipady=30)

        self.bttn_close = ttk.Button(self.frame_input, text="Close", command=lambda: self.send_message("RBT close"))
        self.bttn_close.pack(side="top", padx=10, pady=10)

    # --- GUI frame functions ---    
    def send_message(self, cmd):
        self._parent.send_message(cmd)


# ----------------------------------
# --- GUI Objects: Control frame ---        
# ----------------------------------
class DriveFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self._parent = parent
        # --- GUI Frame Layout ---
        self.frame_input = ttk.LabelFrame(self, text="Drive:")
        self.frame_input.pack(expand=False, fill="y", padx=0, pady=0)

        self.bttn_status = ttk.Button(self.frame_input, text="Status", command=lambda: self.send_message("DRV status"))
        self.bttn_status.pack(side="bottom", padx=10, pady=10)
        
        self.bttn_info = ttk.Button(self.frame_input, text="Info", command=lambda: self.send_message("DRV info"))
        self.bttn_info.pack(side="bottom", padx=10, pady=10)
        
        self.bttn_move = ttk.Button(self.frame_input, text="Reset", command=lambda: self.send_message("DRV reset"))
        self.bttn_move.pack(side="bottom", padx=10, pady=10)

    # --- GUI frame functions ---
    def send_message(self, cmd):
        self._parent.send_message(cmd)

        
# =============================================================================
#         # Speed slider
#         slider_frame = ttk.LabelFrame(root, text="Speed (manual only)")
#         slider_frame.pack(fill="x", padx=5, pady=5)
#         self.speed_slider = ttk.Scale(slider_frame, from_=0.0, to=1.0, value=self.speed,
#                                       command=self.on_speed_change)
#         self.speed_slider.pack(fill="x", padx=5)
# 
#         # Keyboard controls
#         root.bind("<KeyPress>", self.on_key_press)
# =============================================================================




# --- Run ---
if __name__ == "__main__":
    print("This helper module should be used within pc_robot_gui.py.")