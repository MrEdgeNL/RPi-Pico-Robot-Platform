import time

# -----------------------------
# --- Missions (step-loop style)
# -----------------------------
class Mission:
    def __init__(self, output_q):
        self.out_q = output_q
        self.mode = "Nothing"
    
    def start(self):
        raise NotImplementedError

    def update(self):
        raise NotImplementedError

    def stop(self):
        pass


# -----------------------------
# --- Mission: IDLE ---
# -----------------------------
class IdleMission(Mission):
    def start(self):
        self.out_q.put("MSG mode idle")
        self.mode = "idle"

    def update(self):
        # Occasionally log
        self.out_q.put("MSG robot_is_waiting..")
        time.sleep(5)

# -----------------------------
# DriveControlMission
# -----------------------------
class DriveControlMission(Mission):
    def __init__(self, output_q, drive):
        super().__init__(output_q)
        self.drive = drive
        self.mode = "idle"
        self.target_pose = None
        self.last_cmd_time = time.ticks_ms()

    def on_command(self, cmd):
        """Handle commands for direct control and single-pose moves"""
        action = cmd.get("action")

        if action == "direct_control":
            self.mode = "direct"
            v_l, v_r = cmd.get("v_l", 0), cmd.get("v_r", 0)
            self.drive.set_wheel_speed_setpoint(v_l, v_r)
            self.last_cmd_time = time.ticks_ms()

        elif action == "move_to_pose":
            self.mode = "move_to_pose"
            self.target_pose = (
                cmd.get("x", 0),
                cmd.get("y", 0),
                cmd.get("theta", 0),
            )
            self.last_cmd_time = time.ticks_ms()

        elif action == "stop":
            self.mode = "idle"
            self.drive.set_wheel_speed_setpoint(0, 0)

    def step(self):
        now = time.ticks_ms()

        # Timeout check for direct control (joystick safety)
        if self.mode == "direct":
            if time.ticks_diff(now, self.last_cmd_time) > 200:  # 0.2s
                self.drive.set_wheel_speed_setpoint(0, 0)
                self.mode = "idle"
                self.out_q.put({"type": "log", "msg": "Direct control timeout"})
            return

        if self.mode == "move_to_pose" and self.target_pose:
            x, y, theta = self.target_pose
            v_l, v_r = self.drive.MoveToPose(x, y, theta, v_wheelspeed_setpoint=300)
            self.drive.set_wheel_speed_setpoint(v_l, v_r)

            # Detect completion when wheel speeds are ~0
            if abs(v_l) < 1 and abs(v_r) < 1:
                self.out_q.put({"type": "log", "msg": f"Arrived at ({x},{y},{theta})"})
                self.mode = "idle"

        elif self.mode == "idle":
            self.drive.set_wheel_speed_setpoint(0, 0)
        
        print(">pico: " + str(self.drive.pose) )


# -----------------------------
# WaypointMission
# -----------------------------
class WaypointMission(Mission):
    def __init__(self, output_q, drive):
        super().__init__(output_q)
        self.drive = drive
        self.waypoints = []
        self.current_wp_index = 0
        self.mode = "idle"

    def on_command(self, cmd):
        """Handle commands for waypoint navigation"""
        action = cmd.get("action")

        if action == "follow_waypoints":
            self.waypoints = cmd.get("points", [])
            self.current_wp_index = 0
            self.mode = "active"

        elif action == "stop":
            self.mode = "idle"
            self.drive.set_wheel_speed_setpoint(0, 0)

    def step(self):
        if self.mode == "active":
            if self.current_wp_index < len(self.waypoints):
                x, y, theta = self.waypoints[self.current_wp_index]
                v_l, v_r = self.drive.MoveToPose(x, y, theta, v_wheelspeed_setpoint=200)
                self.drive.set_wheel_speed_setpoint(v_l, v_r)

                if abs(v_l) < 1 and abs(v_r) < 1:
                    self.out_q.put({"type": "log", "msg": f"Reached waypoint {self.current_wp_index}"})
                    self.current_wp_index += 1
            else:
                self.out_q.put({"type": "log", "msg": "All waypoints completed"})
                self.mode = "idle"
                self.drive.set_wheel_speed_setpoint(0, 0)

        elif self.mode == "idle":
            self.drive.set_wheel_speed_setpoint(0, 0)

        print(">pico: " + str(self.drive.pose) )

# -----------------------------
# --- Mission: Line Following ---
# -----------------------------
class LineFollowingMission(Mission):
    def __init__(self, output_q):
        super().__init__(output_q)  # Need including parent’s initialization logic.
        self.mode = "linefollow"
        self.ticks = 0
        self._isrunning = False

    def start(self):
        self._isrunning = True
        self.out_q.put("MSG mode linefollowing")

    def update(self):
        if not self._isrunning:
            return
        
        self.ticks += 1
        if self.ticks % 10 == 0:
            self.out_q.put(f"LOG PoSe {self.ticks}")
        if self.ticks>100:
            self._isrunning = False
            self.out_q.put("MSG robot_is_finished")

        time.sleep(0.5)

    def stop(self):
        self._isrunning = False