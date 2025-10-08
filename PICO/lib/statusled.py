import machine, time, _thread

# -----------------------------
# Network object 
# -----------------------------
class StatusLED:
    def __init__(self):
        self.led = machine.Pin("LED", machine.Pin.OUT)
        self.mode = "off"
        self.state = 0
        self.led.value(self.state)
        self.last_toggle = time.ticks_ms()
        self.pattern = None
        self.pattern_index = 0

    # --- Mode setters ---
    def on(self):
        self.mode = "on"
        self.led.value(1)

    def off(self):
        self.mode = "off"
        self.led.value(0)
    
    def toggle_onoff(self):
        self.mode = "toggle_onoff"
        self.state ^= 1
        self.led.value(self.state)

    def blink_slow(self):
        self.mode = "blink_slow"
        self.state = 0
        self.last_toggle = time.ticks_ms()

    def blink_fast(self):
        self.mode = "blink_fast"
        self.state = 0
        self.last_toggle = time.ticks_ms()

    def blink_pattern(self, pattern_ms):
        """
        Blink pattern: list of ON/OFF durations in ms.
        Example: [200, 200, 600, 600] → quick, quick, long pause.
        """
        self.mode = "pattern"
        self.pattern = pattern_ms
        self.pattern_index = 0
        self.state = 1
        self.led.value(1)
        self.last_toggle = time.ticks_ms()

    # --- Internal updater ---
    def _update(self):
        now = time.ticks_ms()

        if self.mode == "blink_slow":
            if time.ticks_diff(now, self.last_toggle) > 1000:  # 1 Hz
                self.state ^= 1
                self.led.value(self.state)
                self.last_toggle = now

        elif self.mode == "blink_fast":
            if time.ticks_diff(now, self.last_toggle) > 200:  # 5 Hz
                self.state ^= 1
                self.led.value(self.state)
                self.last_toggle = now

        elif self.mode == "pattern" and self.pattern:
            if time.ticks_diff(now, self.last_toggle) > self.pattern[self.pattern_index]:
                self.state ^= 1
                self.led.value(self.state)
                self.last_toggle = now
                self.pattern_index = (self.pattern_index + 1) % len(self.pattern)

        # elif self.mode == "on":
        #     self.led.value(1)

        # elif self.mode == "off":
        #     self.led.value(0)


