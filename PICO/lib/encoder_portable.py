# encoder_portable.py

# Encoder Support: this version should be portable between MicroPython platforms
# Thanks to Evan Widloski for the adaptation to use the machine module

# Copyright (c) 2017-2022 Peter Hinch
# Released under the MIT License (MIT) - see LICENSE file

from machine import Pin

class Encoder:
    def __init__(self, pin_nr_A=-1, pin_nr_B=-1, countforwards=True, scale=1.0):
        #KHW20240919: updated 'scale=1.0' to float instead of int.
        #KHW20241115: changed PIN assignment from PIN() to pin-number
        self.scale = scale
        if countforwards:
            self.pin_x = Pin(pin_nr_A)
            self.pin_y = Pin(pin_nr_B)
        else:
            self.pin_x = Pin(pin_nr_B)
            self.pin_y = Pin(pin_nr_A)
        self._x = self.pin_x()
        self._y = self.pin_y()
        self._ticks = 0
        self._prev_ticks = 0
        try:
            self.x_interrupt = self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback, hard=True)
            self.y_interrupt = self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback, hard=True)
        except TypeError:
            self.x_interrupt = self.pin_x.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.x_callback)
            self.y_interrupt = self.pin_y.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.y_callback)

    def x_callback(self, pin_x):
        if (x := pin_x()) != self._x:  # Reject short pulses
            self._x = x
            self.forward = x ^ self.pin_y()
            self._ticks += 1 if self.forward else -1

    def y_callback(self, pin_y):
        if (y := pin_y()) != self._y:
            self._y = y
            self.forward = y ^ self.pin_x() ^ 1
            self._ticks += 1 if self.forward else -1

    def ticks(self, value=None) -> int:
        if value is not None:
            self._ticks = value
        return self._ticks

    def delta_ticks(self) -> int:                
        delta_ticks = self._ticks - self._prev_ticks
        self._prev_ticks = self._ticks
        return delta_ticks

    def position(self, value=None) -> float:
        if value is not None:
            self._ticks = round(value / self.scale)  # Improvement provided by @IhorNehrutsa
        return round(self._ticks * self.scale,1)     # KHW: Showing position in [unit]

    def __str__(self) -> str:
        return f"Ticks: {self._ticks}."

#-----------------------------------------------------------------------
# Test myself:
if __name__ == "__main__":
    from utime import sleep

    EncA = Encoder(10, 11)
    EncB = Encoder(12, 13)

    try:
        while True:
            print( f"{EncA.delta_ticks()} : {EncB.delta_ticks()}" )
            sleep(1)

    except Exception as X:
        print(X)