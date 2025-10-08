# Motor driver class
# Driving one motor by 2 pins: direction-pin and pwm-pin
# Usefull ao.: DRV8835 in Phase/Enable-mode. (MODE pin is pulled up.)

from machine import Pin, PWM
from utime import sleep_ms


class Motor:
    def __init__(self, motorDirPin=-1, motorPwmPin=-1, IsMovingForwards=True):
        self._pin_MotorDir = Pin(motorDirPin, Pin.OUT, value=0)
        self._pin_MotorPWM = PWM( Pin(motorPwmPin) )
        self._pin_MotorPWM.freq(1_000)
        self._pin_MotorPWM.duty_u16(0)
        self.IsForward = IsMovingForwards

    def __str__(self) -> str:
        return f"PWM: {self._pin_MotorPWM.duty_u16}."
    
    def set_motor_dir(self, IsForward=True):
        self._pin_MotorDir.value( IsForward==self.IsForward )
        
    def set_motor_pwm(self, value: int):
        self.set_motor_dir( value>0 )        
        self._pin_MotorPWM.duty_u16( min(abs(value), 65535) )

    def stop(self):
        self._pin_MotorPWM.duty_u16(0)


def Test_RampingUpDown():
    for i in range(0, 65000, 2500):
        mtrA.set_motor_pwm( i )
        mtrB.set_motor_pwm( i )
        print( f"PWM: {i}" )
        sleep_ms(100)
    for i in range(65000, 0, -2500):
        mtrA.set_motor_pwm( i )
        mtrB.set_motor_pwm( i )
        print( f"PWM: {i}" )
        sleep_ms(100)


#-----------------------------------------------------------------------
# Test myself:
if __name__ == "__main__":
    from utime import sleep

    try:
        mtrA = Motor(6,7)
        mtrB = Motor(8,9)

        Test_RampingUpDown()

    except Exception as X:
        print(X)

    finally:
        mtrA.stop()
        mtrB.stop()