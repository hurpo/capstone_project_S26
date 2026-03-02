import pigpio

_pi = pigpio.pi()
if not _pi.connected:
    raise IOError("Can't connect to pigpio")

# Motor speeds for this library are specified as numbers between -MAX_SPEED and
# MAX_SPEED, inclusive.
# This has a value of 480 for historical reasons/to maintain compatibility with
# older libraries for other Pololu boards (which used WiringPi to set up the
# hardware PWM directly).
_max_speed = 480
MAX_SPEED = _max_speed

_pin_M1INA = 16
_pin_M1INB = 18
_pin_M1PWM = 12
#_pin_M1EN = 5

_pin_M2INA = 13
_pin_M2INB = 14
_pin_M2PWM = 33
# _pin_M2EN = 6

class Motor(object):
    MAX_SPEED = _max_speed

    def __init__(self, ina_pin, inb_pin, pwm_pin): #en_diag_pin
        self.ina_pin = ina_pin
        self.inb_pin = inb_pin
        self.pwm_pin = pwm_pin
        #self.en_diag_pin = en_diag_pin

        _pi.set_mode(self.ina_pin, pigpio.OUTPUT)
        _pi.set_mode(self.inb_pin, pigpio.OUTPUT)
        _pi.set_mode(self.pwm_pin, pigpio.OUTPUT)
        #_pi.set_mode(self.en_diag_pin, pigpio.INPUT)
        #_pi.set_pull_up_down(self.en_diag_pin, pigpio.PUD_UP)

        _pi.write(self.ina_pin, 0)
        _pi.write(self.inb_pin, 0)

    def setSpeed(self, speed):
        if speed > self.MAX_SPEED:
            speed = self.MAX_SPEED
        elif speed < -self.MAX_SPEED:
            speed = -self.MAX_SPEED
    
        if speed > 0:
            _pi.write(self.ina_pin, 1)
            _pi.write(self.inb_pin, 0)
        elif speed < 0:
            _pi.write(self.ina_pin, 0)
            _pi.write(self.inb_pin, 1)
        else: 
            _pi.write(self.ina_pin, 1)
            _pi.write(self.inb_pin, 1)

        abs_speed = abs(speed)
        duty_cycle = int(abs_speed * 1000000 / self.MAX_SPEED)
        _pi.hardware_PWM(self.pwm_pin, 20000, duty_cycle)

    def enable(self):
        pass

    def disable(self):
        _pi.write(self.ina_pin, 1)
        _pi.write(self.inb_pin, 1)
        _pi.hardware_PWM(self.pwm_pin, 0, 0)

    # def getFault(self):
    #     return not _pi.read(self.en_diag_pin)

class Motors(object):
    MAX_SPEED = _max_speed

    def __init__(self):
        self.motor1 = Motor(_pin_M1INA, _pin_M1INB, _pin_M1PWM) # _pin_M1EN
        self.motor2 = Motor(_pin_M2INA, _pin_M2INB, _pin_M2PWM)# _pin_M2EN

    def setSpeeds(self, m1_speed, m2_speed):
        self.motor1.setSpeed(m1_speed)
        self.motor2.setSpeed(m2_speed)

    def enable(self):
        self.motor1.enable()
        self.motor2.enable()

    def disable(self):
        self.motor1.disable()
        self.motor2.disable()

    # def getFaults(self):
    #     return self.motor1.getFault() or self.motor2.getFault()

    def forceStop(self):
        # reinitialize the pigpio interface in case we interrupted another command
        # (so this method works reliably when called from an exception handler)
        global _pi
        _pi.stop()
        _pi = pigpio.pi()
        self.setSpeeds(0, 0)

motors = Motors()
