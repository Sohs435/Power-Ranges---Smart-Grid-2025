from machine import Pin, I2C, ADC, PWM, Timer

# ADC pins
va_pin = ADC(Pin(28))
vb_pin = ADC(Pin(26))
vpot_pin = ADC(Pin(27))

OL_CL_pin = Pin(12, Pin.IN, Pin.PULL_UP)
BU_BO_pin = Pin(2, Pin.IN, Pin.PULL_UP)

# I2C for INA219
ina_i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=2400000)

# PWM setup
pwm = PWM(Pin(9))
pwm.freq(100000)

min_pwm = 1000
max_pwm = 64536
pwm_out = max_pwm
pwm_ref = 30000
pwm.duty_u16(65536 - pwm_out)

# Protection / control values
V_TARGET = 10.25
OC_LIMIT = 2.0
PSU_ON_THRESHOLD = 8.0

v_err = 0
v_err_int = 0
v_pi_out = 0

kp = 2000
ki = 500

trip = 0
OC = 0

# Pot filter
v_pot_filt = [0] * 100
v_pot_index = 0

timer_elapsed = 0
count = 0
first_run = 1

SHUNT_OHMS = 0.10


def saturate(signal, upper, lower):
    if signal > upper:
        signal = upper
    if signal < lower:
        signal = lower
    return signal


def tick(t):
    global timer_elapsed
    timer_elapsed = 1


class ina219:

    REG_CONFIG = 0x00
    REG_SHUNTVOLTAGE = 0x01
    REG_BUSVOLTAGE = 0x02
    REG_POWER = 0x03
    REG_CURRENT = 0x04
    REG_CALIBRATION = 0x05

    def __init__(self, sr, address, maxi):
        self.address = address
        self.shunt = sr

    def vshunt(self):
        reg_bytes = ina_i2c.readfrom_mem(self.address, self.REG_SHUNTVOLTAGE, 2)
        reg_value = int.from_bytes(reg_bytes, 'big')

        if reg_value > 2**15:
            sign = -1
            for i in range(16):
                reg_value = reg_value ^ (1 << i)
        else:
            sign = 1

        return float(reg_value) * 1e-5 * sign

    def vbus(self):
        reg_bytes = ina_i2c.readfrom_mem(self.address, self.REG_BUSVOLTAGE, 2)
        reg_value = int.from_bytes(reg_bytes, 'big') >> 3
        return float(reg_value) * 0.004

    def configure(self):
        ina_i2c.writeto_mem(self.address, self.REG_CONFIG, b'\x19\x9F')
        ina_i2c.writeto_mem(self.address, self.REG_CALIBRATION, b'\x00\x00')


while True:

    if first_run:
        ina = ina219(SHUNT_OHMS, 64, 5)
        ina.configure()
        first_run = 0
        loop_timer = Timer(mode=Timer.PERIODIC, freq=1000, callback=tick)

    if timer_elapsed == 1:

        # ADC readings
        va = 1.017 * (12490 / 2490) * 3.3 * (va_pin.read_u16() / 65536)
        vb = 1.015 * (12490 / 2490) * 3.3 * (vb_pin.read_u16() / 65536)

        vpot_in = 1.026 * 3.3 * (vpot_pin.read_u16() / 65536)
        v_pot_filt[v_pot_index] = vpot_in
        v_pot_index = (v_pot_index + 1) % 100
        vpot = sum(v_pot_filt) / 100

        Vshunt = ina.vshunt()
        iL = Vshunt / SHUNT_OHMS

        CL = OL_CL_pin.value()
        BU = BU_BO_pin.value()

        pwm_ref = saturate(65536 - int((vpot / 3.3) * 65536), max_pwm, min_pwm)

        if CL != 1:
            # Open loop
            v_err = 0
            v_err_int = 0
            v_pi_out = 0

            if iL > OC_LIMIT:
                pwm_out = pwm_out - 10
                OC = 1
                pwm_out = saturate(pwm_out, pwm_ref, min_pwm)

            elif iL < -OC_LIMIT:
                pwm_out = pwm_out + 10
                OC = 1
                pwm_out = saturate(pwm_out, max_pwm, pwm_ref)

            else:
                pwm_out = pwm_ref
                OC = 0
                pwm_out = saturate(pwm_out, pwm_ref, min_pwm)

            duty = int(65536 - pwm_out)
            pwm.duty_u16(duty)

        else:
            # Closed loop

            # PSU not on yet: stop PI wind-up
            if va < PSU_ON_THRESHOLD:
                OC = 0

                v_err = 0
                v_err_int = 0
                v_pi_out = 0

                pwm_out = max_pwm
                duty = int(65536 - pwm_out)
                pwm.duty_u16(duty)

            elif abs(iL) > OC_LIMIT:
                OC = 1

                # Reset integral during overcurrent too
                v_err_int = 0

                pwm_out = pwm_out - 100
                pwm_out = saturate(pwm_out, max_pwm, min_pwm)

                duty = int(65536 - pwm_out)
                pwm.duty_u16(duty)

            else:
                OC = 0

                v_err = V_TARGET - vb

                v_err_int = v_err_int + v_err
                v_err_int = saturate(v_err_int, 10000, -10000)

                v_pi_out = (kp * v_err) + (ki * v_err_int)

                pwm_out = saturate(int(v_pi_out), max_pwm, min_pwm)

                duty = int(65536 - pwm_out)
                pwm.duty_u16(duty)

        count += 1
        timer_elapsed = 0

        if count > 100:
            print("Va    = {:.3f} V".format(va))
            print("Vb    = {:.3f} V".format(vb))
            print("Vpot  = {:.3f} V".format(vpot))
            print("iL    = {:.3f} A".format(iL))
            print("v_err = {:.3f}".format(v_err))
            print("v_err_int = {:.1f}".format(v_err_int))
            print("v_pi_out  = {:.1f}".format(v_pi_out))
            print("pwm_out = {:d}".format(pwm_out))
            print("duty  = {:d}".format(duty))
            print("OC = {:b}  CL = {:b}  BU = {:b}".format(OC, CL, BU))
            count = 0