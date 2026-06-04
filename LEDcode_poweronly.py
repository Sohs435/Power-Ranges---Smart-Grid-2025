from machine import Pin, PWM, Timer, SPI
from PID import PID
import time

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
timer_elapsed = 0
firstrun = 1

def tick(t):
    global timer_elapsed
    timer_elapsed = 1

# ---------------------------------------------------------------------------
# PWM pins and enables
# ---------------------------------------------------------------------------
pwm_red_pin = Pin(11)
pwm_yel_pin = Pin(9)
pwm_grn_pin = Pin(7)

pwm_red_en = Pin(10, Pin.OUT)
pwm_yel_en = Pin(8, Pin.OUT)
pwm_grn_en = Pin(6, Pin.OUT)

pwm_red = PWM(pwm_red_pin)
pwm_yel = PWM(pwm_yel_pin)
pwm_grn = PWM(pwm_grn_pin)

pwm_red.freq(100000)
pwm_yel.freq(100000)
pwm_grn.freq(100000)

# ---------------------------------------------------------------------------
# Control parameters
# ---------------------------------------------------------------------------
P_demand = 1

current_setpoint = 0.0
current_max = 0.95
current_min = 0.00

STARTUP_PROBE_CURRENT  = 0.01
VOLTAGE_DROP_THRESHOLD = 0.08

POWER_CONTROL_INTERVAL = 20
TOLERANCE_FACTOR       = 0.03
Kp_power               = 0.01

startup_detected = False

controller_red = PID(0.001, 1, 0, setpoint=current_setpoint, scale='ms')
controller_yel = PID(0.001, 1, 0, setpoint=current_setpoint, scale='ms')
controller_grn = PID(0.001, 1, 0, setpoint=current_setpoint, scale='ms')

# ---------------------------------------------------------------------------
# Accumulators
# ---------------------------------------------------------------------------
count        = 0
power_sum    = 0.0
power_samples = 0

# ---------------------------------------------------------------------------
# Print-deferral: the ISR-driven loop only sets this snapshot dict.
# The main loop prints it outside the timing-critical path.
# ---------------------------------------------------------------------------
PRINT_EVERY = 5          # print once every N power-control intervals (~100 ms)
print_counter = 0
pending_print = False

print_snapshot = {
    "P_demand":          0.0,
    "P_avg":             0.0,
    "error":             0.0,
    "startup_detected":  False,
    "current_setpoint":  0.0,
    "ired": 0.0, "iyel": 0.0, "igrn": 0.0,
    "vred": 0.0, "vyel": 0.0, "vgrn": 0.0,
    "P_out":             0.0,
}

# ---------------------------------------------------------------------------
# SPI / ADC
# ---------------------------------------------------------------------------
spi    = SPI(0, baudrate=400000)
adc_cs = Pin(17, mode=Pin.OUT, value=1)

# Pre-allocated ADC buffers — avoids per-call heap allocation
_tx = bytearray(3)
_rx = bytearray(3)

def readadc(channel):
    _tx[0] = 6 + (channel >> 2)
    _tx[1] = (channel & 3) << 6
    _tx[2] = 0
    adc_cs(0)
    spi.write_readinto(_tx, _rx)
    adc_cs(1)
    return ((_rx[1] & 15) << 8) + _rx[2]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def saturate(duty):
    if duty > 62500:
        return 62500
    if duty < 100:
        return 100
    return duty

def do_print(s):
    """Print the deferred snapshot — called only from the main loop."""
    print("P_demand         = {:.3f} W".format(s["P_demand"]))
    print("P_avg            = {:.3f} W".format(s["P_avg"]))
    print("error            = {:.3f} W".format(s["error"]))
    print("startup_detected = {}".format(s["startup_detected"]))
    print("current_setpoint = {:.3f} A".format(s["current_setpoint"]))
    print("Ired = {:.3f},  Iyel = {:.3f},  Igrn = {:.3f}".format(
        s["ired"], s["iyel"], s["igrn"]))
    print("Vred = {:.3f},  Vyel = {:.3f},  Vgrn = {:.3f}".format(
        s["vred"], s["vyel"], s["vgrn"]))
    print("Pout = {:.3f} W".format(s["P_out"]))
    print("")

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
while True:

    # ---- one-time startup --------------------------------------------------
    if firstrun:
        loop_timer = Timer(mode=Timer.PERIODIC, freq=1000, callback=tick)
        firstrun = 0

        pwm_red_en.value(0)
        pwm_yel_en.value(0)
        pwm_grn_en.value(0)

        print("Startup latch + proportional power control")
        print("P_demand               = {:.3f} W".format(P_demand))
        print("STARTUP_PROBE_CURRENT  = {:.3f} A".format(STARTUP_PROBE_CURRENT))
        print("VOLTAGE_DROP_THRESHOLD = {:.3f} V".format(VOLTAGE_DROP_THRESHOLD))
        print("")

    # ---- deferred print (outside timer-critical path) ----------------------
    if pending_print:
        pending_print = False
        do_print(print_snapshot)

    # ---- 1 kHz control tick ------------------------------------------------
    if timer_elapsed == 1:

        timer_elapsed = 0
        count         += 1

        pwm_red_en.value(1)
        pwm_yel_en.value(1)
        pwm_grn_en.value(1)

        # ADC reads
        ired_pin = 2.497 * (readadc(4) / 4096)
        iyel_pin = 2.497 * (readadc(2) / 4096)
        igrn_pin = 2.497 * (readadc(0) / 4096)

        vred_pin = 2.497 * (readadc(5) / 4096)
        vyel_pin = 2.497 * (readadc(3) / 4096)
        vgrn_pin = 2.497 * (readadc(1) / 4096)

        # Derived measurements
        vred = 2 * vred_pin - ired_pin
        vyel = 2 * vyel_pin - iyel_pin
        vgrn = 2 * vgrn_pin - igrn_pin

        ired = 3 * ired_pin
        iyel = 3 * iyel_pin
        igrn = 3 * igrn_pin

        pred = vred * ired
        pyel = vyel * iyel
        pgrn = vgrn * igrn

        P_out = pred + pyel + pgrn

        power_sum    += P_out
        power_samples += 1

        # Inner current PIDs
        pwm_red_out = saturate(int(controller_red(ired) * 65536))
        pwm_yel_out = saturate(int(controller_yel(iyel) * 65536))
        pwm_grn_out = saturate(int(controller_grn(igrn) * 65536))

        pwm_red.duty_u16(pwm_red_out)
        pwm_yel.duty_u16(pwm_yel_out)
        pwm_grn.duty_u16(pwm_grn_out)

        # ---- outer power loop (every POWER_CONTROL_INTERVAL ticks) ---------
        if count >= POWER_CONTROL_INTERVAL:

            P_avg     = power_sum / power_samples
            error     = P_demand - P_avg
            tolerance = TOLERANCE_FACTOR * P_demand

            # Startup latch: wait until all voltages drop (load connected)
            if not startup_detected:
                if (
                    vred < VOLTAGE_DROP_THRESHOLD and
                    vyel < VOLTAGE_DROP_THRESHOLD and
                    vgrn < VOLTAGE_DROP_THRESHOLD
                ):
                    startup_detected = True

            # Proportional power setpoint adjustment
            if startup_detected:
                if abs(error) > tolerance:
                    current_setpoint += Kp_power * error
            else:
                current_setpoint = STARTUP_PROBE_CURRENT

            # Clamp setpoint
            if current_setpoint > current_max:
                current_setpoint = current_max
            if current_setpoint < current_min:
                current_setpoint = current_min

            controller_red.setpoint = current_setpoint
            controller_yel.setpoint = current_setpoint
            controller_grn.setpoint = current_setpoint

            # Schedule a print — actual output happens at the top of the
            # next main-loop iteration, away from the timing-critical path.
            print_counter += 1
            if print_counter >= PRINT_EVERY:
                print_counter = 0
                print_snapshot["P_demand"]         = P_demand
                print_snapshot["P_avg"]            = P_avg
                print_snapshot["error"]            = error
                print_snapshot["startup_detected"] = startup_detected
                print_snapshot["current_setpoint"] = current_setpoint
                print_snapshot["ired"] = ired
                print_snapshot["iyel"] = iyel
                print_snapshot["igrn"] = igrn
                print_snapshot["vred"] = vred
                print_snapshot["vyel"] = vyel
                print_snapshot["vgrn"] = vgrn
                print_snapshot["P_out"]            = P_out
                pending_print = True

            # Reset accumulators
            count         = 0
            power_sum     = 0.0
            power_samples = 0