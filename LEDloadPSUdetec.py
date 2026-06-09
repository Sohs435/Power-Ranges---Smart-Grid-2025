from machine import Pin, PWM, Timer, SPI
from PID import PID
import time
import network
import urequests
import _thread

# ---------------------------------------------------------------------------
# WiFi / Web server
# ---------------------------------------------------------------------------
SSID = "RonakA55"
PASSWORD = "nonakisguy"

WEB_DEMAND_URL = "https://icelec50015.azurewebsites.net/demand"

P_demand = 0.0
webserver_day = 0
webserver_tick = 0
last_demand = None

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)

    print("Connecting to WiFi...")

    while not wlan.isconnected():
        time.sleep(0.2)

    print("WiFi connected")
    print(wlan.ifconfig())


def wifi_demand_task():
    global P_demand
    global last_demand
    global webserver_day
    global webserver_tick

    while True:
        try:
            response = urequests.get(WEB_DEMAND_URL)
            data = response.json()
            response.close()

            new_demand = float(data["demand"])
            webserver_day = int(data["day"])
            webserver_tick = int(data["tick"])

            if last_demand is None or new_demand != last_demand:
                last_demand = new_demand
                P_demand = new_demand

                print("New webserver demand = {:.3f} W".format(P_demand))
                print("Webserver day =", webserver_day)
                print("Webserver tick =", webserver_tick)
                print("")

        except Exception as e:
            print("Webserver demand read failed:", e)

        time.sleep(1)


# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
timer_elapsed = 0

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
current_setpoint = 0.0
current_max = 0.95
current_min = 0.00

STARTUP_PROBE_CURRENT  = 0.01
VOLTAGE_DROP_THRESHOLD = 0.08

PSU_OFF_V_MIN = 0.10
PSU_OFF_V_MAX = 0.15
CURRENT_ZERO_THRESHOLD   = 0.001
PSU_ON_CURRENT_THRESHOLD = 0.01

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
count = 0
power_sum = 0.0
power_samples = 0


# ---------------------------------------------------------------------------
# Print deferral
# ---------------------------------------------------------------------------
PRINT_EVERY = 5
print_counter = 0
pending_print = False

print_snapshot = {
    "P_demand": 0.0,
    "P_avg": 0.0,
    "error": 0.0,
    "startup_detected": False,
    "current_setpoint": 0.0,
    "ired": 0.0,
    "iyel": 0.0,
    "igrn": 0.0,
    "vred": 0.0,
    "vyel": 0.0,
    "vgrn": 0.0,
    "P_out": 0.0,
    "day": 0,
    "tick": 0,
}


# ---------------------------------------------------------------------------
# SPI / ADC
# ---------------------------------------------------------------------------
spi = SPI(0, baudrate=400000)
adc_cs = Pin(17, mode=Pin.OUT, value=1)

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


def reset_controllers():
    controller_red.reset()
    controller_yel.reset()
    controller_grn.reset()


def do_print(s):
    print("Webserver day    =", s["day"])
    print("Webserver tick   =", s["tick"])
    print("Web demand       = {:.3f} W".format(s["P_demand"]))
    print("LED P_avg        = {:.3f} W".format(s["P_avg"]))
    print("LED Pout instant = {:.3f} W".format(s["P_out"]))
    print("error            = {:.3f} W".format(s["error"]))
    print("startup_detected = {}".format(s["startup_detected"]))
    print("current_setpoint = {:.3f} A".format(s["current_setpoint"]))

    print("Ired = {:.3f},  Iyel = {:.3f},  Igrn = {:.3f}".format(
        s["ired"], s["iyel"], s["igrn"]))

    print("Vred = {:.3f},  Vyel = {:.3f},  Vgrn = {:.3f}".format(
        s["vred"], s["vyel"], s["vgrn"]))

    print("")


# ---------------------------------------------------------------------------
# One-time setup BEFORE main loop
# ---------------------------------------------------------------------------
connect_wifi()

_thread.start_new_thread(wifi_demand_task, ())

loop_timer = Timer(mode=Timer.PERIODIC, freq=1000, callback=tick)

pwm_red_en.value(0)
pwm_yel_en.value(0)
pwm_grn_en.value(0)

print("Startup latch + proportional power control")
print("Initial P_demand           = {:.3f} W".format(P_demand))
print("STARTUP_PROBE_CURRENT      = {:.3f} A".format(STARTUP_PROBE_CURRENT))
print("VOLTAGE_DROP_THRESHOLD     = {:.3f} V".format(VOLTAGE_DROP_THRESHOLD))
print("")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
while True:

    if pending_print:
        pending_print = False
        do_print(print_snapshot)

    if timer_elapsed == 1:

        timer_elapsed = 0
        count += 1

        pwm_red_en.value(1)
        pwm_yel_en.value(1)
        pwm_grn_en.value(1)

        ired_pin = 2.497 * (readadc(4) / 4096)
        iyel_pin = 2.497 * (readadc(2) / 4096)
        igrn_pin = 2.497 * (readadc(0) / 4096)

        vred_pin = 2.497 * (readadc(5) / 4096)
        vyel_pin = 2.497 * (readadc(3) / 4096)
        vgrn_pin = 2.497 * (readadc(1) / 4096)

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

        power_sum += P_out
        power_samples += 1

        pwm_red_out = saturate(int(controller_red(ired) * 65536))
        pwm_yel_out = saturate(int(controller_yel(iyel) * 65536))
        pwm_grn_out = saturate(int(controller_grn(igrn) * 65536))

        pwm_red.duty_u16(pwm_red_out)
        pwm_yel.duty_u16(pwm_yel_out)
        pwm_grn.duty_u16(pwm_grn_out)

        if count >= POWER_CONTROL_INTERVAL:

            P_avg = power_sum / power_samples
            error = P_demand - P_avg
            tolerance = TOLERANCE_FACTOR * P_demand

            # PSU is off: all currents near zero and voltages in idle float window.
            # De-latch, reset setpoint, and flush PID controllers.
            if (
                abs(ired) < CURRENT_ZERO_THRESHOLD and
                abs(iyel) < CURRENT_ZERO_THRESHOLD and
                abs(igrn) < CURRENT_ZERO_THRESHOLD and
                PSU_OFF_V_MIN < vred < PSU_OFF_V_MAX and
                PSU_OFF_V_MIN < vyel < PSU_OFF_V_MAX and
                PSU_OFF_V_MIN < vgrn < PSU_OFF_V_MAX
            ):
                startup_detected = False
                current_setpoint = STARTUP_PROBE_CURRENT
                reset_controllers()

            # PSU is on: current is flowing, latch startup and run power loop.
            elif (
                ired > PSU_ON_CURRENT_THRESHOLD or
                iyel > PSU_ON_CURRENT_THRESHOLD or
                igrn > PSU_ON_CURRENT_THRESHOLD
            ):
                startup_detected = True
                if abs(error) > tolerance:
                    current_setpoint += Kp_power * error

            # Transition region: wait for voltage drop confirming PSU connection.
            else:
                if not startup_detected:
                    if (
                        vred < VOLTAGE_DROP_THRESHOLD and
                        vyel < VOLTAGE_DROP_THRESHOLD and
                        vgrn < VOLTAGE_DROP_THRESHOLD
                    ):
                        startup_detected = True

                if startup_detected:
                    if abs(error) > tolerance:
                        current_setpoint += Kp_power * error
                else:
                    current_setpoint = STARTUP_PROBE_CURRENT

            if current_setpoint > current_max:
                current_setpoint = current_max

            if current_setpoint < current_min:
                current_setpoint = current_min

            controller_red.setpoint = current_setpoint
            controller_yel.setpoint = current_setpoint
            controller_grn.setpoint = current_setpoint

            print_counter += 1

            if print_counter >= PRINT_EVERY:
                print_counter = 0

                print_snapshot["P_demand"]         = P_demand
                print_snapshot["P_avg"]            = P_avg
                print_snapshot["error"]            = error
                print_snapshot["startup_detected"] = startup_detected
                print_snapshot["current_setpoint"] = current_setpoint
                print_snapshot["ired"]             = ired
                print_snapshot["iyel"]             = iyel
                print_snapshot["igrn"]             = igrn
                print_snapshot["vred"]             = vred
                print_snapshot["vyel"]             = vyel
                print_snapshot["vgrn"]             = vgrn
                print_snapshot["P_out"]            = P_out
                print_snapshot["day"]              = webserver_day
                print_snapshot["tick"]             = webserver_tick

                pending_print = True

            count = 0
            power_sum = 0.0
            power_samples = 0