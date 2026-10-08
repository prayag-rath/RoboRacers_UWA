"""control.py - ESC + steering servo over hardware pwm. Started by main.py.

main.py calls  control.drive(forward, steer)  every frame, both -1..1.
The car goes to neutral on exit, Ctrl+C, terminate, a closed terminal and when main.py stops calling
drive() (DEADMAN_S). Not on kill -9 or a frozen pi, and not when a key hangs because the vnc link dropped.
"""
import atexit
import os
import signal
import threading
import time

# configs
PWM_CHIP = "/sys/class/pwm/pwmchip0"
ESC_CH = 0             # pwm0 = gpio 12
SERVO_CH = 1           # pwm1 = gpio 19
PERIOD_NS = 20000000   # 50 hz
NEUTRAL_US = 1500      # servo center pulse
ESC_NEUTRAL_US = 1500  # esc neutral pulse, must match calibrate.py
RANGE_US = 500         # max pulse change
ARM_TIME = 1           # s
DEADMAN_S = 0.3        # no drive() call for this long = neutral

STEER_SIGN = -1        # -1 if steering is reversed
STEER_TRIM = 0.0       # tune center pos [-1,1]

SPEED_START = 0.15     # max speed at start
SPEED_STEP = 0.05      # change with up/down arrow
SPEED_MAX = 1.0
STEER_START = 1.0      # steering factor at start
STEER_STEP = 0.1       # change with left/right arrow

state = {"throttle": 0.0, "steer": 0.0, "max_speed": SPEED_START, "steer_factor": STEER_START,
         "motors": True, "armed": False, "last": 0.0}


def _write(ch, name, value):
    if state["motors"]:
        with open(f"{PWM_CHIP}/pwm{ch}/{name}", "w") as f:
            f.write(str(value))


def _pulse(ch, us):
    _write(ch, "duty_cycle", int(us) * 1000)


def drive(forward, steer):
    # forward / steer in -1..1 (+ = forward / right), bigger values are cut off
    forward = max(-1, min(1, forward))
    steer = max(-1, min(1, steer))
    state["last"] = time.monotonic()
    state["throttle"] = forward * state["max_speed"]
    state["steer"] = steer * state["steer_factor"]
    _pulse(ESC_CH, ESC_NEUTRAL_US + state["throttle"] * RANGE_US)
    _pulse(SERVO_CH, NEUTRAL_US + (STEER_SIGN * state["steer"] + STEER_TRIM) * RANGE_US)


def change_speed(direction):
    state["max_speed"] = max(SPEED_STEP, min(SPEED_MAX, round(state["max_speed"] + direction * SPEED_STEP, 2)))


def change_steer(direction):
    state["steer_factor"] = max(STEER_STEP, min(1.0, round(state["steer_factor"] + direction * STEER_STEP, 1)))


def _deadman():
    # own thread: neutral if main.py stops calling drive(), for example when the window hangs
    while state["armed"]:
        time.sleep(DEADMAN_S / 3)
        if time.monotonic() - state["last"] > DEADMAN_S and (state["throttle"] or state["steer"]):
            try:
                drive(0, 0)
            except OSError:
                pass


def install_signals():
    # terminate / hangup (closed terminal, dropped vnc or ssh) -> leave the program so stop() runs
    def handler(signum, frame):
        raise SystemExit
    for name in ("SIGTERM", "SIGHUP"):
        if hasattr(signal, name):    # no SIGHUP on windows
            signal.signal(getattr(signal, name), handler)


def start(no_motors=False):
    state["motors"] = not no_motors
    if state["motors"]:
        try:
            for ch in (ESC_CH, SERVO_CH):
                if not os.path.exists(f"{PWM_CHIP}/pwm{ch}"):
                    with open(f"{PWM_CHIP}/export", "w") as f:
                        f.write(str(ch))
                    time.sleep(0.3)
                _write(ch, "period", PERIOD_NS)
                _write(ch, "enable", 1)
        except OSError as err:    # no pwm here: run without motors instead of crashing
            state["motors"] = False
            print("PWM failed, running without motors:", err)
    drive(0, 0)
    print("Starting ..." + ("" if state["motors"] else "  (no motors)"))
    time.sleep(ARM_TIME)
    state["armed"] = True
    threading.Thread(target=_deadman, daemon=True).start()
    atexit.register(stop)


def stop():
    # neutral and stop the pulses
    if not state["armed"]:
        return
    state["armed"] = False
    try:
        drive(0, 0)
        time.sleep(0.1)
        _write(ESC_CH, "enable", 0)
        _write(SERVO_CH, "enable", 0)
    except OSError:
        pass