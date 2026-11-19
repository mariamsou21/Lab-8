"""Lab 8 - inverse_kinematics, calibration_setup, send_compensated_angle.
MicroPython for Raspberry Pi Pico."""
import math
from machine import Pin, PWM
from utime import sleep

# ---------------- CONFIG ----------------
SHOULDER_PIN = 0          # GP0 (shoulder)
ELBOW_PIN = 1             # GP1 (elbow); wrist/pen = GP2
JIG_ID = "2"              # test jig ID #2
MIN_US, MAX_US = 500, 2500  # pulse width for 0 and 180 degrees

LA = LB = 155.0           # link lengths (mm)
AX, AY = -50.0, 139.5     # shoulder position (mm)
SHOULDER_OFFSET = 75.0    # eq. 8: servoA = alpha - 75
ELBOW_OFFSET = 150.0      # eq. 9: servoB = 150 - beta (tune per jig)
# ----------------------------------------

shoulder_pwm = PWM(Pin(SHOULDER_PIN)); shoulder_pwm.freq(50)
elbow_pwm = PWM(Pin(ELBOW_PIN)); elbow_pwm.freq(50)


def set_servo(pwm, angle):
    """Send a raw angle (0-180 deg) to a servo, clamped to a safe range."""
    angle = max(0.0, min(180.0, angle))
    pulse_us = MIN_US + (MAX_US - MIN_US) * angle / 180.0
    pwm.duty_u16(int(pulse_us / 20000.0 * 65535))   # 20 ms period at 50 Hz


def _clamp(x):
    return max(-1.0, min(1.0, x))   # guards acos/asin against rounding error


def inverse_kinematics(cx, cy):
    """Return (servoA, servoB) in degrees for pen position C = (cx, cy) in mm.
    Returns None if the point is out of reach."""
    ac = math.sqrt((AX - cx) ** 2 + (AY - cy) ** 2)                 # (1)
    if ac > LA + LB or ac == 0:
        return None
    abase_c = math.sqrt((AX - cx) ** 2 + cy ** 2)                    # (2)
    bac = math.acos(_clamp((LA**2 + ac**2 - LB**2) / (2 * LA * ac)))  # (3)
    acb = math.asin(_clamp(LA * math.sin(bac) / LB))                  # (4)
    yac = math.acos(_clamp((AY**2 + ac**2 - abase_c**2) / (2 * AY * ac)))  # (5)
    alpha = bac + yac                                                 # (6)
    beta = bac + acb                                                  # (7)
    servo_a = math.degrees(alpha) - SHOULDER_OFFSET                   # (8)
    servo_b = ELBOW_OFFSET - math.degrees(beta)                       # (9)
    return servo_a, servo_b


def calibration_setup(filename=None):
    """Read calibration_data_<ID>.txt and return a table of errors:
        {"angles": [0,10,...,180], "shoulder": [...], "elbow": [...]}
    File lines: desired  shoulder_actual  elbow_actual  (# = comment).
    Stored error = desired - actual, i.e. the correction to ADD to a command."""
    if filename is None:
        filename = "calibration_data_%s.txt" % JIG_ID
    angles, sh, el = [], [], []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            d, s, e = [float(v) for v in line.replace(",", " ").split()]
            angles.append(d)
            sh.append(d - s)
            el.append(d - e)
    return {"angles": angles, "shoulder": sh, "elbow": el}


def _lookup_error(table, servo, angle):
    """Linear interpolation of the error table between 10-degree points."""
    xs, ys = table["angles"], table[servo]
    if angle <= xs[0]:
        return ys[0]
    if angle >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if xs[i] <= angle <= xs[i + 1]:
            t = (angle - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + t * (ys[i + 1] - ys[i])


def send_compensated_angle(table, shoulder_angle, elbow_angle):
    """Correct the desired angles with the error table, then drive the servos."""
    s = shoulder_angle + _lookup_error(table, "shoulder", shoulder_angle)
    e = elbow_angle + _lookup_error(table, "elbow", elbow_angle)
    set_servo(shoulder_pwm, s)
    set_servo(elbow_pwm, e)


def draw_square(table=None, x0=60, y0=100, side=60, steps=15, delay=0.05):
    """Trace a square. table=None -> uncalibrated, else compensated."""
    corners = [(x0, y0), (x0 + side, y0), (x0 + side, y0 + side),
               (x0, y0 + side), (x0, y0)]
    for (xa, ya), (xb, yb) in zip(corners, corners[1:]):
        for i in range(steps + 1):
            t = i / steps
            ang = inverse_kinematics(xa + t * (xb - xa), ya + t * (yb - ya))
            if ang is None:
                continue
            if table is None:
                set_servo(shoulder_pwm, ang[0]); set_servo(elbow_pwm, ang[1])
            else:
                send_compensated_angle(table, ang[0], ang[1])
            sleep(delay)


if __name__ == "__main__":
    draw_square(None)                  # 1) without calibration
    sleep(3)                           # (swap paper / change pen colour)
    draw_square(calibration_setup())   # 2) with calibration
