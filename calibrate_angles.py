"""Lab 8 - calibrate_angles.py
Sends 0..180 deg in 10-degree steps to each servo. At each step, mark the arm
position on the paper, then press Enter. Measure the actual angles afterwards
and type them in at the end; calibration_data_<ID>.txt is written."""
from kinematics_calibration import (shoulder_pwm, elbow_pwm, set_servo,
                                    JIG_ID, sleep)

ANGLES = list(range(0, 181, 10))


def sweep(pwm, name):
    print("--- %s servo: mark the arm tip/line at each angle ---" % name)
    for a in ANGLES:
        set_servo(pwm, a)
        sleep(1.0)
        input("Commanded %3d deg - mark it, then press Enter..." % a)


def main():
    # Park the other servo mid-range while one is calibrated.
    set_servo(elbow_pwm, 90)
    sweep(shoulder_pwm, "SHOULDER")
    set_servo(shoulder_pwm, 90)
    sweep(elbow_pwm, "ELBOW")

    print("\nNow enter the ACTUAL measured angles (protractor).")
    rows = []
    for a in ANGLES:
        s = float(input("Shoulder actual for commanded %3d: " % a))
        e = float(input("Elbow    actual for commanded %3d: " % a))
        rows.append((a, s, e))
    fname = "calibration_data_%s.txt" % JIG_ID
    with open(fname, "w") as f:
        f.write("# desired  shoulder_actual  elbow_actual (degrees)\n")
        for a, s, e in rows:
            f.write("%d %.1f %.1f\n" % (a, s, e))
    print("Saved", fname)


main()
