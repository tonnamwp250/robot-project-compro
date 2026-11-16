"""Manual robot - simple assignment draft for CyberPi / mBlock Python.

Joystick: drive. A: select tool. B: operate tool. Centre: stop and exit.
Actuator outputs are OFF by default; this is not calibrated robot firmware.
"""
import time
import cyberpi

# Provisional ports for CyberPi with an mBot2 Shield. Confirm the wiring.
ENABLE_OUTPUTS = False
LEFT_WHEEL = "EM1"
RIGHT_WHEEL = "EM2"
FEEDER = "M1"
LAUNCHER = "M2"
GRIPPER = "S3"  # One jaw servo drives the current linked gripper.
LIFT = "S4"

# Example settings only: tune these after printing and assembly.
DRIVE_POWER = 25
LEFT_SIGN = 1
RIGHT_SIGN = -1
FEED_POWER = 18
LAUNCH_POWER = 15
MAX_JOG_SECONDS = 0.6
GRIP_OPEN, GRIP_CLOSED = 70, 105
LIFT_DOWN, LIFT_UP = 70, 110
TOOLS = ("Gripper", "Lift", "Feeder up", "Feeder down", "Launcher jog")


def motors(left=0, right=0, feeder=0, launcher=0):
    """Set motor power (-100 to 100); do nothing in preview mode."""
    if ENABLE_OUTPUTS:
        cyberpi.mbot2.EM_set_power(left * LEFT_SIGN, LEFT_WHEEL)
        cyberpi.mbot2.EM_set_power(right * RIGHT_SIGN, RIGHT_WHEEL)
        cyberpi.mbot2.motor_set(feeder, FEEDER)
        cyberpi.mbot2.motor_set(launcher, LAUNCHER)


def servo(angle, port):
    if ENABLE_OUTPUTS:
        cyberpi.mbot2.servo_set(angle, port)


def show(message):
    cyberpi.console.clear()
    cyberpi.console.println("OUTPUTS ON" if ENABLE_OUTPUTS else "PREVIEW ONLY")
    cyberpi.console.println(message)


def main():
    tool = 0
    gripped = False
    raised = False
    old_a = old_b = False
    jog_start = 0
    motors()  # Stop motors; do not move either servo at startup.
    show(TOOLS[tool])

    try:
        while True:
            # Stop takes priority over driving and mechanism commands.
            if cyberpi.controller.is_press("middle"):
                motors()
                show("STOPPED - restart")
                break

            a = cyberpi.controller.is_press("a")
            b = cyberpi.controller.is_press("b")
            now = cyberpi.timer.get()

            # One tool change per press; ignore A while B is held.
            if a and not old_a and not b:
                tool = (tool + 1) % len(TOOLS)
                show(TOOLS[tool])

            if b and not old_b:
                motors()
                jog_start = now
                if tool == 0:
                    gripped = not gripped
                    servo(GRIP_CLOSED if gripped else GRIP_OPEN, GRIPPER)
                    show("Grip closed" if gripped else "Grip open")
                elif tool == 1:
                    raised = not raised
                    servo(LIFT_UP if raised else LIFT_DOWN, LIFT)
                    show("Lift up" if raised else "Lift down")
                else:
                    show(TOOLS[tool] + " (hold B)")

            left = right = feeder = launcher = 0
            if not b:  # Operate mechanisms while the chassis is stationary.
                if cyberpi.controller.is_press("up"):
                    left = right = DRIVE_POWER
                elif cyberpi.controller.is_press("down"):
                    left = right = -DRIVE_POWER
                elif cyberpi.controller.is_press("left"):
                    left, right = -DRIVE_POWER, DRIVE_POWER
                elif cyberpi.controller.is_press("right"):
                    left, right = DRIVE_POWER, -DRIVE_POWER

            # Release B, or reach the example timeout, to stop the jog.
            # A new jog requires releasing and pressing B again.
            if b and now - jog_start < MAX_JOG_SECONDS:
                if tool == 2:
                    feeder = FEED_POWER
                elif tool == 3:
                    feeder = -FEED_POWER
                elif tool == 4:
                    launcher = LAUNCH_POWER

            motors(left, right, feeder, launcher)
            old_a, old_b = a, b
            time.sleep(0.05)
    finally:
        motors()  # Motors off; servos retain their last commanded position.


if __name__ == "__main__":
    main()