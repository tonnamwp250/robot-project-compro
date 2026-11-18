import gamepad
from cyberpi import *
console.println('start')
while True:
    fwd = gamepad.get_joystick('Ly')   # เดินหน้า/ถอยหลัง (จอยซ้าย)
    turn = gamepad.get_joystick('Rx')  # หมุนซ้าย/ขวา (จอยขวา)

    left = (fwd + turn) * 2
    right = (fwd - turn) * 2

    # จำกัดความเร็วไม่ให้เกินช่วงที่รับได้ (-200 ถึง 200)
    left = max(-200, min(200, left))
    right = max(-200, min(200, right))

    mbot2.drive_speed(left, -right)

    if gamepad.is_key_pressed('N1'):
        mbot2.servo_set(90,'s3')
    elif gamepad.is_key_pressed('N2'):
        mbot2.servo_set(0,'s3')
    elif gamepad.is_key_pressed('N3'):
        mbot2.servo_set(90,'s4')
    elif gamepad.is_key_pressed('N4'):
        mbot2.servo_set(0,'s4')