"""
Autonomous Tic Tac Toe game for a 6-DOF robotic arm.
Human player: X
Robot/computer player: O

The GUI uses the same board numbering as the real board:
    7 8 9
    4 5 6
    1 2 3

Set USE_ROBOT = True on the Raspberry Pi after checking all servo limits.
Set USE_ROBOT = False to test the GUI and movement sequence without hardware.
"""

import random
import threading
import time
import tkinter as tk
from tkinter import messagebox

# ----------------------------
# Hardware configuration
# ----------------------------
USE_ROBOT = True       # Set to False only when testing without the physical robot.
SERVO_CHANNELS = [0, 1, 2, 3, 4, 5]

# These settings copy the manual controller calibration used to teach the coordinates.
# Only servo 0 is configured as a 270-degree servo. Servos 1-5 remain at the
# default ServoKit 180-degree range. The gripper is servo 5.
SERVO_LIMITS = {
    0: (0, 270),
    1: (0, 180),
    2: (0, 180),
    3: (0, 180),
    4: (0, 180),
    5: (90, 180),
}

MOVE_STEP_DEGREES = 1      # Smaller value = smoother/slower movement.
MOVE_STEP_DELAY = 0.04     # Seconds between interpolation steps.
PAUSE_BETWEEN_POSES = 0.20

if USE_ROBOT:
    from adafruit_servokit import ServoKit
    kit = ServoKit(channels=16)
    kit.servo[0].actuation_range = 270
    kit.servo[0].set_pulse_width_range(500, 2500)
else:
    kit = None

# ----------------------------
# Robot coordinates
# Format: [servo0, servo1, servo2, servo3, servo4, servo5]
# ----------------------------
HOME_OPEN = [180, 20, 110, 180, 145, 120]
HOME_CLOSED = [180, 20, 110, 180, 145, 173]

PIECE_COORDS = {
    1: [
        [111, 15, 180, 140, 90, 120],
        [111, 0, 180, 140, 90, 120],
        [111, 0, 180, 140, 90, 173],
        [111, 30, 180, 140, 90, 173],
        [180, 30, 180, 140, 90, 173],
    ],
    2: [
        [111, 15, 180, 140, 90, 120],
        [128, 0, 180, 157, 100, 120],
        [128, 0, 180, 157, 100, 173],
        [128, 30, 180, 157, 100, 173],
        [190, 30, 180, 140, 140, 173],
    ],
    3: [
        [111, 15, 180, 140, 90, 120],
        [100, 0, 152, 165, 84, 120],
        [100, 0, 180, 165, 75, 173],
        [100, 30, 180, 140, 90, 173],
        [180, 30, 180, 140, 90, 173],
    ],
    4: [
        [270, 15, 180, 140, 74, 120],
        [270, 0, 180, 145, 74, 120],
        [270, 0, 180, 145, 74, 173],
        [270, 30, 180, 145, 74, 173],
        [180, 30, 180, 140, 90, 173],
    ],
    5: [
        [270, 15, 180, 140, 74, 120],
        [257, 0, 180, 160, 67, 120],
        [257, 0, 180, 160, 67, 173],
        [257, 30, 180, 160, 67, 173],
        [180, 30, 180, 140, 90, 173],
    ],
    6: [
        [270, 15, 180, 160, 67, 120],
        [270, 0, 180, 157, 67, 120],
        [270, 0, 180, 157, 67, 173],
        [270, 30, 180, 157, 67, 173],
        [180, 30, 180, 140, 90, 173],
    ],
}

BOARD_COORDS = {
    1: [[164, 35, 180, 128, 123, 173], [164, 14, 180, 121, 123, 173], [164, 6, 180, 124, 123, 120], [164, 36, 180, 124, 123, 120]],
    2: [[196, 20, 180, 124, 145, 173], [164, 3, 180, 128, 145, 173], [164, 3, 180, 128, 145, 120], [164, 36, 180, 124, 123, 120]],
    3: [[232, 20, 180, 125, 180, 173], [232, 0, 180, 128, 180, 173], [232, 0, 180, 128, 180, 120], [232, 36, 180, 128, 180, 120]],
    4: [[173, 20, 174, 140, 130, 173], [173, 0, 174, 140, 130, 173], [173, 0, 174, 140, 130, 120], [173, 30, 174, 140, 130, 120]],
    5: [[194, 20, 180, 140, 140, 173], [197, 4, 180, 140, 140, 173], [197, 4, 180, 140, 140, 120], [197, 30, 180, 140, 140, 120]],
    6: [[228, 20, 180, 140, 180, 173], [222, 3, 180, 138, 180, 173], [222, 3, 180, 138, 180, 120], [222, 30, 180, 138, 180, 120]],
    7: [[174, 30, 180, 160, 131, 173], [174, 0, 180, 160, 131, 173], [174, 0, 180, 160, 131, 120], [222, 30, 180, 138, 180, 120]],
    8: [[196, 30, 180, 160, 145, 173], [196, 0, 180, 160, 145, 173], [196, 0, 180, 160, 145, 120], [196, 30, 180, 160, 145, 120]],
    9: [[213, 30, 180, 160, 168, 173], [213, 0, 180, 162, 168, 173], [213, 0, 180, 162, 168, 120], [213, 30, 180, 162, 168, 120]],
}

current_angles = HOME_OPEN.copy()
robot_busy = False
piece_number = 1


def clamp_angle(servo_index, angle):
    low, high = SERVO_LIMITS[servo_index]
    return max(low, min(high, angle))


def write_servo_angles(angles):
    """Send one complete 6-servo pose to the servo driver."""
    for servo_index, angle in enumerate(angles):
        angle = clamp_angle(servo_index, angle)
        channel = SERVO_CHANNELS[servo_index]
        if USE_ROBOT:
            kit.servo[channel].angle = angle
        else:
            print(f"Servo {servo_index} -> {angle}")


def move_slowly_to(target_angles):
    """Interpolates from current_angles to target_angles for smoother movement."""
    global current_angles

    target_angles = [clamp_angle(i, angle) for i, angle in enumerate(target_angles)]
    max_delta = max(abs(target_angles[i] - current_angles[i]) for i in range(6))
    steps = max(1, int(max_delta / MOVE_STEP_DEGREES))

    for step in range(1, steps + 1):
        intermediate = []
        for i in range(6):
            start = current_angles[i]
            end = target_angles[i]
            value = start + (end - start) * step / steps
            intermediate.append(round(value, 2))
        write_servo_angles(intermediate)
        time.sleep(MOVE_STEP_DELAY)

    current_angles = target_angles.copy()
    time.sleep(PAUSE_BETWEEN_POSES)


def run_pose_sequence(sequence):
    for pose in sequence:
        move_slowly_to(pose)


def robot_pick_and_place(piece_id, board_id):
    """
    Full physical cycle:
    1. Move to selected O piece.
    2. Pick it.
    3. Move above the table.
    4. Move to selected board cell.
    5. Release the piece.
    6. Return to safe/home position.
    """
    if piece_id not in PIECE_COORDS:
        raise ValueError(f"No coordinates defined for piece #{piece_id}.")
    if board_id not in BOARD_COORDS:
        raise ValueError(f"No coordinates defined for board #{board_id}.")

    run_pose_sequence([HOME_OPEN])
    run_pose_sequence(PIECE_COORDS[piece_id])
    run_pose_sequence(BOARD_COORDS[board_id])
    run_pose_sequence([HOME_OPEN])




# ----------------------------
# Manual coordinate testing menu
# ----------------------------
STEP_NAMES_PIECE = [
    "1 Above",
    "2 Pick position",
    "3 Pick / close gripper",
    "4 Above after pick",
    "5 Above table",
]

STEP_NAMES_BOARD = [
    "1 Above",
    "2 Place",
    "3 Place down / open gripper",
    "4 Above safe",
]


def print_pose(label, pose):
    print(f"\n{label}")
    print(f"Angles: {pose}")


def run_named_sequence(title, sequence, step_names, return_home=False):
    print(f"\nRunning: {title}")
    for i, pose in enumerate(sequence):
        name = step_names[i] if i < len(step_names) else f"Step {i + 1}"
        print_pose(name, pose)
        move_slowly_to(pose)
        input("Press ENTER for next step...")
    if return_home:
        print_pose("Returning to HOME_OPEN", HOME_OPEN)
        move_slowly_to(HOME_OPEN)
    print("Done.\n")


def test_piece(piece_id):
    if piece_id not in PIECE_COORDS:
        print(f"Invalid piece number: {piece_id}. Use 1-6.")
        return
    run_named_sequence(
        title=f"Piece #{piece_id} full 5-step sequence",
        sequence=PIECE_COORDS[piece_id],
        step_names=STEP_NAMES_PIECE,
        return_home=True,
    )


def test_board(board_id):
    if board_id not in BOARD_COORDS:
        print(f"Invalid board number: {board_id}. Use 1-9.")
        return
    run_named_sequence(
        title=f"Board #{board_id} full 4-step sequence",
        sequence=BOARD_COORDS[board_id],
        step_names=STEP_NAMES_BOARD,
        return_home=True,
    )


def test_single_pose(kind, item_id, step_id):
    if kind == "p":
        coords = PIECE_COORDS
        names = STEP_NAMES_PIECE
        max_step = 5
        label = "Piece"
    else:
        coords = BOARD_COORDS
        names = STEP_NAMES_BOARD
        max_step = 4
        label = "Board"

    if item_id not in coords:
        print(f"Invalid {label.lower()} number: {item_id}.")
        return
    if step_id < 1 or step_id > max_step:
        print(f"Invalid step number. Use 1-{max_step}.")
        return

    pose = coords[item_id][step_id - 1]
    print_pose(f"{label} #{item_id} - {names[step_id - 1]}", pose)
    move_slowly_to(pose)
    print("Done.\n")


def go_home(open_gripper=True):
    pose = HOME_OPEN if open_gripper else HOME_CLOSED
    print_pose("HOME_OPEN" if open_gripper else "HOME_CLOSED", pose)
    move_slowly_to(pose)


def print_menu():
    print("""
================ ROBOT POSITION TESTER ================
Commands:
  p1 ... p6        Run full 5-step sequence for piece 1-6
  b1 ... b9        Run full 4-step sequence for board position 1-9

  ps              Run all pieces 1-6
  bs              Run all board positions 1-9

  step p 3 2      Go only to Piece #3, step 2
  step b 5 4      Go only to Board #5, step 4

  home            Go to HOME_OPEN
  closed          Go to HOME_CLOSED
  q               Quit
=======================================================
""")


def main():
    print("Starting robot position tester...")
    print("Using same servo calibration as the manual controller.")
    print("Initial move to HOME_OPEN.")
    go_home(open_gripper=True)

    while True:
        print_menu()
        cmd = input("Command: ").strip().lower()

        if cmd in {"q", "quit", "exit"}:
            print("Exiting. Returning to HOME_OPEN.")
            go_home(open_gripper=True)
            break

        if cmd == "home":
            go_home(open_gripper=True)
            continue

        if cmd == "closed":
            go_home(open_gripper=False)
            continue

        if cmd == "ps":
            for piece_id in range(1, 7):
                test_piece(piece_id)
            continue

        if cmd == "bs":
            for board_id in range(1, 10):
                test_board(board_id)
            continue

        if len(cmd) >= 2 and cmd[0] == "p" and cmd[1:].isdigit():
            test_piece(int(cmd[1:]))
            continue

        if len(cmd) >= 2 and cmd[0] == "b" and cmd[1:].isdigit():
            test_board(int(cmd[1:]))
            continue

        parts = cmd.split()
        if len(parts) == 4 and parts[0] == "step":
            kind = parts[1]
            if kind not in {"p", "b"}:
                print("Use 'p' for piece or 'b' for board.")
                continue
            if not parts[2].isdigit() or not parts[3].isdigit():
                print("Use numbers for item and step.")
                continue
            test_single_pose(kind, int(parts[2]), int(parts[3]))
            continue

        print("Unknown command.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted. Returning to HOME_OPEN.")
        go_home(open_gripper=True)
