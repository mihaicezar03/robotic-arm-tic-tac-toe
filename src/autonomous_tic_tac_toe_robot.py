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
USE_ROBOT = False      # Change to True on Raspberry Pi when the robot is connected.
SERVO_CHANNELS = [0, 1, 2, 3, 4, 5]
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 270  # Your coordinates include values above 180, so 270 is used.
MOVE_STEP_DEGREES = 2  # Smaller value = smoother/slower movement.
MOVE_STEP_DELAY = 0.025  # Seconds between interpolation steps.
PAUSE_BETWEEN_POSES = 0.15

if USE_ROBOT:
    from adafruit_servokit import ServoKit # type: ignore
    kit = ServoKit(channels=16)
    for channel in SERVO_CHANNELS:
        kit.servo[channel].actuation_range = SERVO_MAX_ANGLE
else:
    kit = None

# ----------------------------
# Robot coordinates
# Format: [servo0, servo1, servo2, servo3, servo4, servo5]
# ----------------------------
HOME_OPEN = [180, 20, 110, 180, 145, 140]
HOME_CLOSED = [180, 20, 110, 180, 145, 173]

PIECE_COORDS = {
    1: [
        [111, 15, 180, 140, 90, 140],
        [111, 0, 180, 140, 90, 140],
        [111, 0, 180, 140, 90, 173],
        [111, 30, 180, 140, 90, 173],
        [180, 30, 180, 140, 90, 173],
    ],
    2: [
        [111, 15, 180, 140, 90, 140],
        [128, 0, 180, 157, 100, 140],
        [128, 0, 180, 157, 100, 173],
        [128, 30, 180, 157, 100, 173],
        [190, 30, 180, 140, 140, 173],
    ],
    3: [
        [111, 15, 180, 140, 90, 140],
        [100, 0, 152, 165, 84, 140],
        [100, 0, 180, 165, 75, 173],
        [100, 30, 180, 140, 90, 173],
        [180, 30, 180, 140, 90, 173],
    ],
    4: [
        [270, 15, 180, 140, 74, 140],
        [270, 0, 180, 145, 74, 140],
        [270, 0, 180, 145, 74, 173],
        [270, 30, 180, 145, 74, 173],
        [180, 30, 180, 140, 90, 173],
    ],
    5: [
        [270, 15, 180, 140, 74, 140],
        [257, 0, 180, 160, 67, 140],
        [257, 0, 180, 160, 67, 173],
        [257, 30, 180, 160, 67, 173],
        [180, 30, 180, 140, 90, 173],
    ],
    6: [
        [270, 15, 180, 160, 67, 140],
        [270, 0, 180, 157, 67, 140],
        [270, 0, 180, 157, 67, 173],
        [270, 30, 180, 157, 67, 173],
        [180, 30, 180, 140, 90, 173],
    ],
}

BOARD_COORDS = {
    1: [[164, 35, 180, 128, 123, 173], [164, 14, 180, 121, 123, 173], [164, 6, 180, 124, 123, 140], [164, 36, 180, 124, 123, 140]],
    2: [[196, 20, 180, 124, 145, 173], [164, 3, 180, 128, 145, 173], [164, 3, 180, 128, 145, 140], [164, 36, 180, 124, 123, 140]],
    3: [[232, 20, 180, 125, 180, 173], [232, 0, 180, 128, 180, 173], [232, 0, 180, 128, 180, 140], [232, 36, 180, 128, 180, 140]],
    4: [[173, 20, 174, 140, 130, 173], [173, 0, 174, 140, 130, 173], [173, 0, 174, 140, 130, 140], [173, 30, 174, 140, 130, 140]],
    5: [[194, 20, 180, 140, 140, 173], [197, 4, 180, 140, 140, 173], [197, 4, 180, 140, 140, 140], [197, 30, 180, 140, 140, 140]],
    6: [[228, 20, 180, 140, 180, 173], [222, 3, 180, 138, 180, 173], [222, 3, 180, 138, 180, 140], [222, 30, 180, 138, 180, 140]],
    7: [[174, 30, 180, 160, 131, 173], [174, 0, 180, 160, 131, 173], [174, 0, 180, 160, 131, 140], [222, 30, 180, 138, 180, 140]],
    8: [[196, 30, 180, 160, 145, 173], [196, 0, 180, 160, 145, 173], [196, 0, 180, 160, 145, 140], [196, 30, 180, 160, 145, 140]],
    9: [[213, 30, 180, 160, 168, 173], [213, 0, 180, 162, 168, 173], [213, 0, 180, 162, 168, 140], [213, 30, 180, 162, 168, 140]],
}

current_angles = HOME_OPEN.copy()
robot_busy = False
piece_number = 1


def clamp_angle(angle):
    return max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))


def write_servo_angles(angles):
    """Send one complete 6-servo pose to the servo driver."""
    for servo_index, angle in enumerate(angles):
        angle = clamp_angle(angle)
        channel = SERVO_CHANNELS[servo_index]
        if USE_ROBOT:
            kit.servo[channel].angle = angle
        else:
            print(f"Servo {servo_index} -> {angle}")


def move_slowly_to(target_angles):
    """Interpolates from current_angles to target_angles for smoother movement."""
    global current_angles

    target_angles = [clamp_angle(angle) for angle in target_angles]
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
# Tic Tac Toe logic
# ----------------------------
playerX = "X"   # human
playerO = "O"   # robot/computer
curr_player = playerX
turns = 0
game_over = False

# Probability that the robot makes a random move after its first O move.
# Set to 0.0 if you want the robot to always play optimally.
MISTAKE_PROBABILITY = 0.35


def gui_cell_to_board_number(row, column):
    """
    Converts GUI row/column to your real board numbering:
        row 0: 7, 8, 9
        row 1: 4, 5, 6
        row 2: 1, 2, 3
    """
    return (2 - row) * 3 + (column + 1)


def board_to_state():
    state = []
    for row in range(3):
        for column in range(3):
            text = board[row][column]["text"]
            if text == playerX:
                state.append(-1)
            elif text == playerO:
                state.append(1)
            else:
                state.append(0)
    return state


def analyzeboard(state):
    win_combinations = [
        [0, 1, 2], [3, 4, 5], [6, 7, 8],
        [0, 3, 6], [1, 4, 7], [2, 5, 8],
        [0, 4, 8], [2, 4, 6]
    ]

    for combo in win_combinations:
        if state[combo[0]] != 0 and state[combo[0]] == state[combo[1]] == state[combo[2]]:
            return state[combo[0]]
    return 0


def minimax(state, player):
    result = analyzeboard(state)
    if result != 0:
        return result * player

    best_value = -2
    best_pos = -1

    for i in range(9):
        if state[i] == 0:
            state[i] = player
            score = -minimax(state, -player)
            state[i] = 0
            if score > best_value:
                best_value = score
                best_pos = i

    if best_pos == -1:
        return 0
    return best_value


def get_best_move(state):
    empty_positions = [i for i in range(9) if state[i] == 0]
    if not empty_positions:
        return -1

    o_moves = sum(1 for value in state if value == 1)
    if o_moves >= 1 and random.random() < MISTAKE_PROBABILITY:
        return random.choice(empty_positions)

    best_value = -2
    best_pos = -1

    for i in empty_positions:
        state[i] = 1
        score = -minimax(state, -1)
        state[i] = 0
        if score > best_value:
            best_value = score
            best_pos = i

    return best_pos


def highlight_winner(cells, winner_text):
    label.config(text=winner_text + " is the winner!", foreground=color_yellow)
    for row, column in cells:
        board[row][column].config(foreground=color_yellow, background=color_light_gray)


def check_winner():
    global game_over

    for row in range(3):
        if board[row][0]["text"] == board[row][1]["text"] == board[row][2]["text"] and board[row][0]["text"] != "":
            highlight_winner([(row, 0), (row, 1), (row, 2)], board[row][0]["text"])
            game_over = True
            return

    for column in range(3):
        if board[0][column]["text"] == board[1][column]["text"] == board[2][column]["text"] and board[0][column]["text"] != "":
            highlight_winner([(0, column), (1, column), (2, column)], board[0][column]["text"])
            game_over = True
            return

    if board[0][0]["text"] == board[1][1]["text"] == board[2][2]["text"] and board[0][0]["text"] != "":
        highlight_winner([(0, 0), (1, 1), (2, 2)], board[0][0]["text"])
        game_over = True
        return

    if board[0][2]["text"] == board[1][1]["text"] == board[2][0]["text"] and board[0][2]["text"] != "":
        highlight_winner([(0, 2), (1, 1), (2, 0)], board[0][2]["text"])
        game_over = True
        return

    if turns == 9:
        game_over = True
        label.config(text="Tie!", foreground=color_yellow)


def set_buttons_state(state):
    for row in range(3):
        for column in range(3):
            board[row][column].config(state=state)
    restart_button.config(state=state)


def finish_robot_turn():
    global curr_player, robot_busy
    robot_busy = False
    set_buttons_state("normal")
    if not game_over:
        curr_player = playerX
        label.config(text=curr_player + "'s turn", foreground="white")


def robot_thread_task(selected_piece, selected_board_position):
    try:
        robot_pick_and_place(selected_piece, selected_board_position)
    except Exception as error:
        window.after(0, lambda: messagebox.showerror("Robot error", str(error)))
    finally:
        window.after(0, finish_robot_turn)


def computer_turn():
    global curr_player, turns, piece_number, robot_busy

    if game_over or robot_busy:
        return

    state = board_to_state()
    move = get_best_move(state)
    if move == -1:
        return

    row = move // 3
    column = move % 3
    board_position_number = gui_cell_to_board_number(row, column)

    board[row][column]["text"] = playerO
    turns += 1
    check_winner()

    selected_piece = piece_number
    piece_number += 1

    robot_busy = True
    set_buttons_state("disabled")
    label.config(text=f"Robot moving: piece {selected_piece} -> board {board_position_number}", foreground="white")

    thread = threading.Thread(
        target=robot_thread_task,
        args=(selected_piece, board_position_number),
        daemon=True
    )
    thread.start()


def set_tile(row, column):
    global curr_player, turns

    if game_over or robot_busy or curr_player != playerX:
        return

    if board[row][column]["text"] != "":
        return

    board[row][column]["text"] = playerX
    turns += 1
    check_winner()

    if game_over:
        return

    curr_player = playerO
    label.config(text=curr_player + "'s turn", foreground="white")
    window.after(250, computer_turn)


def new_game():
    global turns, game_over, curr_player, piece_number, robot_busy

    if robot_busy:
        messagebox.showwarning("Robot busy", "Wait until the robot finishes the current movement.")
        return

    turns = 0
    game_over = False
    curr_player = playerX
    piece_number = 1
    robot_busy = False

    label.config(text=curr_player + "'s turn", foreground="white")
    set_buttons_state("normal")

    for row in range(3):
        for column in range(3):
            board[row][column].config(text="", foreground=color_blue, background=color_gray)

    threading.Thread(target=lambda: robot_pick_and_place_home_only(), daemon=True).start()


def robot_pick_and_place_home_only():
    try:
        run_pose_sequence([HOME_OPEN])
    except Exception:
        pass


# ----------------------------
# GUI setup
# ----------------------------
color_blue = "#4584b6"
color_yellow = "#ffde57"
color_gray = "#343434"
color_light_gray = "#646464"

window = tk.Tk()
window.title("Tic Tac Toe - X and O Robot")
window.resizable(False, False)

frame = tk.Frame(window)
label = tk.Label(
    frame,
    text=curr_player + "'s turn",
    font=("Consolas", 18),
    background=color_gray,
    foreground="white"
)
label.grid(row=0, column=0, columnspan=3, sticky="we")

board = [[None, None, None], [None, None, None], [None, None, None]]
for row in range(3):
    for column in range(3):
        board[row][column] = tk.Button(
            frame,
            text="",
            font=("Consolas", 50, "bold"),
            background=color_gray,
            foreground=color_blue,
            width=4,
            height=1,
            command=lambda row=row, column=column: set_tile(row, column)
        )
        board[row][column].grid(row=row + 1, column=column)

restart_button = tk.Button(
    frame,
    text="restart",
    font=("Consolas", 20),
    background=color_gray,
    foreground="white",
    command=new_game
)
restart_button.grid(row=4, column=0, columnspan=3, sticky="we")
frame.pack()

window.update()
window_width = window.winfo_width()
window_height = window.winfo_height()
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()
window_x = int((screen_width / 2) - (window_width / 2))
window_y = int((screen_height / 2) - (window_height / 2))
window.geometry(f"{window_width}x{window_height}+{window_x}+{window_y}")

# Move the robot to the initial safe pose when the software starts.
threading.Thread(target=lambda: run_pose_sequence([HOME_OPEN]), daemon=True).start()

window.mainloop()
