"""Tests for the command conversion code in this module."""
import ezrassor_keyboard_controller as keyboard


def test_create_command_with_no_pressed_keys():
    """Should create a command that does nothing."""
    board = keyboard.Board()

    command = keyboard.create_command(board)

    assert command.routine_action is None
    assert command.wheel_action is not None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is keyboard.ArmAction.STOP
    assert command.back_arm_action is keyboard.ArmAction.STOP
    assert command.front_drum_action is keyboard.DrumAction.STOP
    assert command.back_drum_action is keyboard.DrumAction.STOP


def test_create_command_with_a_pressed_routine_key():
    """Should create a command that starts a routine."""
    board = keyboard.Board()
    board._pressed.add(keyboard.Key.AUTO_DRIVE_ROUTINE)

    command = keyboard.create_command(board)

    assert command.routine_action is keyboard.RoutineAction.AUTO_DRIVE
    assert command.wheel_action is not None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is keyboard.ArmAction.STOP
    assert command.back_arm_action is keyboard.ArmAction.STOP
    assert command.front_drum_action is keyboard.DrumAction.STOP
    assert command.back_drum_action is keyboard.DrumAction.STOP


def test_create_command_with_a_pressed_wheel_key():
    """Should create a command that moves the wheels."""
    board = keyboard.Board()
    board._pressed.add(keyboard.Key.MOVE_FORWARD)

    command = keyboard.create_command(board)

    assert command.routine_action is None
    assert command.wheel_action is not None
    assert command.wheel_action.linear_x == 1.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is keyboard.ArmAction.STOP
    assert command.back_arm_action is keyboard.ArmAction.STOP
    assert command.front_drum_action is keyboard.DrumAction.STOP
    assert command.back_drum_action is keyboard.DrumAction.STOP


def test_create_command_with_a_pressed_front_arm_key():
    """Should create a command that manipulates the front arm."""
    board = keyboard.Board()
    board._pressed.add(keyboard.Key.RAISE_FRONT_ARM)

    command = keyboard.create_command(board)

    assert command.routine_action is None
    assert command.wheel_action is not None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is keyboard.ArmAction.RAISE
    assert command.back_arm_action is keyboard.ArmAction.STOP
    assert command.front_drum_action is keyboard.DrumAction.STOP
    assert command.back_drum_action is keyboard.DrumAction.STOP


def test_create_command_with_a_pressed_back_arm_key():
    """Should create a command that manipulates the back arm."""
    board = keyboard.Board()
    board._pressed.add(keyboard.Key.RAISE_BACK_ARM)

    command = keyboard.create_command(board)

    assert command.routine_action is None
    assert command.wheel_action is not None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is keyboard.ArmAction.STOP
    assert command.back_arm_action is keyboard.ArmAction.RAISE
    assert command.front_drum_action is keyboard.DrumAction.STOP
    assert command.back_drum_action is keyboard.DrumAction.STOP


def test_create_command_with_a_pressed_front_drum_key():
    """Should create a command that manipulates the front drum."""
    board = keyboard.Board()
    board._pressed.add(keyboard.Key.DIG_FRONT_DRUM)

    command = keyboard.create_command(board)

    assert command.routine_action is None
    assert command.wheel_action is not None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is keyboard.ArmAction.STOP
    assert command.back_arm_action is keyboard.ArmAction.STOP
    assert command.front_drum_action is keyboard.DrumAction.DIG
    assert command.back_drum_action is keyboard.DrumAction.STOP


def test_create_command_with_a_pressed_back_drum_key():
    """Should create a command that manipulates the back drum."""
    board = keyboard.Board()
    board._pressed.add(keyboard.Key.DIG_BACK_DRUM)

    command = keyboard.create_command(board)

    assert command.routine_action is None
    assert command.wheel_action is not None
    assert command.wheel_action.linear_x == 0.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is keyboard.ArmAction.STOP
    assert command.back_arm_action is keyboard.ArmAction.STOP
    assert command.front_drum_action is keyboard.DrumAction.STOP
    assert command.back_drum_action is keyboard.DrumAction.DIG


def test_create_command_with_several_pressed_keys():
    """Should create a command that instructs the EZRASSOR to do many things.

    This test ensures that complex key presses are fully translated.
    """
    board = keyboard.Board()
    board._pressed.add(keyboard.Key.FULL_AUTONOMY_ROUTINE)
    board._pressed.add(keyboard.Key.MOVE_BACKWARD)
    board._pressed.add(keyboard.Key.RAISE_FRONT_ARM)
    board._pressed.add(keyboard.Key.LOWER_BACK_ARM)
    board._pressed.add(keyboard.Key.DIG_FRONT_DRUM)
    board._pressed.add(keyboard.Key.DUMP_BACK_DRUM)

    command = keyboard.create_command(board)

    assert command.routine_action is keyboard.RoutineAction.FULL_AUTONOMY
    assert command.wheel_action is not None
    assert command.wheel_action.linear_x == -1.0
    assert command.wheel_action.angular_z == 0.0
    assert command.front_arm_action is keyboard.ArmAction.RAISE
    assert command.back_arm_action is keyboard.ArmAction.LOWER
    assert command.front_drum_action is keyboard.DrumAction.DIG
    assert command.back_drum_action is keyboard.DrumAction.DUMP
