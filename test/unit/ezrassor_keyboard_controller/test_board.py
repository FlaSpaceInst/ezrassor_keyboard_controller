"""Tests for the stateful board object in this module."""
import ezrassor_keyboard_controller as keyboard


def test_board_saves_supported_pressed_keys():
    """Should save pressed keys that are in the Key enumeration."""
    board = keyboard.Board()

    board.press(keyboard.Key.MOVE_FORWARD.value)
    board.press(keyboard.Key.RAISE_FRONT_ARM.value)
    board.press(keyboard.Key.AUTO_DIG_ROUTINE.value)

    assert board.is_pressed(keyboard.Key.MOVE_FORWARD.value)
    assert board.is_pressed(keyboard.Key.RAISE_FRONT_ARM.value)
    assert board.is_pressed(keyboard.Key.AUTO_DIG_ROUTINE.value)


def test_board_ignores_unsupported_pressed_keys():
    """Should ignore pressed keys that are not in the Key enumeration."""
    board = keyboard.Board()

    board.press("!")
    board.press("@")
    board.press("#")

    assert not board.is_pressed("!")
    assert not board.is_pressed("@")
    assert not board.is_pressed("#")


def test_board_forgets_released_keys():
    """Should forget released keys."""
    board = keyboard.Board()

    board.press(keyboard.Key.MOVE_FORWARD.value)
    board.press(keyboard.Key.RAISE_FRONT_ARM.value)
    board.press(keyboard.Key.AUTO_DIG_ROUTINE.value)
    board.release(keyboard.Key.MOVE_FORWARD.value)
    board.release(keyboard.Key.RAISE_FRONT_ARM.value)
    board.release(keyboard.Key.AUTO_DIG_ROUTINE.value)

    assert not board.is_pressed(keyboard.Key.MOVE_FORWARD.value)
    assert not board.is_pressed(keyboard.Key.RAISE_FRONT_ARM.value)
    assert not board.is_pressed(keyboard.Key.AUTO_DIG_ROUTINE.value)


def test_board_can_save_and_forget_a_key_several_times():
    """Should support pressing and releasing a key over and over."""
    board = keyboard.Board()

    board.press(keyboard.Key.MOVE_FORWARD.value)
    assert board.is_pressed(keyboard.Key.MOVE_FORWARD.value)

    board.release(keyboard.Key.MOVE_FORWARD.value)
    assert not board.is_pressed(keyboard.Key.MOVE_FORWARD.value)

    board.press(keyboard.Key.MOVE_FORWARD.value)
    assert board.is_pressed(keyboard.Key.MOVE_FORWARD.value)

    board.release(keyboard.Key.MOVE_FORWARD.value)
    assert not board.is_pressed(keyboard.Key.MOVE_FORWARD.value)

    board.press(keyboard.Key.MOVE_FORWARD.value)
    assert board.is_pressed(keyboard.Key.MOVE_FORWARD.value)

    board.release(keyboard.Key.MOVE_FORWARD.value)
    assert not board.is_pressed(keyboard.Key.MOVE_FORWARD.value)


def test_board_can_check_both_stringified_and_enumerated_key_arguments():
    """Should support checking if a key is pressed in different forms.

    This method is designed to support a stringified key name or a Key
    enumeration value.
    """
    board = keyboard.Board()

    board.press(keyboard.Key.MOVE_FORWARD.value)

    assert board.is_pressed(keyboard.Key.MOVE_FORWARD)
    assert board.is_pressed(keyboard.Key.MOVE_FORWARD.value)
