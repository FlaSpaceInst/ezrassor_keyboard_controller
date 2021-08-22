"""A stateful board object that holds a set of pressed keys."""
import enum
import threading


class Key(enum.Enum):
    """Available keys for the EZRASSOR."""

    MOVE_FORWARD = "w"
    MOVE_BACKWARD = "s"
    TURN_LEFT = "a"
    TURN_RIGHT = "d"
    RAISE_FRONT_ARM = "u"
    LOWER_FRONT_ARM = "j"
    RAISE_BACK_ARM = "i"
    LOWER_BACK_ARM = "k"
    DIG_FRONT_DRUM = "y"
    DUMP_FRONT_DRUM = "h"
    DIG_BACK_DRUM = "o"
    DUMP_BACK_DRUM = "l"
    STOP_ROUTINE = "0"
    AUTO_DRIVE_ROUTINE = "1"
    AUTO_DIG_ROUTINE = "2"
    AUTO_DUMP_ROUTINE = "3"
    AUTO_DOCK_ROUTINE = "4"
    FULL_AUTONOMY_ROUTINE = "5"


class Board:
    """A stateful board object that holds a set of pressed keys.

    This object is thread-safe.
    """

    def __init__(self):
        """Initialize this board."""
        self._pressed = set()
        self._lock = threading.Lock()

    def is_pressed(self, key):
        """Check if a key is currently pressed on this board.

        This method supports string keys and Key enumeration keys.
        """
        with self._lock:
            if isinstance(key, Key):
                return key in self._pressed
            else:
                try:
                    return Key(key) in self._pressed
                except ValueError:
                    return False

    def press(self, key):
        """Press a key on this board.

        Only keys within the Key enumeration are saved.
        """
        with self._lock:
            try:
                self._pressed.add(Key(key))
            except ValueError:
                pass

    def release(self, key):
        """Release a key on this board."""
        with self._lock:
            try:
                self._pressed.discard(Key(key))
            except ValueError:
                pass
