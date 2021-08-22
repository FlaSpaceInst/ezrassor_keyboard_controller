"""Data structures that make up an EZRASSOR command."""
import enum
import ezrassor_keyboard_controller as keyboard


def create_command(board):
    """Create a command from a board."""

    # Set the routine action.
    if board.is_pressed(keyboard.Key.STOP_ROUTINE):
        routine_action = RoutineAction.STOP
    elif board.is_pressed(keyboard.Key.AUTO_DRIVE_ROUTINE):
        routine_action = RoutineAction.AUTO_DRIVE
    elif board.is_pressed(keyboard.Key.AUTO_DIG_ROUTINE):
        routine_action = RoutineAction.AUTO_DIG
    elif board.is_pressed(keyboard.Key.AUTO_DUMP_ROUTINE):
        routine_action = RoutineAction.AUTO_DUMP
    elif board.is_pressed(keyboard.Key.AUTO_DOCK_ROUTINE):
        routine_action = RoutineAction.AUTO_DOCK
    elif board.is_pressed(keyboard.Key.FULL_AUTONOMY_ROUTINE):
        routine_action = RoutineAction.FULL_AUTONOMY
    else:
        routine_action = None

    # Set the wheel action.
    if board.is_pressed(keyboard.Key.MOVE_FORWARD):
        wheel_action = WheelAction(linear_x=1.0, angular_z=0.0)
    elif board.is_pressed(keyboard.Key.MOVE_BACKWARD):
        wheel_action = WheelAction(linear_x=-1.0, angular_z=0.0)
    elif board.is_pressed(keyboard.Key.TURN_LEFT):
        wheel_action = WheelAction(linear_x=0.0, angular_z=1.0)
    elif board.is_pressed(keyboard.Key.TURN_RIGHT):
        wheel_action = WheelAction(linear_x=0.0, angular_z=-1.0)
    else:
        wheel_action = WheelAction(linear_x=0.0, angular_z=0.0)

    # Set the front arm action.
    if board.is_pressed(keyboard.Key.RAISE_FRONT_ARM):
        front_arm_action = ArmAction.RAISE
    elif board.is_pressed(keyboard.Key.LOWER_FRONT_ARM):
        front_arm_action = ArmAction.LOWER
    else:
        front_arm_action = ArmAction.STOP

    # Set the back arm action.
    if board.is_pressed(keyboard.Key.RAISE_BACK_ARM):
        back_arm_action = ArmAction.RAISE
    elif board.is_pressed(keyboard.Key.LOWER_BACK_ARM):
        back_arm_action = ArmAction.LOWER
    else:
        back_arm_action = ArmAction.STOP

    # Set the front drum action.
    if board.is_pressed(keyboard.Key.DIG_FRONT_DRUM):
        front_drum_action = DrumAction.DIG
    elif board.is_pressed(keyboard.Key.DUMP_FRONT_DRUM):
        front_drum_action = DrumAction.DUMP
    else:
        front_drum_action = DrumAction.STOP

    # Set the back drum action.
    if board.is_pressed(keyboard.Key.DIG_BACK_DRUM):
        back_drum_action = DrumAction.DIG
    elif board.is_pressed(keyboard.Key.DUMP_BACK_DRUM):
        back_drum_action = DrumAction.DUMP
    else:
        back_drum_action = DrumAction.STOP

    return Command(
        wheel_action,
        front_arm_action,
        back_arm_action,
        front_drum_action,
        back_drum_action,
        routine_action,
    )


class Command:
    """A command containing actions for an EZRASSOR."""

    def __init__(
        self,
        wheel_action,
        front_arm_action,
        back_arm_action,
        front_drum_action,
        back_drum_action,
        routine_action,
    ):
        """Initialize this command with actions."""
        self.wheel_action = wheel_action
        self.front_arm_action = front_arm_action
        self.back_arm_action = back_arm_action
        self.front_drum_action = front_drum_action
        self.back_drum_action = back_drum_action
        self.routine_action = routine_action


class WheelAction:
    """This action describes how to move the wheels of an EZRASSOR."""

    def __init__(self, linear_x, angular_z):
        """Initialize this action with movement floats."""
        self.linear_x = linear_x
        self.angular_z = angular_z


class ArmAction(enum.Enum):
    """This action describes how to move the arms of an EZRASSOR."""

    LOWER = -1.0
    STOP = 0.0
    RAISE = 1.0


class DrumAction(enum.Enum):
    """This action describes how to move the drums of an EZRASSOR."""

    DUMP = -1.0
    STOP = 0.0
    DIG = 1.0


class RoutineAction(enum.Enum):
    """This action describes which routine to execute for an EZRASSOR."""

    AUTO_DRIVE = 0b000001
    AUTO_DIG = 0b000010
    AUTO_DUMP = 0b000100
    AUTO_DOCK = 0b001000
    FULL_AUTONOMY = 0b010000
    STOP = 0b100000
