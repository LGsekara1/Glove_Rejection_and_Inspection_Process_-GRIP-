"""ODrive enum compatibility and project constants."""
from __future__ import annotations

try:  # ODrive 0.5.x
    from odrive.enums import (  # type: ignore
        AXIS_STATE_IDLE,
        AXIS_STATE_MOTOR_CALIBRATION,
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
        AXIS_STATE_CLOSED_LOOP_CONTROL,
        CONTROL_MODE_POSITION_CONTROL,
        CONTROL_MODE_VELOCITY_CONTROL,
        INPUT_MODE_PASSTHROUGH,
        INPUT_MODE_TRAP_TRAJ,
    )
except Exception:  # Allows core modules and simulator to run without ODrive installed.
    AXIS_STATE_IDLE = 1
    AXIS_STATE_MOTOR_CALIBRATION = 4
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8
    CONTROL_MODE_VELOCITY_CONTROL = 2
    CONTROL_MODE_POSITION_CONTROL = 3
    INPUT_MODE_PASSTHROUGH = 1
    INPUT_MODE_TRAP_TRAJ = 5

AXIS_STATE_NAMES: dict[int, str] = {
    0: "UNDEFINED",
    1: "IDLE",
    2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE",
    4: "MOTOR_CALIBRATION",
    5: "SENSORLESS_CONTROL",
    6: "ENCODER_INDEX_SEARCH",
    7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL",
    9: "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND",
    11: "HOMING",
    12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION",
}


def axis_state_name(value: int) -> str:
    return AXIS_STATE_NAMES.get(int(value), f"UNKNOWN_STATE_{int(value)}")

ENCODER_MODE_SPI_ABS_CUI = 0x100 + 0
ENCODER_MODE_SPI_ABS_AMS = 0x100 + 1
ENCODER_MODE_SPI_ABS_AEAT = 0x100 + 2
ENCODER_MODE_SPI_ABS_RLS = 0x100 + 3
ENCODER_MODE_SPI_ABS_MA732 = 0x100 + 5

ENCODER_MODE_LABELS: dict[str, int] = {
    "CUI AMT22x": ENCODER_MODE_SPI_ABS_CUI,
    "AMS (AS5047/AS5048)": ENCODER_MODE_SPI_ABS_AMS,
    "Broadcom AEAT": ENCODER_MODE_SPI_ABS_AEAT,
    "RLS": ENCODER_MODE_SPI_ABS_RLS,
    "MA732": ENCODER_MODE_SPI_ABS_MA732,
}
ENCODER_MODE_VALUES_TO_LABELS = {value: label for label, value in ENCODER_MODE_LABELS.items()}

FILTER_TYPES = (
    "None",
    "Moving Average",
    "Low-pass (1-pole)",
    "Butterworth",
    "Median",
)

DEFAULT_CONFIG_FILENAME = "five_bar_dashboard_config.json"
