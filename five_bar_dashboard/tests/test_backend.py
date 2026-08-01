from five_bar_dashboard.backend import ODriveManager
from five_bar_dashboard.constants import AXIS_STATE_IDLE


def test_idle_axis_snapshot_keeps_encoder_position_available() -> None:
    manager = ODriveManager(simulate=True)
    drive = manager.connect()
    drive.axis0._pos = 1.234567
    drive.axis1._pos = -0.765432
    drive.axis0.requested_state = AXIS_STATE_IDLE
    drive.axis1.requested_state = AXIS_STATE_IDLE

    with manager.access() as locked_drive:
        axis0 = manager.read_axis_snapshot_locked(locked_drive, 0)
        axis1 = manager.read_axis_snapshot_locked(locked_drive, 1)

    assert axis0.current_state == AXIS_STATE_IDLE
    assert axis1.current_state == AXIS_STATE_IDLE
    assert axis0.pos_turns == 1.234567
    assert axis1.pos_turns == -0.765432


def test_structured_error_report_opens_even_when_no_errors() -> None:
    manager = ODriveManager(simulate=True)
    drive = manager.connect()
    with manager.access() as locked_drive:
        report = manager.read_error_report_locked(locked_drive)

    assert report["has_errors"] is False
    assert "Summary: no active errors" in report["formatted_text"]
    assert report["axes"]["axis0"]["state_name"] == "IDLE"

    drive.axis0.error = 0x800
    with manager.access() as locked_drive:
        report = manager.read_error_report_locked(locked_drive)
    assert report["has_errors"] is True
    assert "0x00000800" in report["formatted_text"]
