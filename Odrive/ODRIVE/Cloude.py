
import odrive
from odrive.enums import *
import time

# ---- State constants (for reference / fallback if enums import differs) ----
AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_HOMING = 11


def wait_for_idle(axis, timeout=30):
    """Wait until axis returns to IDLE, or timeout."""
    start = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start > timeout:
            print("  -> TIMEOUT waiting for IDLE")
            return False
        time.sleep(0.1)
    return True


def check_errors(axis, axis_name):
    """Print and return whether axis/motor/encoder have errors."""
    has_error = (axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0)
    if has_error:
        print(f"  [{axis_name}] ERROR -> axis.error={axis.error}, "
              f"motor.error={axis.motor.error}, encoder.error={axis.encoder.error}")
    else:
        print(f"  [{axis_name}] OK, no errors")
    return not has_error


def run_state(axis, axis_name, state, state_name, timeout=30):
    """Request a state, wait for completion, report errors."""
    print(f"[{axis_name}] Requesting {state_name} ...")
    axis.requested_state = state
    time.sleep(0.2)  # give firmware a moment to actually leave IDLE
    ok_timing = wait_for_idle(axis, timeout=timeout)
    ok_errors = check_errors(axis, axis_name)
    return ok_timing and ok_errors


def main():
    print("Connecting to ODrive...")
    odrv0 = odrive.find_any()
    print("Connected.")

    print("Clearing errors...")
    odrv0.clear_errors()
    time.sleep(0.2)

    axes = [("axis0", odrv0.axis0), ("axis1", odrv0.axis1)]

    # ---- 1. Motor calibration (only needed if not already calibrated) ----
    for name, axis in axes:
        if not axis.motor.is_calibrated:
            ok = run_state(axis, name, AXIS_STATE_MOTOR_CALIBRATION, "MOTOR_CALIBRATION")
            if not ok:
                print(f"[{name}] Motor calibration failed. Aborting.")
                return
        else:
            print(f"[{name}] Motor already calibrated, skipping.")

    # ---- 2. Encoder offset calibration (only needed if not already calibrated) ----
    for name, axis in axes:
        if not axis.encoder.is_ready:
            ok = run_state(axis, name, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, "ENCODER_OFFSET_CALIBRATION")
            if not ok:
                print(f"[{name}] Encoder calibration failed. Aborting.")
                return
        else:
            print(f"[{name}] Encoder already calibrated, skipping.")

    # ---- 3. Sanity check before homing ----
    all_ready = True
    for name, axis in axes:
        if not axis.motor.is_calibrated or not axis.encoder.is_ready:
            print(f"[{name}] Not ready for homing (motor.is_calibrated="
                  f"{axis.motor.is_calibrated}, encoder.is_ready={axis.encoder.is_ready})")
            all_ready = False

    if not all_ready:
        print("Aborting before homing due to unmet calibration state.")
        return

    # ---- 4. Homing ----
    print("\nStarting homing on both axes...")
    odrv0.axis0.requested_state = AXIS_STATE_HOMING
    odrv0.axis1.requested_state = AXIS_STATE_HOMING
    time.sleep(0.2)

    ok0 = wait_for_idle(odrv0.axis0, timeout=60)
    ok1 = wait_for_idle(odrv0.axis1, timeout=60)

    print("\n--- Post-homing status ---")
    ok0_err = check_errors(odrv0.axis0, "axis0")
    ok1_err = check_errors(odrv0.axis1, "axis1")

    if ok0 and ok1 and ok0_err and ok1_err:
        print("\nHoming completed successfully on both axes.")
    else:
        print("\nHoming did NOT complete cleanly. See errors above.")

    # Optional: use odrivetool's error dumper for human-readable decoded errors
    try:
        from odrive.utils import dump_errors
        print("\n--- Decoded errors ---")
        dump_errors(odrv0)
    except ImportError:
        pass


if __name__ == "__main__":
    main()