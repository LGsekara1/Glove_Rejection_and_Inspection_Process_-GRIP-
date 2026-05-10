import json
import serial
import time

SERIAL_PORT = "COM5"
BAUD = 115200

class ScaraSender:
    def __init__(self, port=SERIAL_PORT, baud=BAUD, enabled=False):
        self.enabled = enabled
        self.ser = None
        if enabled:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)

    def send_pick(self, decision, track_id, bin_name, x=None, y=None):
        msg = {
            "cmd": "pick",
            "track_id": int(track_id),
            "decision": decision,
            "bin": bin_name,
            "x": None if x is None else float(x),
            "y": None if y is None else float(y),
            "timestamp": time.time()
        }
        line = json.dumps(msg) + "\n"
        if self.enabled and self.ser is not None:
            self.ser.write(line.encode("utf-8"))
        else:
            print("[SIM SCARA]", line.strip())

    def close(self):
        if self.ser is not None:
            self.ser.close()

def decision_to_bin(decision):
    if "MANUAL" in decision:
        return "manual_bin"
    if "RIGHT" in decision:
        return "wrong_hand_bin"
    if "LABEL_DEFECT" in decision:
        return "defect_bin"
    if "WRONG_SIZE" in decision:
        return "wrong_size_bin"
    return "unknown_bin"

if __name__ == "__main__":
    sender = ScaraSender(enabled=False)
    decision = "PICK_LABEL_DEFECT"
    sender.send_pick(decision, track_id=12, bin_name=decision_to_bin(decision))
    sender.close()
