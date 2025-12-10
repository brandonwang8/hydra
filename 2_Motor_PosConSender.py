import serial
import time
import glob
from textwrap import dedent


def find_port():
    """Find UART port — prefer /dev/serial0 if available."""
    for candidate in ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"):
        if glob.glob(candidate):
            return candidate
    return None


def wait_for_ready(ser, timeout=5.0):
    """Wait for READY or probe for response."""
    start = time.time()
    while time.time() - start < timeout:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("<<", line)
            if "READY" in line.upper():
                return True
    # Fallback: probe with STATUS
    print("No READY detected, probing board...")
    ser.write(b"STATUS\n")
    time.sleep(0.3)
    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("<<", line)
            if "STATUS" in line.upper() or "READY" in line.upper():
                return True
    return False


def print_commands():
    print(
        dedent(
            """
        Commands:
          SET_POS 1 90     → set Motor 1 position to 90°
          SET_POS 2 -45    → set Motor 2 position to −45°
          STATUS           → query both motors
          RESET            → reinitialize board (soft restart)
          EXIT             → send motors to zero and quit
        """
        )
    )


def parse_status(line):
    """Format status line for readability."""
    parts = line.split()
    if len(parts) >= 16:
        return (
            f"Motor 1: POS={parts[2]}°, SP={parts[4]}°, THR={parts[6]} ({parts[7]})\n"
            f"Motor 2: POS={parts[10]}°, SP={parts[12]}°, THR={parts[14]} ({parts[15]})"
        )
    return line

def main():
    should_exit = False  # <-- NEW FLAG

    while not should_exit:  # <-- check flag instead of while True
        port = find_port()
        if not port:
            print("Motor2040 not found. Retrying in 3s...")
            time.sleep(3)
            continue

        print(f"\nConnecting to {port}...")
        try:
            ser = serial.Serial(port, 115200, timeout=0.2)
            time.sleep(0.5)
        except serial.SerialException as e:
            print(f"Error opening {port}: {e}")
            time.sleep(3)
            continue

        print("Waiting for board ready...\n")
        if not wait_for_ready(ser):
            print("Board did not respond. Retrying...")
            ser.close()
            time.sleep(2)
            continue

        print("Board ready.")
        print_commands()

        # AUTO-PING FOR STATUS
        print("Requesting initial status...\n")
        ser.write(b"STATUS\n")
        time.sleep(0.3)
        while ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                if line.startswith("STATUS"):
                    print(parse_status(line))
                else:
                    print("<<", line)

        try:
            while True:
                # Drain any feedback
                while ser.in_waiting:
                    line = ser.readline().decode(errors="ignore").strip()
                    if line:
                        if line.startswith("STATUS"):
                            print(parse_status(line))
                        else:
                            print("<<", line)

                cmd = input(">> ").strip()
                if not cmd:
                    continue

                if cmd.lower() == "exit":
                    print("Sending EXIT...")
                    ser.write(b"EXIT\n")
                    time.sleep(0.3)
                    while ser.in_waiting:
                        line = ser.readline().decode(errors="ignore").strip()
                        if line:
                            print("<<", line)
                    ser.close()
                    print("Closed connection. Sender exiting.")
                    should_exit = True  # <-- tell outer loop to exit
                    break

                elif cmd.lower() == "reset":
                    print("Sending RESET...")
                    ser.write(b"RESET\n")
                    time.sleep(0.3)
                    while ser.in_waiting:
                        line = ser.readline().decode(errors="ignore").strip()
                        if line:
                            print("<<", line)
                    print("Board reinitialized.\n")

                else:
                    ser.write((cmd + "\n").encode())
                    time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nUser interrupted. Closing.")
            ser.close()
            break


if __name__ == "__main__":
    main()
