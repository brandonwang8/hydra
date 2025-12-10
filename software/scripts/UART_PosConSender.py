import serial
import time
import glob

def find_port():
    """Find the UART port — prefer /dev/serial0 if available."""
    if glob.glob("/dev/serial0"):
        return "/dev/serial0"
    elif glob.glob("/dev/ttyAMA0"):
        return "/dev/ttyAMA0"
    elif glob.glob("/dev/ttyS0"):
        return "/dev/ttyS0"
    else:
        return None

def main():

    time.sleep(3.0)

    port = find_port()
    if not port:
        print("Motor2040 not found.")
        return

    print(f"Connecting to {port}...")
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(2)  # allow CPY to reset

    print("UART connected. Type commands like: SET_POS 90   or   exit")



    while True:
        # Drain any pending feedback before prompting
        while True:
            line = ser.readline().decode().strip()
            if not line:
                break
            print("<<", line)

        cmd = input(">> ").strip()
        if cmd.lower() == "exit":
            break

        ser.write((cmd + "\n").encode())
        time.sleep(0.05)

if __name__ == "__main__":
    main()
