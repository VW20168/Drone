import serial
import pynmea2
import io

# Configure serial port
ser = serial.Serial("/dev/ttyS0", 115200, timeout=1)  # Replace "/dev/ttyS0" with your serial port
sio = io.TextIOWrapper(ser, newline='\r\n')

try:
    while True:
        try:
            line = sio.readline()
            msg = pynmea2.parse(line)
            if isinstance(msg, pynmea2.GGA):
                print(f"Timestamp: {msg.timestamp}, Latitude: {msg.latitude}, Longitude: {msg.longitude}")
        except pynmea2.ParseError as e:
            print(f"Parse error: {e}")
        except UnicodeDecodeError as e:
            print(f"Decode error: {e}")

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
