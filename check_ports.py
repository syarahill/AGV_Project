import serial.tools.list_ports
from config import WHEEL_PORT, SENSOR_PORT

def check_available_ports():
    available_ports = [p.device for p in serial.tools.list_ports.comports()]
    
    print("Available COM ports:", available_ports)
    
    if WHEEL_PORT not in available_ports:
        print(f"WARNING: Motor controller port {WHEEL_PORT} not found!")
    if SENSOR_PORT not in available_ports:
        print(f"WARNING: Sensor port {SENSOR_PORT} not found!")

if __name__ == "__main__":
    check_available_ports()