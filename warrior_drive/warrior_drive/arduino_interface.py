from std_msgs.msg import String
import serial

"""
A Class that acts as a interface for the Arduino and Ros2

"""

class ArduinoInterface():
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate)

    def read_line(self):
        if self.ser.in_waiting > 0: # Is there something to read?
            line = self.ser.readline().decode('utf-8').rstrip()
            return line
        return None
    
    def write_line(self, line):
        self.ser.write((line + '\n').encode('utf-8'))

if __name__ == '__main__':
    arduino = ArduinoInterface()
    while True:
        if arduino.read_line() is not None:
            print(arduino.read_line())