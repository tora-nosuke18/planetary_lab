

#ROS2 publish(float64multiarray): /C620/actual_rad "speed1, speed2, speed3, speed4"
#ROS2 subscribe1(float64multiarray): /rover/left_targets "speed1, speed2"
#ROS2 subscribe2(float64multiarray): /rover/right_targets "speed3, speed4" 

#Serial send: motor wheel rad "speed1, speed2, speed3, speed4"
#Serial receive: motor actual rad "speed1, speed2, speed3, speed4"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time
import threading

class Serial4Wheel(Node):
    def __init__(self):
        super().__init__('serial_4wheel')
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/C620/actual_rad', 10)
        self.wheel_sub = self.create_subscription(Float64MultiArray, '/rover/targets', self.wheel_cb, 10)
        #self.timer = self.create_timer(0.1, self.send_serial)
        #self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.get_logger().info('Serial 4 wheel node has started')
        #self.actual_wheel_data = [0.0, 0.0, 0.0, 0.0]#Float64MultiArray()
        self.serial_wheel_data=[0,0,0,0]
        self.serial_send_data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.timer = self.create_timer(1.0, self.receive_serial)


    def wheel_cb(self, msg):
        self.serial_wheel_data[0] = msg.data[0]
        self.serial_wheel_data[1] = msg.data[1]
        self.serial_wheel_data[2] = msg.data[2]
        self.serial_wheel_data[3] = msg.data[3]
    #    self.get_logger().info('callback')
        self.send_serial()
        
        

        
    def send_serial(self):
    #    self.get_logger().info('send serial')
        self.serial_send_data[0] = 0xFF
        for i in range(4):
            data = int(self.serial_wheel_data[i] * 100)
            if(data > 0):
                sign = 1
            else:
                sign = 0
            value = abs(data)
            self.serial_send_data[i*2+1] = (i<<6) | (sign << 5) | ((value >> 8) & 0x1F)
            self.serial_send_data[i*2+2] = value & 0xFF
        self.ser.write(self.serial_send_data)
        time.sleep(0.1)
        self.receive_serial()

    def receive_serial(self):
        if self.ser.in_waiting >=9:
            self.get_logger().info('start reading')
            
            serial_read_data = self.ser.read(9)
            #self.get_logger().info("%d",serial_read_data[0])
            if serial_read_data[0] == 0xFF:
                self.get_logger().info('processing data')
                actual_wheel_data = Float64MultiArray()
                speeds = [0.0, 0.0, 0.0, 0.0]
                for i in range(4):
                    motor_num = int((serial_read_data[i*2+1] & 0xC0) >> 6)
                    if i == motor_num:
                        if serial_read_data[i*2+1] & 0x30:
                            sign = 1
                        else:
                            sign = -1
                        value = int(((serial_read_data[i*2+1] & 0x1F) << 8) | serial_read_data[i*2+2])
                        speeds[i] = sign * float(value) / 100
                    else:
                        """self.get_logger().info('motor num mismatch')"""
                actual_wheel_data.data = speeds
                
                self.wheel_pub.publish(actual_wheel_data)
                self.get_logger().info('receive serial')
            while self.ser.in_waiting > 0:
        #        self.get_logger().info('overreading')
                self.ser.read()
        #self.decode

    #パブリッシャのループ　TODO:ほかのシリアルインターフェースを用いる
    # def run(self):
    #     while True:
    #         try:
    #             data = self.ser.readline().decode().strip()
    #             data = data.split(',')
    #             if len(data) == 4:
    #                 self.actual_wheel_data.data = [float(data[0]), float(data[1]), float(data[2]), float(data[3])]
    #                 self.wheel_pub.publish(self.actual_wheel_data)
    #         except Exception as e:
    #             self.get_logger().error(str(e))
    #             break

    def stop(self):
        self.ser.close()
        self.get_logger().info('Serial 4 wheel node has stopped')

def main(args=None):
    rclpy.init(args=args)
    serial_4wheel = Serial4Wheel()
    #serial_4wheel.run()
    #t = threading.Thread(target = receive_serial)
    #t.start()
    rclpy.spin(serial_4wheel)
    #while True:
        #serial_4wheel.send_serial()
        #time.sleep(1)
        

    
    serial_4wheel.stop()

if __name__ == '__main__':
    main()