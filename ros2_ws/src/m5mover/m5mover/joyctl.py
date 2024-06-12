import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from m5mover_msg.msg import MoverCommand
from m5mover_msg.msg import MoverInfo

class JoyTranslate(Node): 
    spdr = 0
    spdl = 0
    led = 0
    lift = 0

    def __init__(self):
        super().__init__('joyctl') 
        self.publisher = self.create_publisher(MoverCommand, '/cmd_msg', 10)
        self.subscription_mover = self.create_subscription(MoverInfo, '/info_msg',  self.info_callback, 10)
        self.subscription_joy = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd = MoverCommand()

    def joy_callback(self, Joy):
        self.spdr = int(Joy.axes[2] * 100);
        self.spdl = int(Joy.axes[1] * 100);
        if Joy.buttons[0] == 1:
            self.led = 1
        elif Joy.buttons[1] == 1:
            self.led = 2
        elif Joy.buttons[2] == 1:
            self.led = 3
        elif Joy.buttons[3] == 1:
            self.led = 4
        else:
            self.led = 0

        if Joy.axes[5] == 1:
            self.lift = 200
        elif Joy.axes[5] == -1:
            self.lift = 100
        else:
            self.lift = 0

        self.cmd.spdr = self.spdr        
        self.cmd.spdl = self.spdl        
        self.cmd.led = self.led        
        self.cmd.lift = self.lift        
#        print(self.cmd)

        self.publisher.publish(self.cmd)

    def info_callback(self, msg):
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    joy_translate = JoyTranslate()
    rclpy.spin(joy_translate)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

