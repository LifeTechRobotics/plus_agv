import rclpy
from rclpy.node import Node
from m5mover_msg.msg import MoverCommand
from m5mover_msg.msg import MoverInfo
import socket
import re

target_ip = "192.168.0.246"
target_port = 10200
buffer_size = 4096


class Responder(Node):
    socket_flag = False
    connect_flag = False

    def __init__(self):
        super().__init__('m5mover')
        self.sub = self.create_subscription(MoverCommand, '/cmd_msg', self.msg_callback, 10)
        self.pub = self.create_publisher(MoverInfo, '/info_msg', 10)
        self.cmd = MoverCommand()
        self.info = MoverInfo()
        self.timer = self.create_timer(0.1, self.timer_callback)
       
    def timer_callback(self):
        print(self.cmd)
        # socket
        self.create_socket()
        # connect server
        self.connect_server()
        
        # send to M5Stack
        self.send_data(self.cmd)

        # dta recieve
        self.reciv_data()
        
        # get info data
#        print(self.info)
        self.pub.publish(self.info)

    def msg_callback(self, msg):
        self.cmd = msg

    # create socket
    def create_socket(self):
        # TCP Create Socket
        try:
            if self.socket_flag == False:
                self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#                print("socket");
                self.socket_flag = True
        except:
            self.socket_flag = False

    # connect_server
    def connect_server(self):
        # TCP connect server
        if self.socket_flag == False:
            return

        try:
            if self.connect_flag == False:
                self.tcp_client.connect((target_ip, target_port))
#                print("connect");
                self.connect_flag = True
        except:
            self.socket.close()
            self.socket_flag = False
            self.connect_flag = False

    # send data
    def send_data(self, cmd):
        # LED R
        if cmd.led & 0b0001:
            led = "255,"
        else:
            led = "000,"
        # LED G
        if cmd.led & 0b0010:
            led = led + "255,"
        else:
            led = led + "000,"
        # LED B
        if cmd.led & 0b0100:
            led = led + "255,"
        else:
            led = led + "000,"

        # Brightness
        led = led + "10"

        data = "###," + str(cmd.spdr) + ","  + str(cmd.spdl) + ","  + led + "," + str(cmd.lift) + '\r'
#        print(data)
        
        if self.connect_flag == False:
            return

        try:
            self.tcp_client.send(data.encode("ascii"))
#            print("send:", data)
            
        except:
            self.tcp_client.close()
            self.socket_flag = False
            self.connect_flag = False

    # data recive
    def reciv_data(self):
        try:
            data = self.tcp_client.recv(65)
            stg = data.decode()
#            print(stg)
            param = re.split(',', stg)
            self.info.battery = int(param[1])
            self.info.spdr = int(param[2])
            self.info.spdl = int(param[3])
            self.info.encr = int(param[4])
            self.info.encl = int(param[5])
            self.info.status = int(param[6])
            self.info.weight = int(param[7])
            self.info.lift = int(param[8])
            print(param)
            
        except:
            self.tcp_client.close()
            self.socket_flag = False
            self.connect_flag = False

        return self.info

def main():
    rclpy.init()
    node = Responder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
   
