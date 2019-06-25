import messages_pb2
import socket
UDP_IP = "192.168.0.1"
UDP_PORT = 7000

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))


def proto_reader():
    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes        
        ProtoData = messages_pb2.DataMessage()
        ProtoData.ParseFromString(data)
        print(ProtoData)

               
        
if __name__ == '__main__':
    try:
        proto_reader()
    except KeyboardInterrupt:
        pass