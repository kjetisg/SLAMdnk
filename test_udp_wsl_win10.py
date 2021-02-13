
import socket

UDP_IP_ADDRESS = "172.19.240.1"
UDP_PORT = 8080
message = "Hello from WSL!"


def run_udp_client(message):
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.sendto(message.encode('utf-8'), (UDP_IP_ADDRESS, UDP_PORT))


def run_udp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((UDP_IP_ADDRESS, UDP_PORT))

    while True:
        data, addr = server.recvfrom(1024)
        print("Received: ", data)

def server(local_ip, local_port):
    server = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    server.bind((local_ip, local_port))
    print("Server Up And Listening")
    print("-----------------------")
    #sleep(999)
    while True:
        bytes_adr_pair = server.recvfrom(1024)
        msg = bytes_adr_pair[0]
        address = bytes_adr_pair[1]
        print("{} Sent: {}".format(address, msg))

if __name__ == "__main__":
    run_udp_client(message)
    # run_udp_server()
    # server("0.0.0.0", 6006)
    # server("172.19.243.9", 10901)
    # server("172.19.243.9", 6006)