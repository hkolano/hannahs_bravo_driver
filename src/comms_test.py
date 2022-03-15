#!/usr/bin/env python3
import rospy
import time
import socket
from bplprotocol import BPLProtocol, PacketID, PacketReader

MANIPULATOR_IP_ADDRESS = "192.168.2.3"
MANIPULATOR_PORT = 6789

DEVICE_IDS = [1, 2, 3, 4, 5, 6, 7]

REQUEST_FREQUENCY = 100

if __name__ == '__main__':

    manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0)

    pr = PacketReader()

    # Device_id 0XFF is broadcast that is received by all devices.
    request_packet = BPLProtocol.encode_packet(0x01, PacketID.REQUEST, bytes([PacketID.POSITION]))

    positions = ["-"] * len(DEVICE_IDS)

    next_req_time = time.time() + (1/REQUEST_FREQUENCY)

    while True:

        try:
            recv_bytes, address = sock.recvfrom(4096)
        except BaseException:
            recv_bytes = b''
        if recv_bytes:
            packets = pr.receive_bytes(recv_bytes)
            print("elapsed time: {}".format(time.time()-sent_time))
            for device_id, packet_id, data in packets:
                print("device: {}".format(device_id))
                if packet_id == PacketID.POSITION:

                    if device_id in DEVICE_IDS:
                        position = BPLProtocol.decode_floats(data)[0]

                        idx = DEVICE_IDS.index(device_id)

                        positions[idx] = f"{position:.2f}"

                        print_string = f"{time.perf_counter():.3f}| Positions: "
                        for dev_id, position in zip(DEVICE_IDS, positions):
                            print_string += f"{dev_id}: {position}, "
                        print(print_string)

        if time.time() >= next_req_time:
            next_req_time += (1/REQUEST_FREQUENCY)
            sock.sendto(request_packet, manipulator_address)
            sent_time = time.time()

