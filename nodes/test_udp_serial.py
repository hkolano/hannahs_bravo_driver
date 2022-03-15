#!/usr/bin/env python3

"""ROS/bpl_passthrough/scripts/request_joint_positions.py"""

from urllib import request
import rospy
from bpl_msgs.msg import Packet
from std_msgs.msg import Float32

from bplprotocol import BPLProtocol, PacketID
import time
from collections import deque  

"""
Measure the latency to contact Bravo arm.

Requests the position of joint 1 (jaw), then publishes and prints the time that it takes to get a response.

Upon receipt of a response, it waits a bit then sends another message.

Filters the time with a moving average filter.

To use this script, first `roslaunch bpl_passthrough serial_passthrough.launch` (or `udp_passthrough.launch`).
Then `rosrun hannahs_bravo_driver test_udp_serial.py`.

You can plot out the time that it takes using, e.g., rqt_plot.
"""

# class MovingAvg:
#     def __init__(self):
#         self.q = deque(maxlen=20)

#     def update(self, x):
#         self.q.append(x)
#         return sum(self.q)/len(self.q)

sent_time = None
# filt = MovingAvg()

def receive_packet(packet: Packet):

    print("Received a packet")

    global sent_time, filt, tx_publisher, latency_publisher

    device_id = packet.device_id
    packet_id = packet.packet_id
    data = bytearray(packet.data)

    if packet_id == PacketID.POSITION and device_id == 1: # rotating base position

        rec_time = time.time_ns()
        # time_took = filt.update((rec_time - sent_time)/1e6)
        time_took = (rec_time - sent_time)/1e6
        print(f"Time took: {time_took} ms")
        latency_publisher.publish(Float32(time_took))

        position = BPLProtocol.decode_floats(data)[0]
        print("Position Received: {} - {}".format(device_id, position))
        

        time.sleep(0.15) # wait a bit for all the position messages to get through

        print("------- REQUESTING A PACKET ------")
        tx_publisher.publish(request_packet)
        sent_time = time.time_ns()


if __name__ == '__main__':
    tx_publisher = rospy.Publisher("tx", Packet, queue_size=100)
    latency_publisher = rospy.Publisher("latency", Float32, queue_size=100)

    rospy.init_node("request_joint_position_script")
    
    rx_subscriber = rospy.Subscriber("rx", Packet, receive_packet)

    request_packet = Packet(0x01, PacketID.REQUEST, [PacketID.POSITION])

    print("------- REQUESTING A PACKET ------")
    tx_publisher.publish(request_packet)
    sent_time = time.time_ns()

    rospy.spin()
