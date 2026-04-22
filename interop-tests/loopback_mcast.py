"""Loopback-aware UDP multicast CAN bus for testing.

python-can's UdpMulticastBus doesn't set IP_MULTICAST_IF, so multicast
goes out the default interface (usually WiFi/ethernet) and doesn't loop
back to other processes on localhost. This wrapper patches the socket to
use the loopback interface.
"""

import socket
import struct

from can.interfaces.udp_multicast.bus import UdpMulticastBus

MCAST_GROUP = "239.74.163.2"


class LoopbackMulticastBus(UdpMulticastBus):
    """UdpMulticastBus that forces multicast through loopback."""

    def __init__(self, channel=MCAST_GROUP, **kwargs):
        super().__init__(channel=channel, **kwargs)
        sock = self._multicast._socket
        # Send multicast on loopback
        sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_MULTICAST_IF,
            socket.inet_aton("127.0.0.1"),
        )
        # Ensure loopback delivery
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        # Re-join multicast group on loopback interface
        mreq = struct.pack(
            "4s4s",
            socket.inet_aton(channel),
            socket.inet_aton("127.0.0.1"),
        )
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
