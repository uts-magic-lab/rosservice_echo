#! /usr/bin/env python
from scapy.all import *
import rospy
import rosgraph.masterapi
import struct
import sys
from pydoc import locate
import socket


"""
Like rostopic echo but for services.

Needs to be run as root as it uses low level networking stuff.

To see the messages you need to have them installed.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

TCP_FLAGS = {
    'F': 'FIN',
    'S': 'SYN',
    'R': 'RST',
    'P': 'PSH',
    'A': 'ACK',
    'U': 'URG',
    'E': 'ECE',
    'C': 'CWR',
}

global previous_srv_type
global previous_srv_name
global previous_pkt


def create_cb(service_name, port, verbose_mode):
    def service_monitor_callback(pkt):
        global previous_srv_type
        global previous_srv_name
        global previous_pkt
        # Workaround for local interfaces, we get the same message twice
        if pkt == previous_pkt:
            return
        else:
            previous_pkt = pkt
        payload = pkt.sprintf("%TCP.payload%")
        src = pkt.sport
        dst = pkt.dport
        flags = pkt.sprintf('%TCP.flags%')
        flags_str = ''
        for flag in flags:
            flags_str += TCP_FLAGS[flag] + ' '
        srcip = pkt.sprintf('%IP.src%')
        dstip = pkt.sprintf('%IP.dst%')
        if verbose_mode:
            print("===============================")
            print("Packet:\n%s\n" % (pkt.__repr__()))
        if src == port:
            direction = 'OUTCOMING'
        elif dst == port:
            direction = 'INCOMING'
        if verbose_mode:
            print("-> %s service message from port %d to port %d from ip %s to ip %s TCP flags: %s" %
                  (direction, src, dst, srcip, dstip, flags_str))
        else:
            if verbose_mode:
                print(
                    "Error: Received a packet that neither the source or destination port is the one requested.")
                return

        if len(payload) >= 4:
            if direction == 'OUTCOMING':
                # Hack to check in between Responses and TCPROS protocol
                # If this is reimplemented in a stateful way so we can follow
                # the TCP flow this is not needed
                if 'callerid=' not in payload:
                    ok = struct.unpack('<B', payload[0])[0]
                    if not ok:
                        error_len = struct.unpack("<I", payload[1:5])[0]
                        error_str = payload[5:]
                        print("Error: service [%s] responded with an error (of length %d): %s" %
                              (previous_srv_name, error_len, error_str))
                        return
                    else:
                        # Rip off OK byte
                        payload = payload[1:]

            is_info_request_or_response = False
            if 'callerid=' in payload\
                    and 'md5sum=' in payload\
                    and 'service=' in payload:
                    # and 'probe=' in payload\  # on initial request only, and md5sum=* in that case
                is_info_request_or_response = True
                if verbose_mode:
                    print("  Service info REQUEST:")
            elif 'callerid=' in payload\
                    and 'md5sum=' in payload\
                    and 'request_type=' in payload\
                    and 'response_type=' in payload\
                    and 'type=' in payload:
                is_info_request_or_response = True
                if verbose_mode:
                    print("  Service info RESPONSE:")

            if is_info_request_or_response:
                curr_bytes = 4
                bytes_left_to_read = len(payload) - curr_bytes
                while bytes_left_to_read > 0:
                    field_length = struct.unpack("<I",
                                                 payload[curr_bytes:curr_bytes + 4])[0]
                    curr_bytes += 4
                    if field_length > len(payload):
                        continue

                    field = payload[curr_bytes:curr_bytes + field_length]
                    curr_bytes += field_length
                    fieldname, fieldvalue = field.split('=')
                    if verbose_mode:
                        print("  %s = %s" % (fieldname, fieldvalue))
                    if fieldname == 'type':
                        previous_srv_type = fieldvalue
                    if fieldname == 'service':
                        previous_srv_name = fieldvalue

                    bytes_left_to_read = len(payload) - curr_bytes
            else:
                # Here the magic of the class that deserializes is needed
                # because it knows what is coming
                if previous_srv_type is not None and previous_srv_name == service_name:
                    pkg_name, srv_type = previous_srv_type.split('/')
                    srv_class = locate(pkg_name + ".srv." + srv_type)
                    if srv_class is None:
                        print("You don't have %s package with %s message installed, can't show messages other than raw: %s" % (
                            pkg_name, srv_type, payload))
                    else:
                        srv_req_class = locate(
                            pkg_name + ".srv." + srv_type + "Request")
                        srv_resp_class = locate(
                            pkg_name + ".srv." + srv_type + "Response")

                        if direction == 'OUTCOMING':
                            data_class = srv_resp_class()
                            msg_type_used = previous_srv_type + "Response"
                        elif direction == 'INCOMING':
                            data_class = srv_req_class()
                            msg_type_used = previous_srv_type + "Request"

                        # Removing the size of the payload itself
                        payload = payload[4:]
                        msg_instance = data_class.deserialize(payload)
                        print("[%s]:\n%s" %
                              (msg_type_used, msg_instance))
                    # Mimic rostopic echo
                    if direction == 'OUTCOMING':
                        print("---")

        else:
            if verbose_mode:
                print("  TCP protocol message.")
    return service_monitor_callback


if __name__ == '__main__':
    global previous_srv_type
    global previous_srv_name
    global previous_pkt
    previous_srv_type = None
    previous_srv_name = ''
    previous_pkt = None
    argv = rospy.myargv(sys.argv)
    if len(argv) != 2 and len(argv) != 3:
        print("Usage:")
        print(sys.argv[0] + " /service_to_echo [-v]")
        exit(0)
    service_to_sniff = argv[1]
    verbose_mode = False
    if len(argv) == 3 and argv[2] == '-v':
        verbose_mode = True

    master = rosgraph.masterapi.Master('/service_spy')
    sys_state = master.getSystemState()
    services = sys_state[2]
    found_service = False
    for service_name, node_name in services:
        rpc_url = master.lookupService(service_name)
        port = int(rpc_url.split(':')[-1])

        if service_name == service_to_sniff:
            found_service = True
            if verbose_mode:
                print("Service %s uses RPC url: %s" %
                      (service_to_sniff, rpc_url))
            try:
                sniffer = sniff(prn=create_cb(service_name, port, verbose_mode),
                                filter="tcp and ( port " + str(port) + " )",
                                store=0)
            except socket.error as e:
                print("Socket error: %s" % (e))
                print("Are you running this program as root?")

    if not found_service:
        print("Could not find service %s, are you sure it's running?" %
              (service_to_sniff))
