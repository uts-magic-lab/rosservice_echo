# rosservice_echo
Just like `rostopic echo /your_topic` but with services.

Well, similarly.

You need to run it as **root** from a machine that is either doing the service calls or exposing the server. That's because it
internally uses [scapy](http://www.secdev.org/projects/scapy/) to sniff network packets.

To be able to see the deserialized messages you need the messages installed. You'll get a nice blob of random characters otherwise (plaintext is readable!).

Possible future **TODO**: `rosservicebag record/play` maybe. With the work in [message_thief](https://github.com/uts-magic-lab/message_thief), [rosduct](https://github.com/uts-magic-lab/rosduct) and maybe [rosimport](https://github.com/pyros-dev/rosimport) it should be *not that hard*.


# Usage

```bash
rosrun rosservice_echo rosservice_echo.py /service_name [-v]
```

The service name must be existing at the moment. `-v` is for (VERY) verbose output.

# Examples

Shell 1:

```bash
roscore
```

Shell 2:

```bash
# Optional, just showing how to get it to run
mkdir -p rosservice_echo_ws/src
cd rosservice_echo_ws/src
git clone https://github.com/uts-magic-lab/rosservice_echo
cd ..
rosdep install --from-paths src --ignore-src
# Or sudo apt-get install python-scapy
catkin_make
source devel/setup.bash

rosrun rosservice_echo rosservice_echo.py /rosout/get_loggers
```

Shell 3:

```bash
rosservice call /rosout/get_loggers
```

The output from `rosservice_echo` will be:

```
[roscpp/GetLoggersRequest]:

[roscpp/GetLoggersResponse]:
loggers: 
  - 
    name: ros
    level: info
  - 
    name: ros.roscpp
    level: info
  - 
    name: ros.roscpp.roscpp_internal
    level: info
  - 
    name: ros.roscpp.superdebug
    level: warn
---
```

If you run `rosservice_echo.py` with `-v`:

```
Service /rosout/get_loggers uses RPC url: rosrpc://xpssam:54182
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=60 id=39775 flags=DF frag=0L ttl=64 proto=tcp chksum=0xa05a src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43098 dport=54182 seq=664942391 ack=0 dataofs=10L reserved=0L flags=S window=43690 chksum=0xff30 urgptr=0 options=[('MSS', 65495), ('SAckOK', ''), ('Timestamp', (8170600, 0)), ('NOP', None), ('WScale', 7)] |>>>

-> INCOMING service message from port 43098 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: SYN 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=60 id=0 flags=DF frag=0L ttl=64 proto=tcp chksum=0x3bba src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43098 seq=2832308807 ack=664942392 dataofs=10L reserved=0L flags=SA window=43690 chksum=0xff30 urgptr=0 options=[('MSS', 65495), ('SAckOK', ''), ('Timestamp', (8170600, 8170600)), ('NOP', None), ('WScale', 7)] |>>>

-> OUTCOMING service message from port 54182 to port 43098 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: SYN ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=39776 flags=DF frag=0L ttl=64 proto=tcp chksum=0xa061 src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43098 dport=54182 seq=664942392 ack=2832308808 dataofs=8L reserved=0L flags=A window=342 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170600, 8170600))] |>>>

-> INCOMING service message from port 43098 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=134 id=39777 flags=DF frag=0L ttl=64 proto=tcp chksum=0xa00e src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43098 dport=54182 seq=664942392 ack=2832308808 dataofs=8L reserved=0L flags=PA window=342 chksum=0xff7a urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170600, 8170600))] |<Raw  load='N\x00\x00\x00\x14\x00\x00\x00callerid=/rosservice\x08\x00\x00\x00md5sum=*\x07\x00\x00\x00probe=1\x1b\x00\x00\x00service=/rosout/get_loggers' |>>>>

-> INCOMING service message from port 43098 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: PSH ACK 
  Service info REQUEST:
  callerid = /rosservice
  md5sum = *
  probe = 1
  service = /rosout/get_loggers
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=13856 flags=DF frag=0L ttl=64 proto=tcp chksum=0x5a2 src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43098 seq=2832308808 ack=664942474 dataofs=8L reserved=0L flags=A window=342 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170600, 8170600))] |>>>

-> OUTCOMING service message from port 54182 to port 43098 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=229 id=13857 flags=DF frag=0L ttl=64 proto=tcp chksum=0x4f0 src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43098 seq=2832308808 ack=664942474 dataofs=8L reserved=0L flags=PA window=342 chksum=0xffd9 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170600, 8170600))] |<Raw  load="\xad\x00\x00\x00\x10\x00\x00\x00callerid=/rosout'\x00\x00\x00md5sum=32e97e85527d4678a8f9279894bb64b0%\x00\x00\x00request_type=roscpp/GetLoggersRequest'\x00\x00\x00response_type=roscpp/GetLoggersResponse\x16\x00\x00\x00type=roscpp/GetLoggers" |>>>>

-> OUTCOMING service message from port 54182 to port 43098 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: PSH ACK 
  Service info RESPONSE:
  callerid = /rosout
  md5sum = 32e97e85527d4678a8f9279894bb64b0
  request_type = roscpp/GetLoggersRequest
  response_type = roscpp/GetLoggersResponse
  type = roscpp/GetLoggers
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=39778 flags=DF frag=0L ttl=64 proto=tcp chksum=0xa05f src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43098 dport=54182 seq=664942474 ack=2832308985 dataofs=8L reserved=0L flags=A window=350 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170600, 8170600))] |>>>

-> INCOMING service message from port 43098 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=39779 flags=DF frag=0L ttl=64 proto=tcp chksum=0xa05e src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43098 dport=54182 seq=664942474 ack=2832308985 dataofs=8L reserved=0L flags=FA window=350 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170601, 8170600))] |>>>

-> INCOMING service message from port 43098 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: FIN ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=13858 flags=DF frag=0L ttl=64 proto=tcp chksum=0x5a0 src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43098 seq=2832308985 ack=664942475 dataofs=8L reserved=0L flags=FA window=342 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170601, 8170601))] |>>>

-> OUTCOMING service message from port 54182 to port 43098 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: FIN ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=39780 flags=DF frag=0L ttl=64 proto=tcp chksum=0xa05d src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43098 dport=54182 seq=664942475 ack=2832308986 dataofs=8L reserved=0L flags=A window=350 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170601, 8170601))] |>>>

-> INCOMING service message from port 43098 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=60 id=29400 flags=DF frag=0L ttl=64 proto=tcp chksum=0xc8e1 src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43112 dport=54182 seq=685301359 ack=0 dataofs=10L reserved=0L flags=S window=43690 chksum=0xff30 urgptr=0 options=[('MSS', 65495), ('SAckOK', ''), ('Timestamp', (8170679, 0)), ('NOP', None), ('WScale', 7)] |>>>

-> INCOMING service message from port 43112 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: SYN 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=60 id=0 flags=DF frag=0L ttl=64 proto=tcp chksum=0x3bba src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43112 seq=2023061531 ack=685301360 dataofs=10L reserved=0L flags=SA window=43690 chksum=0xff30 urgptr=0 options=[('MSS', 65495), ('SAckOK', ''), ('Timestamp', (8170679, 8170679)), ('NOP', None), ('WScale', 7)] |>>>

-> OUTCOMING service message from port 54182 to port 43112 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: SYN ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=29401 flags=DF frag=0L ttl=64 proto=tcp chksum=0xc8e8 src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43112 dport=54182 seq=685301360 ack=2023061532 dataofs=8L reserved=0L flags=A window=342 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |>>>

-> INCOMING service message from port 43112 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=174 id=29402 flags=DF frag=0L ttl=64 proto=tcp chksum=0xc86d src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43112 dport=54182 seq=685301360 ack=2023061532 dataofs=8L reserved=0L flags=PA window=342 chksum=0xffa2 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |<Raw  load="v\x00\x00\x00(\x00\x00\x00callerid=/rosservice_30277_1511811765843'\x00\x00\x00md5sum=32e97e85527d4678a8f9279894bb64b0\x1b\x00\x00\x00service=/rosout/get_loggers" |>>>>

-> INCOMING service message from port 43112 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: PSH ACK 
  Service info REQUEST:
  callerid = /rosservice_30277_1511811765843
  md5sum = 32e97e85527d4678a8f9279894bb64b0
  service = /rosout/get_loggers
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=54443 flags=DF frag=0L ttl=64 proto=tcp chksum=0x6716 src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43112 seq=2023061532 ack=685301482 dataofs=8L reserved=0L flags=A window=342 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |>>>

-> OUTCOMING service message from port 54182 to port 43112 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=229 id=54444 flags=DF frag=0L ttl=64 proto=tcp chksum=0x6664 src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43112 seq=2023061532 ack=685301482 dataofs=8L reserved=0L flags=PA window=342 chksum=0xffd9 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |<Raw  load="\xad\x00\x00\x00\x10\x00\x00\x00callerid=/rosout'\x00\x00\x00md5sum=32e97e85527d4678a8f9279894bb64b0%\x00\x00\x00request_type=roscpp/GetLoggersRequest'\x00\x00\x00response_type=roscpp/GetLoggersResponse\x16\x00\x00\x00type=roscpp/GetLoggers" |>>>>

-> OUTCOMING service message from port 54182 to port 43112 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: PSH ACK 
  Service info RESPONSE:
  callerid = /rosout
  md5sum = 32e97e85527d4678a8f9279894bb64b0
  request_type = roscpp/GetLoggersRequest
  response_type = roscpp/GetLoggersResponse
  type = roscpp/GetLoggers
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=29403 flags=DF frag=0L ttl=64 proto=tcp chksum=0xc8e6 src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43112 dport=54182 seq=685301482 ack=2023061709 dataofs=8L reserved=0L flags=A window=350 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |>>>

-> INCOMING service message from port 43112 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=56 id=29404 flags=DF frag=0L ttl=64 proto=tcp chksum=0xc8e1 src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43112 dport=54182 seq=685301482 ack=2023061709 dataofs=8L reserved=0L flags=PA window=350 chksum=0xff2c urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |<Raw  load='\x00\x00\x00\x00' |>>>>

-> INCOMING service message from port 43112 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: PSH ACK 
[roscpp/GetLoggersRequest]:

===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=169 id=54445 flags=DF frag=0L ttl=64 proto=tcp chksum=0x669f src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43112 seq=2023061709 ack=685301486 dataofs=8L reserved=0L flags=PA window=342 chksum=0xff9d urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |<Raw  load='\x01p\x00\x00\x00\x04\x00\x00\x00\x03\x00\x00\x00ros\x04\x00\x00\x00info\n\x00\x00\x00ros.roscpp\x04\x00\x00\x00info\x1a\x00\x00\x00ros.roscpp.roscpp_internal\x04\x00\x00\x00info\x15\x00\x00\x00ros.roscpp.superdebug\x04\x00\x00\x00warn' |>>>>

-> OUTCOMING service message from port 54182 to port 43112 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: PSH ACK 
[roscpp/GetLoggersResponse]:
loggers: 
  - 
    name: ros
    level: info
  - 
    name: ros.roscpp
    level: info
  - 
    name: ros.roscpp.roscpp_internal
    level: info
  - 
    name: ros.roscpp.superdebug
    level: warn
---
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=54446 flags=DF frag=0L ttl=64 proto=tcp chksum=0x6713 src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43112 seq=2023061826 ack=685301486 dataofs=8L reserved=0L flags=FA window=342 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |>>>

-> OUTCOMING service message from port 54182 to port 43112 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: FIN ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=29405 flags=DF frag=0L ttl=64 proto=tcp chksum=0xc8e4 src=127.0.0.1 dst=127.0.1.1 options=[] |<TCP  sport=43112 dport=54182 seq=685301486 ack=2023061827 dataofs=8L reserved=0L flags=FA window=350 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |>>>

-> INCOMING service message from port 43112 to port 54182 from ip 127.0.0.1 to ip 127.0.1.1 TCP flags: FIN ACK 
  TCP protocol message.
===============================
Packet:
<Ether  dst=00:00:00:00:00:00 src=00:00:00:00:00:00 type=0x800 |<IP  version=4L ihl=5L tos=0x0 len=52 id=54447 flags=DF frag=0L ttl=64 proto=tcp chksum=0x6712 src=127.0.1.1 dst=127.0.0.1 options=[] |<TCP  sport=54182 dport=43112 seq=2023061827 ack=685301487 dataofs=8L reserved=0L flags=A window=342 chksum=0xff28 urgptr=0 options=[('NOP', None), ('NOP', None), ('Timestamp', (8170679, 8170679))] |>>>

-> OUTCOMING service message from port 54182 to port 43112 from ip 127.0.1.1 to ip 127.0.0.1 TCP flags: ACK 
  TCP protocol message.
```
