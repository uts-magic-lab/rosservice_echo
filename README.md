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

# Install

Meanwhile I don't release this (should I?) you can do:

```bash
mkdir -p rosservice_echo_ws/src
cd rosservice_echo_ws/src
git clone https://github.com/uts-magic-lab/rosservice_echo
cd ..
rosdep install --from-paths src --ignore-src
# Or sudo apt-get install python-scapy
catkin_make
source devel/setup.bash
```

# Examples

## rosout

Shell 1:

```bash
roscore
```

Shell 2:

```bash
# In the shell that you did
# source rosservice_echo_ws/devel/setup.bash
rosrun rosservice_echo rosservice_echo.py /rosout/get_loggers
```

Shell 3:

```bash
rosservice call /rosout/get_loggers "{}"
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

To find an example of the verbose output check [README_echo_verbose.md](README_echo_verbose.md).


## PR2 simulation

```bash
# sudo apt-get install ros-indigo-pr2-simulator
roslaunch pr2_gazebo pr2_empty_world.launch
```

Try `rosservice list` and see if you see anything interesting.

I tried and apparently there is stuff doing service calls non stop very very fast (and I was using a slow computer) and I could not deserialize the messages (I use a very lame non-state-based technique), but it did show the traffic!
