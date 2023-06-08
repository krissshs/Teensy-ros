# Teensy-ros

## Program publishes Teensy serial messages over <i>teensy_topic</i> and writes messages heard on <i>mtr_cmd_topic</i> to Teensy serial.

<b>Install necessary dependencies:</b>
```
rosdep install -i --from-path src --rosdistro foxy -y
```

<b>Build the package</b>
```
colcon build --packages-select teensy_com
```

<b>Publish Teensy serial messages</b>
```
source install/setup.bash && ros2 run teensy_com transmitter
```

<b>Simulate node that sends motor commands</b>
```
source install/setup.bash && ros2 run teensy_com mtr_cmd_sim
```

## Change baudrate and frequency
To change baudrate and message publishing interval (in seconds), navigate to ```src/teensy_com/publisher.py``` and chage ```BAUDRATE``` and ```PUBLISH_TIMEOUT```.

