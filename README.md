# Teensy-ros

## Program publishes Teensy serial messages over <i>teensy_topic</i> and writes messages heard on <i>mtr_cmd_topic</i> to Teensy serial. In addition to that, program reads GPS and compass data and publishes it over <i>gps_topic</i>.

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
source install/setup.bash && ros2 run teensy_com teensy
```

<b>Simulate node that sends motor commands and recieves GPS and compass data</b>
```
source install/setup.bash && ros2 run teensy_com simulator
```

<b>Publish GPS and compass data</b>
```
source install/setup.bash && ros2 run teensy_com gps_publisher
```

## Change baudrate and frequency
To change Teensy baudrate, navigate to ```src/teensy_com/publisher.py``` and change ```BAUDRATE_TEENSY```.

