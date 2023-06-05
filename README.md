# Teensy-ros

## Program publishes Teensy serial messages over <i>teensy_topic</i>.

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

<b>Recieve Teensy serial messages</b>
```
source install/setup.bash && ros2 run teensy_com reciever
```

## Change baudrate and frequency
To change baudrate and message recieving interval (in seconds), navigate to ```src/teensy_com/publisher.py``` and chage ```BAUDRATE``` and ```TIMEOUT```.

