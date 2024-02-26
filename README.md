# triorb-core

## Installation
```
python -m pip install -U git+https://github.com/TriOrb-Inc/triorb-core.git
```

## Quickstart
After connecting to the robot and starting up, the robot moves forward at a speed of 0.1 m/s for 5 seconds and then stops (sleeps).
```python
import time
import triorb_core
r = triorb_core.robot("/dev/ttyACM0")
r.wakeup()
r.set_vel_relative(0.0, 0.1, 0.0, acc=1000)
time.sleep(1.0)
r.brake() # Stops after moving forward for 1 second at a speed of 0.1 m/s.
r.sleep()
```

## API Reference

### triorb_core.robot(port=None, node=None)
Connects to the robot (control ECU).
#### Parameters:
- port - (optional) Set the URL of the USB serial device.
- node - (optional) Set ROS2 Node instance.
#### Returns: Robot object
#### Return type: triorb_core.robot
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
```

### triorb_core.robot.close_serial()
Close the connection to the robot (control ECU)
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.close_serial()
```

### triorb_core.robot.wakeup()
Excites all motors on the robot.
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
```

### triorb_core.robot.sleep()
Turns off the excitation of all motors on the robot.
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
#---<Control the robot>---
r.sleep()
```

### triorb_core.robot.set_vel_relative(vx, vy, vw, acc=None, dec=None)
Sets the movement speed based on the current robot posture. Note that the robot starts moving immediately after the setting.
#### Parameters:
- vx - Velocity in X-axis direction [m/s]
- vy - Velocity in Y-axis direction [m/s]
- vw - Rotation speed around Z-axis (Yaw rate) [rad/s]
- acc - (optional) Acceleration time [ms]
- dec - (optional) Deceleration time [ms]
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
r.set_vel_relative(0.0, 0.1, 0.0, acc=1000) # The robot moves forward at a speed of 0.1m/s while accelerating for 1000ms.
```

### triorb_core.robot.set_pos_relative(x, y, w, acc=None, dec=None, vel_xy=None, vel_w=None)
Set the amount of movement based on the current robot posture. Note that the movement starts immediately after the setting.
#### Parameters:
- x - Amount of movement in X-axis direction [m]
- y - Amount of movement in Y-axis direction [m]
- w - Amount of rotation around Z-axis [deg]
- acc - (optional) Acceleration time [ms]
- dec - (optional) Deceleration time [ms]
- vel_xy - (optional) Velocity in XY-axis direction [m/s]
- vel_w - (optional) Rotation speed around Z-axis (Yaw rate) [rad/s]
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
r.set_pos_absolute(-1.0, 0.5, 0.0, vel_xy=0.2) # Moves sideways -1m, forward 0.5m, at a speed of 0.2m/s.
```

### triorb_core.robot.set_vel_absolute(vx, vy, vw, acc=None, dec=None)
Set the movement speed with respect to the odometry origin. Note that the movement starts immediately after the setting.
#### Parameters:
- vx - Velocity in X-axis direction [m/s]
- vy - Velocity in Y-axis direction [m/s]
- vw - Rotation speed around Z-axis (Yaw rate) [rad/s]
- acc - (optional) Acceleration time [ms]
- dec - (optional) Deceleration time [ms]
#### Returns: 
#### Return type: list of response
#### Usage:
```python

```

### triorb_core.robot.set_pos_absolute(x, y, w, acc=None, dec=None, vel_xy=None, vel_w=None)
Set the amount of movement with respect to the odometry origin. Note that the movement starts immediately after the setting.
#### Parameters:
- x - Amount of movement in X-axis direction [m]
- y - Amount of movement in Y-axis direction [m]
- w - Amount of rotation around Z-axis [deg]
- acc - (optional) Acceleration time [ms]
- dec - (optional) Deceleration time [ms]
- vel_xy - (optional) Velocity in XY-axis direction [m/s]
- vel_w - (optional) Rotation speed around Z-axis (Yaw rate) [rad/s]
#### Returns: 
#### Return type: list of response
#### Usage:
```python

```

### triorb_core.robot.join()
Wait until the robot movement is completed. Note that this function only works after the set_pos function is executed.
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
r.set_pos_relative(0.0, 0.5, 0.0)
r.join()
print("Done.") # "Done." is displayed when the forward movement of 0.5m is completed.
```

### triorb_core.robot.brake()
Sets the robot's movement speed to 0 (≒braking is applied).
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import time
import triorb_core
r = triorb_core.robot()
r.wakeup()
r.set_vel_relative(0.0, 0.1, 0.0)
time.sleep(1.0)
r.brake() # Stops after moving forward for 1 second at a speed of 0.1 m/s.
```

### triorb_core.robot.get_pos()
Obtains the current robot posture with respect to the odometry origin.
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
print(r.get_pos())
```

### triorb_core.robot.get_motor_status(params=["error","state","voltage","power"], _id=[1,2,3])
Obtains the status of each motor mounted on the robot.
#### Parameters:
- params - (optional) Set the parameters to be acquired:
  - error: motor alarm
  - state: motor state
  - voltage: main power supply voltage
  - power: electric power
- _id - (optional) Set the motor IDs for which parameters are to be acquired.
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
print(r.get_motor_status(params=["power"], _id=[1])) # Obtains and displays the main power supply voltage.
```

### triorb_core.robot.reset_origin()
Sets the current robot posture as the odometry origin.
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
r.reset_origin()
```

### triorb_core.robot.set_odometry(x, y, w)
Modify the odometry origin so that the current robot posture is (x, y, w).
#### Parameters:
- x - X-axis coordinates [m].
- y - Y-axis coordinates [m].
- w - Angle around Z-axis [deg].
#### Returns: 
#### Return type: list of response
#### Usage:
```python
import triorb_core
r = triorb_core.robot()
r.wakeup()
r.set_odometry(0,0,90) # Sets the current posture as the odometry origin +90deg.
```