# Trajectory-Genrator

## 1. Trajectory constraint setup
Navigate to traj_gen/config/traj_setup.yaml.

Set maximum velocity and acceleration of PAN and LIFT motors(Output values).

## 2. Offset value setup for lift motor
Go to the traj_gen/launch/traj_gen.launch.

The parameter value from **offset_pos0** to **offset_pos2** represents 
the offset position value of the lift motors.

## 3. Mode value

### mode_val: 1
Steer before lift and wheel motor moves.

### mode_val: 2
Generate and publish the trajectory that lift and wheel motors move simultaneously.

### mode_val: 3
Steer before pan and wheel motor moves.

### mode_val: 4
Create and publish the trajectory which run pan and wheel motors at the same time.

### mode_val: 5
Steer before moving forward.

### mode_val: 6
Get /cmd_vel and move forward or backword.

## 4. Topic

### Subscribed topic

/mode_val

/actual

/cmd_vel

### Published topic

/target

/target_dxl


## 5. How to launch

Execute the node by using roslaunch.

```
roslaunch traj_gen traj_gen.launch 
```

Create the trajectory of lift and wheel.
Herein, the goal position is not the relative value but the absolute one.

```
rostopic pub /mode_val traj_gen/mode "mode_val: 2 
goal_pos:
- 18.0
- 22.0
- 20.0"
```

## 6. Lift motor trajectory test result

Bug fixed.

Target increment value of lift motor starts from 0 when the offset value of lift motor is set. 

<image src="figures/lift_motor_test.png">