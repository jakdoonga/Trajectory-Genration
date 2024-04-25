# Trajectory-Genration

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
Run lift and wheel motors simultaneously.

### mode_val: 3
Steer before pan and wheel motor moves.

### mode_val: 4
Run pan and wheel motors at the same time.

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

Move lift and wheel synchronously.

```
rostopic pub /mode_val traj_gen/mode "mode_val: 2 
goal_pos:
- 18.0
- 22.0
- 20.0"
```