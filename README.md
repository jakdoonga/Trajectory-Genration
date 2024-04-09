# Trajectory-Genration

## 1. Trajectory constraint setup
Navigate to traj_gen/config/traj_setup.yaml.

Set maximum velocity and acceleration of PAN and LIFT motors(Output values).

## 2. Initial value setup
Go to the traj_gen/launch/traj_gen.launch.

The index from 0 to 2 - PAN motor

The index from 3 to 5 - LIFT motor

The parameter value from **init_pos3** to **init_pos4** represents the  position 
(ouput) of motor which locates at the initial state.

Add or subtract the incremental values through using the parameter from **init_inc0** to **init_inc5**.

## 3. Mode value

### mode_val: 1
Run steering motor before lift and wheel motor moves.

### mode_val: 2
Run lift and wheel motors simultaneously.

### mode_val: 3
Run steering motor before pan and wheel motor moves.

### mode_val: 4
Run pan and wheel motors at the same time.

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

Run lift and wheel synchronously.

```
rostopic pub /mode_val traj_gen/mode "mode_val: 2 
goal_pos:
- 18.0
- 22.0
- 20.0"
```