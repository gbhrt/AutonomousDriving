### About
A minimal implementation for testing basic autonomous driving functionality of a Kia Niro.
It includes drivers for the GNSS+IMU system (Applanix) and the drive-by-wire system (Idan).
Additionally, it can be tested using a simple standalone Python simulator.

###Run

Simulation mode - launch simple simulator:
```bash
ros2 launch kia_test sim_load.launch.py auto_sync_mode:=True
```
With the KIA - launch GNSS node (Applanix) and drive-by-wire (IDAN)
```bash
ros2 launch kia_test kia_load.launch.py
```

Record a trajectory:
```bash
ros2 launch kia_test record_trajectory.launch.py
```

Drive the vehicle along the trajectory
```bash
ros2 launch kia_test drive_vehicle.launch.py
```
