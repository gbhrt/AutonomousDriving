מצב סימולציה:
הפעלת סימולטור פשוט:
ros2 launch kia_test sim_load.launch.py auto_sync_mode:=True

עם ה KIA:
הפעלת חיישנים ועידן:
ros2 launch kia_test kia_load.launch.py


הקלטת מסלול (שם הקובץ נקבע ב launch file, כרגע נשמר ב kia_test/param/trajectories/trajectory1.txt בתוך install):
ros2 launch kia_test record_trajectory.launch.py

הסעת הרכב לאורך המסלול:
ros2 launch kia_test drive_vehicle.launch.py



