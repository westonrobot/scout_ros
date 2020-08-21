```
$ rostopic pub -1 /scout_motor_fl_controller/command std_msgs/Float64 "data: 0.5"
```

Convert xacro to urdf

```
$ rosrun xacro xacro -o model.urdf model.urdf.xacro
```

Generate proto for Webots

```
$ rosrun xacro xacro -o scout_description/urdf/scout_v2.urdf scout_description/urdf/scout_v2_webots.xacro
$ python -m urdf2webots.importer --input=./scout_description/urdf/scout_v2.urdf --output=scout_description/proto/ScoutV2.proto
```

Convert urdf to sdf

```
$ gz sdf -p scout_v2.urdf > scout_v2.sdf
```

Reference:

[1] https://github.com/cyberbotics/urdf2webots