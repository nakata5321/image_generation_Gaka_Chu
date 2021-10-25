# ROS package for generating word from Google trends

## This package created for [gaka-chu online][db1].

## requirements
- ROS melodic. Install [here][db2]

## install
```shell
pip install -r requirements.txt
sudo apt-get install python3-empy
```
Build ros package with `catkin build` and python3
```shell
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## start node
```shell
roslaunch image_generation image_generation.launch
```





[db1]: <https://github.com/Multi-Agent-io/gaka-chu.online>
[db2]: <http://wiki.ros.org/melodic/Installation>