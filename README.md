# robot_model

Simple unicycle robot model, drawn on a 2D map that can be drawn bz hand in B/W color, exported as png and loaded in the model. The robot can be moved inside the map using the keyboard arrows. When the robot collide with an edge it just stop. To make the model more realistic a random noise has been implemented on the kinematic model as well as in the observation model. The observations consist in a range measurement with the nearest point to the robot, if this point is in the range of the sensor, and the bearing.


To use the robot model clone the repository and follow these steps:

```
cd robot_model/build
cmake ..
make 
./robot_model
```

this last command launches the 2D map and the robot as follow


![mobile_robot_model_lidar](https://github.com/2u121o/robot_model/assets/32509386/4c3b152b-68cb-4c37-9e1e-48e996ddc150)

where the thick edges represent the walls, that the robot can not cross. The robot body is represented by the circle and the orientation wrt the x-axis by the black line inside the robot. Instead the light blu line represent the range measurement with the closest point.


This robot model can be easily implemented as an external library, see the main.cpp as an example.
