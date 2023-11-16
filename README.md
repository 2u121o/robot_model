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


![map_screen](https://user-images.githubusercontent.com/32509386/184494559-a312c9d1-7c3f-4152-b2b4-d0c1db807212.png)

where the thick edges represent the walls, that the robot can not cross. The robot body is represented by the circle and the orientation wrt the x-axis by the black line inside the robot. Instead the light blu line represent the range measurement with the closest point.

This robot model can be easily implemented as an external library, see the main.cpp as an example.
