# Aruco-Marker-Based-Navigation

###  Visual Navigation

Use of fiducial markers in visual navigation is a prominent area of research these days, Aruco markers have proved to be very promising among other fiducial markers like April Tag [9], QR tag. Aruco markers are 2D binary patterns which can be associated with ID’s and each unique ID can be used to associate with different activities or tasks in case of MAYA. 

An ArUco marker has a wide black border and an inner binary matrix which determines its  unique ID. The black border facilitates its fast detection in the image.The marker size determines the size of the internal matrix. For instance a marker size of 4x4 is composed of 16 bits.


#####     Aruco Markers
![image](https://github.com/MAYA-1-0/Aruco-Marker-Based-Navigation/blob/main/images/Screenshot%20from%202022-02-22%2022-21-32.png)

ArUco Markers are used for the Navigation, Multiple markers can be placed at different locations in a building and can be used to localise the robot and also find the parameters needed to navigate towards any markers in case of automatic docking system for charging. We’ve employed Realsense d455 for visual and depth perception

### Marker Search Algorithm
![img](https://github.com/MAYA-1-0/Aruco-Marker-Based-Navigation/blob/main/images/Screenshot%20from%202022-02-22%2023-20-04.png)

#### 1 . Robot’s camera plane and Marker’s plane are always parallel- Ideal Case

#####  Marker and Robot's Pose Representation
![img](https://github.com/MAYA-1-0/Aruco-Marker-Based-Navigation/blob/main/images/Screenshot%20from%202022-02-22%2022-50-32.png)


When the above two planes are parallel, which is only true in ideal cases, depth of the marker can be found from the realsense depth camera, further angle between robot’s x axis and the axis along the aligned centroid of frame and the marker. Let ‘a’ be the height. ‘b’ is the breadth of the detected marker, d is the depth of the marker’s centroid from the robot and d_c is the depth of the centroid of the frame. 
Let (x,y) be the point of the centroid and (x1,y1) be the centroid of the detected marker.
To find angle theta, find a new depth ‘d_n’ at point (x1,y) and use basic trigonometric equations to find the angle theta 
theta=acos(d/d_n)							
With this Robot can easily move towards a marker which is in the frame by travelling distance and theta calculated as above.

##### Depth Calculation for Marker
![img](https://github.com/MAYA-1-0/Aruco-Marker-Based-Navigation/blob/main/images/Screenshot%20from%202022-02-22%2022-53-43.png)


#### 2. Robot’s camera plane and  Marker’s plane are not parallel

When two planes are not parallel to each other calculations are much more complex when compared to the ideal case as there are parallax effects. As represented in the fig theta and depth required to reach the marker in the frame can be found.

#####  Marker and Robot's Pose Representation
![img](https://github.com/MAYA-1-0/Aruco-Marker-Based-Navigation/blob/main/images/Screenshot%20from%202022-02-22%2023-16-52.png)



