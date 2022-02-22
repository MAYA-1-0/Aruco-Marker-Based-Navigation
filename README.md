# Aruco-Marker-Based-Navigation

###  Visual Navigation

Use of fiducial markers in visual navigation is a prominent area of research these days, Aruco markers have proved to be very promising among other fiducial markers like April Tag [9], QR tag. Aruco markers are 2D binary patterns which can be associated with ID’s and each unique ID can be used to associate with different activities or tasks in case of MAYA. 

An ArUco marker has a wide black border and an inner binary matrix which determines its  unique ID. The black border facilitates its fast detection in the image.The marker size determines the size of the internal matrix. For instance a marker size of 4x4 is composed of 16 bits.


#####     Aruco Markers
![image](https://github.com/MAYA-1-0/Aruco-Marker-Based-Navigation/blob/main/images/Screenshot%20from%202022-02-22%2022-21-32.png)

ArUco Markers are used for the Navigation, Multiple markers can be placed at different locations in a building and can be used to localise the robot and also find the parameters needed to navigate towards any markers in case of automatic docking system for charging. We’ve employed Realsense d455 for visual and depth perception

#### 1 . Robot’s camera plane and Marker’s plane are always parallel- Ideal Case

![img]()

