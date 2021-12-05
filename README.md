# Vison-based-lander
Subscribing to semsor_msgs/msg/image from a downward facing camera, this package will publish cmd_vel messages to move a drone to an Aruco target and effect landing through a action client.  A fancy way of describing vision based precision landing.

Through active control over the whole landing process this cod should be able to land a drone on its assigned marker in windy conditions.

The packages comprises two nodes:
## Tracker Node
The tracker node subscribes to a presumably downward facing camera and detects an Aruco marker.  Using OpenCV the marker is translated to a coordinate pose. These coordinates are published in the landing_target frame.  Further a base_camera->landing_target transform is published to maintain the coordinates of the marker relative to the drone. To make this work, a base_link->base_camera static transform must be published.
```
ros2 run tf2_ros static_transform_publisher 0.05 0.05 -0.03 0 0 -2.36 base_link base_camera
```
## Lander Node
Reads the last transform passed from base_camera to landing_target.  If the transform is recent enough, it is assumed that the traget is in sight. 
Sends out cmd_vel commands to the [Drone Node](https://github.com/slaghuis/drone_mavsdk.git) to bring about movement.  This is a closed loop control system.

The code is structures as a finite state machine.  On accepting a Action Server request to land at a given coordinate, the state machine is moved from PENDING to SEEKING.
### Seeking State
Flies toward the given coordinates at a parameterised approach alltitude, hoping to sight the landing target on arrival.  If the target is sighted, the state is changed to APPROACHING.  This type of flight will fly straight into a wall.  No collision avoidance is done.
### Approaching State
Reads the last transform passed from base_camera to landing_target the lander aims to minimise the positional error over the target, whilst maintaining the approach altitude.  If stability is achieved, the state is changed to DESCENDING.
### Descending State
Whilst maintaining position over the target, the altitude is reduced to a within a minimum of the landing target.  On achoving this minimum flight altitude, the state is changed to LANDING.
### Landing State
 Landing is tricky. Ground effect causes random perturbations in vehicle velocity, and we can't correct for that very well because we're so close to the ground we don't have much room to maneuver or margin for error. We just call the land service advertised by the [Drone Node](https://github.com/slaghuis/drone_mavsdk.git).  This will land and disarm the drone.  The state is returned to PENDING.
 
# Depends
This node needs the [Lander Interfaces](https://github.com/slaghuis/Lander_Interfaces) package.
Flight control is either being done by [Drone MAVSDK](https://github.com/slaghuis/drone_mavsdk) or [Drone RTPS](https://github.com/slaghuis/drone_rtps), but then any controller that uses velocity messages could be used (with some re-mapping possibly).
Some camera node is needed to publish the sensor_msgs/msg/image message to detect Aruco Markers on. A simple option is [Camera Lite](https://github.com/slaghuis/camera_lite.git).  Camera lite describes the installation of OpenVC that is needed to detect the Aruco markers.

# Warning
This code is being tested.  It has not flown.
