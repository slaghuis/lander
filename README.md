# Vison-based-lander
Uses a downward facing camera to see a landing target, and guides the drone to land there.  This node will consume and leverage topics with the xdrone nodes.

# Depends
This node needs to run the [Tracker Node](https://github.com/slaghuis/Tracker) to do the landing target detection, and publish tracking messages.
Flight control is either being doen by [Drone MAVSDK](https://github.com/slaghuis/drone_mavsdk) or [Drone RTPS](https://github.com/slaghuis/drone_rtps), but then any controller that uses velocity mesahges could be used (with some re-mapping possibly)
# Code Structure
This ROS2 node is an action server, based on a state machine code pattern.  The node is in Pending mode, waiting.  On receiving the call to action, the action server sets the node in Seek mode.  Seek mode flies to a given posiiton (typically the takeoff position).  Once the Tracker node detects the landing target, the node switches to Approaching state.  The drone is stablilised over the target before switching to "Descending" state.  When the drone is low enough, the flight controller "Land" instruction is used to drop the last couple of centimeters and disarm the drone.
# Warning
This code is being tested.  The interaction between the tracking messages received from the [Tracker Node](https://github.com/slaghuis/Tracker) and this code is not yet tested.  (August 2021)
