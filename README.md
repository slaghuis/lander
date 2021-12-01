# Warning
Do not use this code.  I am in the process of fundamentally rewriting it.  This time it will ue tf2 and aruco markers.  It is just a better option.

# Vison-based-lander
Receive guidance from a Tracker node and guides the drone to land there.  This node will consume and leverage topics with the xdrone nodes.

# Depends
This node needs to run the [Tracker Node](https://github.com/slaghuis/Tracker) to do the landing target detection, and publish tracking messages.  Tracker messages are defined in the [Lander Interfaces](https://github.com/slaghuis/Lander_Interfaces) package.
Flight control is either being done by [Drone MAVSDK](https://github.com/slaghuis/drone_mavsdk) or [Drone RTPS](https://github.com/slaghuis/drone_rtps), but then any controller that uses velocity mesahges could be used (with some re-mapping possibly)
# Code Structure
This ROS2 node is an action server, based on a state machine code pattern.  The node is in Pending mode, waiting.  On receiving the call to action, the action server sets the node in Seek mode.  Seek mode flies to a given posiiton (typically the takeoff position).  Once the Tracker node detects the landing target, the node switches to Approaching state.  The drone is stablilised over the target before switching to "Descending" state.  When the drone is low enough, the flight controller "Land" instruction is used to drop the last couple of centimeters and disarm the drone.
# Warning
This code is being tested.  The interaction between the tracking messages received from the [Tracker Node](https://github.com/slaghuis/Tracker) and this code is not yet tested.  (Simulation and then flight testing scheduled for September 2021)
