# Vison-based-lander
Uses a downward facing camera to see a landing target, and guides the drone to land there.  This node will consume and leverage topics with the xdrone nodes.

# Code Structure
This ROS2 node is an action server, based on a state machine code pattern.  The node is in Pending mode, waiting.  On receiving the call to action, the action server sets the node in Seek mode.  Seek mode flies to a given posiiton (typically the takeoff position).  Once the Tracker node detects the landing target, the node switches to Approaching state.  The drone is stablilised over the target before switching to "Descending" state.  When the drone is low enough, the flight controller "Land" isntruction is used to drop the last couple of centimeters and disarm the drone.

# Outstanding
As of 19 August 2021, the Tracker node is still outstanding.  This node is under development.  This node will read a sensor_msgs/msg/Image message use OpenCV to detect the landing target. Watch this space.
