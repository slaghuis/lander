
#include "lander/lander_node.h"


LanderServer::LanderServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("lander_node", options)
{
  using namespace std::placeholders;

  this->TransitionTo(new PendingState);   // Setup the state machine in pending mode
  
  // Create a transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  
  // Create drone velocity publisher
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 1);
  
  // Declare node parameters    
  this->declare_parameter<float>("target_seek_altitude", DEFAULT_TARGET_SEEK_ALTITUDE);
  this->declare_parameter<float>("max_speed_xy", DEFAULT_MAX_SPEED_XY);
  this->declare_parameter<float>("max_speed_z", DEFAULT_MAX_SPEED_Z);
  this->declare_parameter<float>("max_yaw_speed", DEFAULT_MAX_YAW_SPEED);
  this->declare_parameter("pid_xy", std::vector<double>{0.7, 0.0, 0.0});
  this->declare_parameter("pid_z", std::vector<double>{0.7, 0.0, 0.0});
  this->declare_parameter("pid_yaw", std::vector<double>{0.7, 0.0, 0.0});    
  this->declare_parameter<float>("waypoint_radius_error", DEFAULT_WAYPOINT_RADIUS_ERROR);
  this->declare_parameter<float>("yaw_threshold", DEFAULT_YAW_THRESHOLD);
  this->declare_parameter<float>("altitude_threshold", DEFAULT_ALTITUDE_THRESHOLD);
  this->declare_parameter<int>("holddown", DEFAULT_HOLDDOWN);
  this->declare_parameter<std::string>("map_frame", "map");

  one_off_timer_ = this->create_wall_timer(
    1000ms, std::bind(&LanderServer::init, this));
 
}

void LanderServer::init() {
  // Only run this once.  Stop the timer that triggered this.
  this->one_off_timer_->cancel();
  
  // Read parameters that are needed in this class (where the variables has been declared)
  this->get_parameter("map_frame", map_frame_);
  
  set_landing_target_client = this->create_client<SetTarget>("lander/set_target");
  
  using namespace std::placeholders;
  
    // Setup the action server
  this->action_server_ = rclcpp_action::create_server<Land>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "lander/land",
    std::bind(&LanderServer::handle_goal, this, _1, _2),
    std::bind(&LanderServer::handle_cancel, this, _1),
    std::bind(&LanderServer::handle_accepted, this, _1));
  
    // We are open for business, sitting under a tree waiting for customers.
  
}

void LanderServer::TransitionTo(State *state) {
  RCLCPP_INFO(this->get_logger(), "Context: Transition to %s", typeid(*state).name());
  if (this->state_ != nullptr)
    delete this->state_;
  this->state_ = state;
  this->state_->set_context(this);
  this->execute_mission();
}

void LanderServer::execute_mission() {
  this->state_->execute_mission();
}

void LanderServer::abort() {
  this->state_->abort();
}

bool LanderServer::is_working() {
  return this->state_->is_working();
}

// ACTION SERVER ///////////////////////////////////////////////////////////////////////////////////////////// 
  rclcpp_action::GoalResponse LanderServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received land request on target %d", goal->target);
    (void)uuid;
    // Let's reject sequences that are over 9000
    //if (goal->order > 9000) {
    //  return rclcpp_action::GoalResponse::REJECT;
    //}
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse LanderServer::handle_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void LanderServer::execute(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Attempting a sensor directed landing on a Aruco target.");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Land::Feedback>();
    auto & distance_to_target = feedback->distance_to_target;
    auto result = std::make_shared<Land::Result>();
  
    // Read the target (There must be a better way of doing this)
    target_position_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    target_position_->pose.position.x = goal->pose.pose.position.x;
    target_position_->pose.position.y = goal->pose.pose.position.y;
    target_position_->pose.position.z = goal->pose.pose.position.z;        // Remember to overwrite this with the seeking altitude!
    target_position_->pose.orientation.x = goal->pose.pose.orientation.x;
    target_position_->pose.orientation.y = goal->pose.pose.orientation.y;
    target_position_->pose.orientation.z = goal->pose.pose.orientation.z;
    target_position_->pose.orientation.w = goal->pose.pose.orientation.w;
    
    // Tell the tracker node what target we will land on
    set_landing_target( goal->target );
    
    this->TransitionTo(new SeekingState);   // Start the sequence
    
    while ( rclcpp::ok() && is_working() ) {   // Check to see if we are back in pending
      
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->outcome = false;
        goal_handle->canceled(result);
        
        this->abort();  // Abort flight.  Place the craft in a stable position. 
        
        RCLCPP_INFO(this->get_logger(), "All flight movement is stopped!");
        return;
      }
      
      distance_to_target = read_distance_to_target();
      
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_DEBUG(this->get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->outcome = true;   // TODO : Check if the craft is landed and disarmed
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "The eagle has landed.");
    }
  }

  void LanderServer::handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    using namespace std::placeholders;
    
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&LanderServer::execute, this, _1), goal_handle}.detach();
  }

// SIMPLE SERVICE CLIENT /////////////////////////////////////////////////////////////////////////////////////
bool LanderServer::set_landing_target(int target_number) {
  RCLCPP_INFO(this->get_logger(), "Landing target %i selected.", target_number);
  
  auto request = std::make_shared<SetTarget::Request>();
  request->target = target_number;
  
  while (!set_landing_target_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service [lander/set_target]. Exiting.");
      return false;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Service [lander/set_target] not available, waiting again...");
  }

  auto result = set_landing_target_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Landing target set to %i", target_number);
    return true;
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to set the landing target.  Landing on the first target.");
  }
  
  return false;
}

// UTILITY FUNCTIONS /////////////////////////////////////////////////////////////////////////////////////////
bool LanderServer::read_position(double *x, double *y, double *z)
{
  std::string from_frame = "base_link"; 
  std::string to_frame = map_frame_.c_str();
    
  geometry_msgs::msg::TransformStamped transformStamped;
    
  // Look up for the transformation between map and base_link frames
  // and save the last position in the 'map' frame
  try {
    transformStamped = tf_buffer_->lookupTransform(
      to_frame, from_frame,
      tf2::TimePointZero);
      *x = transformStamped.transform.translation.x;
      *y = transformStamped.transform.translation.y;
      *z = transformStamped.transform.translation.z;
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->get_logger(), "Could not transform %s to %s: %s",
      to_frame.c_str(), from_frame.c_str(), ex.what());
    return false;  
  }
  return true;
}

bool LanderServer::read_position(double *x, double *y, double *z, double *w)
{
  std::string from_frame = "base_link"; 
  std::string to_frame = map_frame_.c_str();
    
  geometry_msgs::msg::TransformStamped transformStamped;
    
  // Look up for the transformation between map and base_link frames
  // and save the last position in the 'map' frame
  try {
    transformStamped = tf_buffer_->lookupTransform(
      to_frame, from_frame,
      tf2::TimePointZero);
      *x = transformStamped.transform.translation.x;
      *y = transformStamped.transform.translation.y;
      *z = transformStamped.transform.translation.z;
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->get_logger(), "Could not transform %s to %s: %s",
      to_frame.c_str(), from_frame.c_str(), ex.what());
    return false;  
  }
    
  // Orientation quaternion
  tf2::Quaternion q(
      transformStamped.transform.rotation.x,
      transformStamped.transform.rotation.y,
      transformStamped.transform.rotation.z,
      transformStamped.transform.rotation.w);

  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q);

  // Roll Pitch and Yaw from rotation matrix
  double roll, pitch; 
  m.getRPY(roll, pitch, *w);
   

  return true;
}

bool LanderServer::read_target_position(double *x, double *y, double *z, double *w)
{
  std::string from_frame = "base_camera"; 
  std::string to_frame = map_frame_.c_str();
    
  geometry_msgs::msg::TransformStamped transformStamped;
    
  // Look up for the transformation between map and base_link frames
  // and save the last position in the 'map' frame
  try {
    transformStamped = tf_buffer_->lookupTransform(
      to_frame, from_frame,
      tf2::TimePointZero);
      *x = transformStamped.transform.translation.x;
      *y = transformStamped.transform.translation.y;
      *z = transformStamped.transform.translation.z;
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->get_logger(), "Could not transform %s to %s: %s",
      to_frame.c_str(), from_frame.c_str(), ex.what());
    return false;  
  }
    
  // Orientation quaternion
  tf2::Quaternion q(
      transformStamped.transform.rotation.x,
      transformStamped.transform.rotation.y,
      transformStamped.transform.rotation.z,
      transformStamped.transform.rotation.w);

  // 3x3 Rotation matrix from quaternion
  tf2::Matrix3x3 m(q);

  // Roll Pitch and Yaw from rotation matrix
  double roll, pitch; 
  m.getRPY(roll, pitch, *w);
   
  return true;
}

bool LanderServer::target_is_close()
{
  std::string from_frame = "base_camera"; 
  std::string to_frame = map_frame_.c_str();
    
  geometry_msgs::msg::TransformStamped transformStamped;
    
  rclcpp::Time now = this->get_clock()->now();
  try {
    transformStamped = tf_buffer_->lookupTransform(
      to_frame, from_frame,
      now, 50ms);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->get_logger(), "No recent transform available from %s to %s: %s",
      to_frame.c_str(), from_frame.c_str(), ex.what());
    return false;  
  }
       
}

void LanderServer::set_velocity(geometry_msgs::msg::Twist setpoint)
{
  publisher_->publish(setpoint);
}

std::shared_ptr<geometry_msgs::msg::PoseStamped> LanderServer::landing_target_position()
{
  return target_position_;
}

double LanderServer::read_distance_to_target() 
{
  double x,y,z;
  
  if (read_position(&x, &y, &z) ) { 
  
    return sqrt( pow(x-target_position_->pose.position.x, 2) +
                 pow(y-target_position_->pose.position.y, 2) +
                 pow(z-target_position_->pose.position.z, 2) );     
  } else {
    return 0.0;
  }  
}

void LanderServer::stop_movement() {
  rclcpp::Rate loop_rate(2);
    
  geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    
  setpoint.angular.x = 0.0;
  setpoint.angular.y = 0.0;
  setpoint.angular.z = 0.0;

  setpoint.linear.x = 0.0;
  setpoint.linear.y = 0.0;  
  setpoint.linear.z = 0.0;  
  this->set_velocity(setpoint);        
  loop_rate.sleep();    
  this->set_velocity(setpoint); // Just to be sure :-)       
    
  return;
}

// MAIN //////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<LanderServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
