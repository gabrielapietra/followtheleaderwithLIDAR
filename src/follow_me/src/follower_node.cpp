#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <math.h>

#ifdef USE_MATPLOT
#include <matplot/matplot.h>
using namespace matplot;
#endif

// #define SHOW_POSITION_LOGS
#define SHOW_SENSOR_LOGS
// Turtlebot3 Burger Parameters
#define LIDAR_SAMPLES 360
#define LIDAR_MAX_RANGE 3.5

class Follow {
private:
  // master
  ros::Subscriber masterGetPose_; // leitura odom
  geometry_msgs::Pose2D masterPose_;

  // follower
  ros::Subscriber followerGetPose_; // leitura odom
  geometry_msgs::Pose2D followerPose_;

  // control
  ros::Publisher FollowerControl_;

  //lidar
    std::vector<float> buffer_lidar_;
    std::vector<float> lidar_diff_;
    bool is_in_motion = false;
    bool save_sensor_buffer = true;


public:
  Follow(ros::NodeHandle nh) :
    buffer_lidar_(LIDAR_SAMPLES, 0), lidar_diff_(LIDAR_SAMPLES, 0) 
  {
    masterGetPose_ =
        nh.subscribe("tb3_0/scan", 100, &Follow::getMasterPose, this);
    followerGetPose_ =
        nh.subscribe("tb3_1/odom", 100, &Follow::getFollowerPose, this);
    FollowerControl_ = nh.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel", 100);
  }

  void getMasterPose(const sensor_msgs::LaserScan::ConstPtr msg) {
    int indice = 0;
    auto sensor = std::vector<float>(msg->ranges.size());

    // replace inf to LIDAR_MAX_RANGE
    std::transform(
        msg->ranges.begin(), msg->ranges.end(), sensor.begin(),
        [](float it) { return (it > LIDAR_MAX_RANGE) ? LIDAR_MAX_RANGE : it; });

    int max_number = 0; // max_number é o maior valor do vetor lidar_diff_
    int indice = 0; //indice_max_number é o indice do maior valor 
    //representando (posição, graus)
     
    for (size_t i = 0; i < LIDAR_SAMPLES; i++){
        lidar_diff_[i] = abs(lidar_diff_[i]);

        if(lidar_diff_[i] > max_number)
        {
            max_number = lidar_diff_[i];
            indice = i;
        }
    }

    int x = max_number * cos(indice * M_PI / 180);
    int y = max_number * sin(indice * M_PI / 180);

    //copy sensor to buffer
      if (!is_in_motion && save_sensor_buffer)
      {
      	std::copy(sensor.begin(), sensor.end(), buffer_lidar_.begin());
	    save_sensor_buffer = false;
#ifdef SHOW_POSITION_LOGS
      ROS_INFO("[Follower] Save lidar buffer data!");
#endif
      }

#ifdef SHOW_SENSOR_LOGS
      ROS_INFO("lidar_diff_ Data:");
      for(auto it = std::begin(lidar_diff_); it != std::end(lidar_diff_); ++it)
	std::cout << *it << ", ";
      std::cout << std::endl;
#endif

    //
    // your code here
    //

    masterPose_.x = x;
    masterPose_.y = y;

#ifdef USE_MATPLOT
    std::vector<double> theta = linspace(0, 2 * pi, LIDAR_SAMPLES);
	plot(theta, lidar_diff_);
	ylim({-LIDAR_MAX_RANGE - 0.1, LIDAR_MAX_RANGE + 0.1});
	xlim({0 , 2 * pi});
#endif

#ifdef SHOW_POSITION_LOGS 
      ROS_INFO("[Master] x:%.2lf, y: %.2lf, theta: %.2lf", masterPose_.x, masterPose_.y, masterPose_.theta);
#endif

  }

  void getFollowerPose(const nav_msgs::Odometry::ConstPtr msg) {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose2d.theta = yaw;

    ROS_INFO("[Follower] x: %.2lf, y: %.2lf, theta: %.2lf", pose2d.x, pose2d.y,
             pose2d.theta);
  }

  void moveToTarget() {
    // control
    geometry_msgs::Twist command;

    double vk_p, v_t, k_pa, k_pl;
    double maxLinearVel, maxAngVel;
    double omega_t = 0, theta_r, error, ang;

    k_pl = 1.5;
    k_pa = 6;

    // p n haver escorregamento das rodas, padrões definidos pelo robô
    maxLinearVel = 0.22;
    maxAngVel = 2.84;

    error = sqrt(pow(masterPose_.x - followerPose_.x, 2) +
                 pow(masterPose_.y - followerPose_.y, 2));
    v_t = k_pl * error;

    // linear
    if (v_t > maxLinearVel)
      v_t = maxLinearVel;

    // n bater quando ficar próximo de 25 cm
    if (error < 0.25)
      command.linear.x = 0.0;

    else
      command.linear.x = v_t;

    // angular
    theta_r = atan2((masterPose_.y - followerPose_.y),
                    (masterPose_.x - followerPose_.x));
    omega_t = k_pa * (theta_r - followerPose_.theta);

    if (abs(omega_t) > maxAngVel) {
      if (omega_t < 0)
        omega_t = -maxAngVel;
      else
        omega_t = maxAngVel;
    }

    command.angular.z = omega_t;

    FollowerControl_.publish(command);

    #ifdef SHOW_POSITION_LOGS
      ROS_INFO("[Follower] Linear Control %.2f, Angular Control: %.2f", v_t, omega_t);
    #endif

  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tb3_follow_node");
  ros::NodeHandle nh;
  Follow follower(nh);
  ros::Rate loop_rate(0.5);

  while (ros::ok()) 
  {
    ros::spinOnce();
    follower.moveToTarget();
    loop_rate.sleep();

  }
  return 0;
}