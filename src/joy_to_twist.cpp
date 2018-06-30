#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class JoyToTwist
{
public:
  JoyToTwist();

private:
  void joyCallback(const sensor_msgs::Joy& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  enum JoyAxis {
    LEFT_STICK_HORIZONTAL = 0,
    LEFT_STICK_VERTICAL = 1,
    L2 = 2,
    RIGHT_STICK_HORIZONTAL = 3,
    RIGHT_STICK_VERTICAL = 4,
    R2 = 5,
    CROSS_HORIZONTAL = 6,
    CROSS_VERTICAL = 7
  };

};

JoyToTwist::JoyToTwist():
  l_scale_(0.5),
  a_scale_(4.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = nh_.subscribe("joy", 1, &JoyToTwist::joyCallback, this);

}

void JoyToTwist::joyCallback(const sensor_msgs::Joy& joy)
{
  geometry_msgs::Twist vel;
  vel.linear.x = l_scale_ * joy.axes[LEFT_STICK_VERTICAL];
  vel.angular.z = a_scale_ * (joy.axes[R2] - joy.axes[L2]) / 2.0;

  vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_to_twist");
  JoyToTwist joytotwist;

  ros::spin();
}
