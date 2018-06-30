#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class JoyToTwist
{
public:
  JoyToTwist();


private:
  void joyCallback(const sensor_msgs::Joy& joy);

  ros::NodeHandle pnh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  std::string connection_mode_;

  enum JoyAxesUSB {
    LEFT_STICK_HORIZONTAL_U = 0,
    LEFT_STICK_VERTICAL_U = 1,
    L2_U = 2,
    RIGHT_STICK_HORIZONTAL_U = 3,
    RIGHT_STICK_VERTICAL_U = 4,
    R2_U = 5,
    CROSS_HORIZONTAL_U = 6,
    CROSS_VERTICAL_U = 7
  };

  enum JoyAxisBluetooth {
    LEFT_STICK_HORIZONTAL_B = 0,
    LEFT_STICK_VERTICAL_B = 1,
    RIGHT_STICK_HORIZONTAL_B = 2,
    L2_B = 3,
    R2_B = 4,
    RIGHT_STICK_VERTICAL_B = 5,
  };

};

JoyToTwist::JoyToTwist():
  l_scale_(0.5),
  a_scale_(4.0),
  pnh_("~")
{
  pnh_.param("scale_angular", a_scale_, a_scale_);
  pnh_.param("scale_linear", l_scale_, l_scale_);
  pnh_.param("mode", connection_mode_, std::string("default"));

  std::cout << "mode = " << connection_mode_ << std::endl;
  if (connection_mode_ != "usb" && connection_mode_ != "bluetooth"){
    std::cout << "connection_mode is neither usb or bluetooth."
                 " set usb as a default." << std::endl;
    connection_mode_ = "usb";
  }

  vel_pub_ = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = pnh_.subscribe("/joy", 1, &JoyToTwist::joyCallback, this);

}

void JoyToTwist::joyCallback(const sensor_msgs::Joy& joy)
{
  geometry_msgs::Twist vel;

  double joy_L_ver, joy_L2, joy_R2;
  if (connection_mode_ == "usb"){
    joy_L_ver = joy.axes[LEFT_STICK_VERTICAL_U];
    joy_L2 = joy.axes[L2_U];
    joy_R2 = joy.axes[R2_U];
  } else if (connection_mode_ == "bluetooth") {
    joy_L_ver = joy.axes[LEFT_STICK_VERTICAL_B];
    joy_L2 = joy.axes[L2_B];
    joy_R2 = joy.axes[R2_B];
  }

  vel.linear.x = l_scale_ * joy_L_ver;
  vel.angular.z = a_scale_ * (joy_R2 - joy_L2) / 2.0;

  vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_to_twist");
  JoyToTwist joytotwist;

  ros::spin();
}
