#include <ros/ros.h>
#include <joint_trajectory_generator/trajectory_generation.h>

class gqsTraj
{
private:
  int gqsJointSize;
  double max_vel;
  double max_acc;

  std::unique_ptr<trajectory::TrajectoryGenerator> g;
  ros::NodeHandle n;

public:
  gqsTraj(int gqs_joints, double max_v, double max_a);
  void trajSmoothCallback(const trajectory_msgs::JointTrajectory& traj_in);
  ros::Subscriber subscribeTraj;
  ros::Publisher publishTraj;
};

gqsTraj::gqsTraj(int gqs_joints, double max_v, double max_a)
{
  this->gqsJointSize = gqs_joints;
  this->max_vel = max_v;
  this->max_acc = max_a;

  // generate ros details
  this->subscribeTraj = this->n.subscribe("/garbage_quick_sort/gqsTrajRough", 1, &gqsTraj::trajSmoothCallback, this);
  this->publishTraj = this->n.advertise<trajectory_msgs::JointTrajectory>("/garbage_quick_sort/gqsTrajSmooth", 1);
  this->g = std::make_unique<trajectory::TrajectoryGenerator>(this->max_vel, this->max_acc, this->gqsJointSize);
}

void gqsTraj::trajSmoothCallback(const trajectory_msgs::JointTrajectory& traj_in)
{
  trajectory_msgs::JointTrajectory traj_out;
  // call the smoothing library
  this->g->generate(traj_in, traj_out);

  // publish
  this->publishTraj.publish(traj_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gqs_traj_smooth_node");
  gqsTraj gqsTrajSmoother(4, 12.0, 1.0);

  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}