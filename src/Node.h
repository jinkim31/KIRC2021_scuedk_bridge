#ifndef SCUEDK_BRIDGE_NODE_H
#define SCUEDK_BRIDGE_NODE_H

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <scuedk.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace scue;
using namespace ros;

class Node
{
private:
    unique_ptr<Scue> robot;
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
    double limit(const double &x, const double& min, const double& max);
    Publisher twistReadPublisher, jointReadPublisher;
public:
    Node(int argc, char **argv);
    ~Node();
};


#endif //SCUEDK_BRIDGE_NODE_H
