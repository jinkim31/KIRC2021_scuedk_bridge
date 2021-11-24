#include "Node.h"

Node::Node(int argc, char **argv)
{
    cout<<"scuedk_bridge started."<<endl;
    ros::init(argc, argv, "scuedk_bridge");
    ros::NodeHandle n;

    robot = make_unique<Scue>(n);
    initJointMap();

    ros::Subscriber twistSub = n.subscribe("cmd_vel", 10, &Node::twistCallback, this);
    ros::Subscriber jointSub = n.subscribe("joint_state", 10, &Node::jointCallback, this);
    tf::TransformBroadcaster odom_broadcaster;

    cout<<sizeof(FlipperController)<<" "<<sizeof(TrackController)<<endl;
    int a=0;
    ros::Rate loopRate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }
}

Node::~Node()
{

}

void Node::twistCallback(const geometry_msgs::Twist_<allocator<void>>::ConstPtr &msg)
{
    robot->set(robot->ref.trackController.tracks[0].velPid.target, 10.0f);
    robot->set(robot->ref.trackController.tracks[1].velPid.target, 10.0f);
}

void Node::jointCallback(const sensor_msgs::JointState_<allocator<void>>::ConstPtr &msg)
{
    robot->set(robot->ref.manipulator.targetPosition[jointMap.find(msg->name[0])->second], limit(msg->position[0], -179.0, 179.0));
    robot->set(robot->ref.manipulator.targetPosition[jointMap.find(msg->name[1])->second], limit(-180.0+msg->position[1], -179.0, 0.0));
    robot->set(robot->ref.manipulator.targetPosition[jointMap.find(msg->name[2])->second], limit(180.0+msg->position[2], 0.0, 179.0));
    robot->set(robot->ref.manipulator.targetPosition[jointMap.find(msg->name[3])->second], limit(msg->position[3], -179.0, 179.0));
    robot->set(robot->ref.manipulator.targetPosition[jointMap.find(msg->name[4])->second], limit(msg->position[4], -90.0, 90.0));
    robot->set(robot->ref.manipulator.targetPosition[jointMap.find(msg->name[5])->second], limit(msg->position[5], -179.0, 179.0));

    for(int i=0; i<msg->name.size(); i++)
    {
        int jointIdx = jointMap.find(msg->name[i])->second;
        robot->set(robot->ref.manipulator.targetAcceleration[jointIdx], msg->effort[i]);
    }
    robot->applySet(true);
}

void Node::initJointMap()
{
    jointMap.insert(pair<string, int>("joint0", 0));
    jointMap.insert(pair<string, int>("joint1", 1));
    jointMap.insert(pair<string, int>("joint2", 2));
    jointMap.insert(pair<string, int>("joint3", 3));
    jointMap.insert(pair<string, int>("joint4", 4));
    jointMap.insert(pair<string, int>("joint5", 5));
}

double  Node::limit(const double &x, const double& min, const double& max)
{
    if(x > max) return max;
    if(x < min) return min;
    return x;
}