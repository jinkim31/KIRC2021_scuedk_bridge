#include "Node.h"

Node::Node(int argc, char **argv)
{
    // ROS init
    ros::init(argc, argv, "scuedk_bridge");
    ros::NodeHandle n;

    //ScueDK init
    robot = make_unique<Scue>(n);

    // ROS communications
    ros::Subscriber twistSub = n.subscribe("cmd_vel", 10, &Node::twistCallback, this);
    ros::Subscriber twistTurtleSub = n.subscribe("turtle1/cmd_vel", 10, &Node::twistCallback, this);
    ros::Subscriber jointSub = n.subscribe("joint_state", 10, &Node::jointCallback, this);
    jointReadPublisher = n.advertise<sensor_msgs::JointState>("joint_state_read",1);
    jointReadPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_read",1);
    tf::TransformBroadcaster odom_broadcaster;

    // ScueDK cycle handler
    robot->addCycleListener(make_shared<Scue::CycleListener>([&]
    {
        cout<<"cycle event. timeDelta:"<<robot->getTimeDelta()<<endl;

        // twist forward kinematics
        double wheelBase = 0.8;
        double wheelRadius = 0.15;

        geometry_msgs::Twist twist;
        double velL = robot->get(robot->ref.trackController.tracks[0].encoderReading) / robot->getTimeDelta() * wheelRadius; // in m/s
        double velR = robot->get(robot->ref.trackController.tracks[0].encoderReading) / robot->getTimeDelta() * wheelRadius; // in m/s
        twist.linear.x = (velL + velR) / 2; // in m/s
        twist.angular.z = (velL - twist.linear.x) / (wheelBase / 2);  //in rad/s

        //TF odometry

        // UI readout
    }));

    // main loop
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
    cout<<"twist callback"<<endl;

    // twist inverse kinematics
    robot->set(robot->ref.trackController.tracks[0].velPid.target, msg->linear.x*10 - msg->angular.z*10);
    robot->set(robot->ref.trackController.tracks[1].velPid.target, msg->linear.x*10 + msg->angular.z*10);
    robot->applySet();
}

void Node::jointCallback(const sensor_msgs::JointState_<allocator<void>>::ConstPtr &msg)
{
    for(int i=0; i<msg->name.size(); i++)
    {
        if     (msg->name[i] == "joint0") {robot->set(robot->ref.manipulator.targetPosition[0], limit(msg->position[i], -179.0, 179.0));
            robot->set(robot->ref.manipulator.targetAcceleration[0], msg->effort[i]);}
        else if(msg->name[i] == "joint1") {robot->set(robot->ref.manipulator.targetPosition[1], limit(-180.0+msg->position[i], -179.0, 0.0));
            robot->set(robot->ref.manipulator.targetAcceleration[1], msg->effort[i]);}
        else if(msg->name[i] == "joint2") {robot->set(robot->ref.manipulator.targetPosition[2], limit(180.0+msg->position[i], 0.0, 179.0));
            robot->set(robot->ref.manipulator.targetAcceleration[2], msg->effort[i]);}
        else if(msg->name[i] == "joint3") {robot->set(robot->ref.manipulator.targetPosition[3], limit(msg->position[i], -179.0, 179.0));
            robot->set(robot->ref.manipulator.targetAcceleration[3], msg->effort[i]);}
        else if(msg->name[i] == "joint4") {robot->set(robot->ref.manipulator.targetPosition[4], limit(msg->position[i], -90.0, 90.0));
            robot->set(robot->ref.manipulator.targetAcceleration[4], msg->effort[i]);}
        else if(msg->name[i] == "joint5") {robot->set(robot->ref.manipulator.targetPosition[5], limit(msg->position[i], -179.0, 179.0));
            robot->set(robot->ref.manipulator.targetAcceleration[5], msg->effort[i]);}
        else if(msg->name[i] == "flipper0") {robot->set(robot->ref.manipulator.targetPosition[0], msg->position[i]);}
        else if(msg->name[i] == "flipper1") {robot->set(robot->ref.manipulator.targetPosition[1], msg->position[i]);}
        else if(msg->name[i] == "flipper2") {robot->set(robot->ref.manipulator.targetPosition[2], msg->position[i]);}
        else if(msg->name[i] == "flipper3") {robot->set(robot->ref.manipulator.targetPosition[3], msg->position[i]);}
    }

    robot->applySet();
}


double  Node::limit(const double &x, const double& min, const double& max)
{
    if(x > max) return max;
    if(x < min) return min;
    return x;
}