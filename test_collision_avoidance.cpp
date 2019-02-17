#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <XBotInterface/RobotInterface.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/MinimumVelocity.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/utils/AutoStack.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

int main ( int argc, char ** argv )
{
    /* Init ROS */
    ros::init ( argc, argv, "control_interface" );
    ros::NodeHandle nh;

    /* Init robot and model */
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    auto robot = XBot::RobotInterface::getRobot ( path_to_config_file );
    auto model = XBot::ModelInterface::getModel ( path_to_config_file );

    Eigen::VectorXd q0_motor;
    Eigen::VectorXd qh_motor;
    Eigen::VectorXd qh, q, dq;
    robot->getMotorPosition ( q0_motor );
    robot->getRobotState ( "home", qh_motor );
    qh.resize ( 6+qh_motor.size() );
    qh << Eigen::VectorXd::Zero ( 6 ), qh_motor;
    model->setJointPosition ( qh );
    model->update();
    q = qh;

    string base_link = "torso_2";
    string left_arm_link = ( *robot ) ( "left_arm" ).getTipLinkName();
    auto left_arm_task = boost::make_shared<OpenSoT::tasks::velocity::Cartesian>
                         ( base_link + "_TO_" + left_arm_link,
                           qh,
                           *model,
                           left_arm_link,
                           base_link
                         );
    left_arm_task->setLambda ( 1 );
    left_arm_task->setOrientationErrorGain ( 1 );

    Eigen::Affine3d left_arm_initial_pose;
    model->getPose ( left_arm_link, base_link, left_arm_initial_pose );

    string right_arm_link = ( *robot ) ( "right_arm" ).getTipLinkName();
    auto right_arm_task = boost::make_shared<OpenSoT::tasks::velocity::Cartesian>
                          ( base_link + "_TO_" + right_arm_link,
                            qh,
                            *model,
                            right_arm_link,
                            base_link
                          );
    right_arm_task->setLambda ( 1 );
    right_arm_task->setOrientationErrorGain ( 1 );

    Eigen::Affine3d right_arm_initial_pose;
    model->getPose ( right_arm_link, base_link, right_arm_initial_pose );

    auto velocity_task = boost::make_shared<OpenSoT::tasks::velocity::MinimumVelocity> ( qh.size() );
    velocity_task->setLambda ( 0.1 );

    Eigen::VectorXd q_min, q_max;
    model->getJointLimits ( q_min, q_max );
    auto joint_limit_constraint = boost::make_shared<OpenSoT::constraints::velocity::JointLimits> ( qh, q_max, q_min );

    auto self_collsion_constraint = boost::make_shared<OpenSoT::constraints::velocity::SelfCollisionAvoidance> ( qh, *model, base_link, 1, 0.05, 1 );
    std::list<LinkPairDistance::LinksPair> whiteList;
    whiteList.emplace_back ( "arm1_7", "arm2_7" );
    self_collsion_constraint->setCollisionWhiteList ( whiteList );

    std::vector<std::string> interested_links = {"arm1_1", "arm1_2", "arm1_3", "arm1_4", "arm1_5", "arm1_6", "arm1_7", "arm1_8"};
//     std::map<std::string, Eigen::Affine3d> environment_collision_frames;
    Eigen::Affine3d collision_pose;
    collision_pose.translation() << 0.75, 0, 0.5; // in world frame
    collision_pose.linear() = Eigen::Matrix3d::Identity();
//     environment_collision_frames["env"] = collision_pose;
    std::map<std::string, boost::shared_ptr<fcl::CollisionObjectd>> envionment_collision_objects;
    std::shared_ptr<fcl::CollisionGeometryd> shape = std::make_shared<fcl::Boxd> ( 0.1, 0.6, 1.4 );
    boost::shared_ptr<fcl::CollisionObjectd> collision_object ( new fcl::CollisionObjectd ( shape ) );
    fcl::Transform3d shape_origin;
    shape_origin.translation() << 0.75, 0, 0.5; // in world frame
    shape_origin.linear() = Eigen::Matrix3d::Identity();
    collision_object->setTransform ( shape_origin );
    envionment_collision_objects["env"] = collision_object;
//     fcl::Transform3d shape_origin;
//     shape_origin.translation() << 0.75, 0, 0.5; // in world frame
//     shape_origin.linear() = Eigen::Matrix3d::Identity();
//     envionment_collision_objects["env"]->setTransform ( shape_origin );
    auto environment_collsion_constraint = boost::make_shared<OpenSoT::constraints::velocity::CollisionAvoidance> ( qh, *model, base_link, interested_links, envionment_collision_objects, 1, 0.05, 1 );

    auto autostack_ = boost::make_shared<OpenSoT::AutoStack> ( left_arm_task + right_arm_task + 0.1*velocity_task ); // + 0.2*postural_task%indices
    autostack_ << joint_limit_constraint;
    autostack_ << environment_collsion_constraint;
//     autostack_ << self_collsion_constraint;

    /* Create solver */
    double eps_regularization = 1e12;
    OpenSoT::solvers::solver_back_ends solver_backend = OpenSoT::solvers::solver_back_ends::qpOASES;
    auto solver = boost::make_shared<OpenSoT::solvers::iHQP> ( autostack_->getStack(),
                  autostack_->getBounds(),
                  eps_regularization,
                  solver_backend );

    /* visualization */
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker> ( "visualization_marker", 10 );
    visualization_msgs::Marker cube;
    cube.header.frame_id = "world";
    cube.header.stamp = ros::Time::now();
    cube.ns = "environment";
    cube.action = visualization_msgs::Marker::ADD;
    cube.id = 0;
    cube.type = visualization_msgs::Marker::CUBE;

    cube.scale.x = 0.1;
    cube.scale.y = 0.6;
    cube.scale.z = 1.4;
    tf::poseEigenToMsg ( collision_pose, cube.pose );

    cube.color.g = 1.0;
    cube.color.a = 0.5;

    /* Init time */
    uint control_frequency = 200;
    ros::Rate loop_rate ( control_frequency );
    double homing_period = 3;
    double start_time = ros::Time::now().toSec();
    while ( ros::ok() ) {
        ros::spinOnce();

        double time = ros::Time::now().toSec();
        if ( ( time - start_time ) <= homing_period ) {
            auto q_cmd = q0_motor + 0.5* ( 1-cos ( 3.1415* ( time - start_time ) /homing_period ) ) * ( qh_motor-q0_motor );
            robot->setPositionReference ( q_cmd );
            robot->setVelocityReference ( Eigen::VectorXd::Zero ( q_cmd.size() ) );
            robot->move();
        } else {

            double length = 0.5;
            double period = 5;
            Eigen::Affine3d desired_pose;
            double t = time - start_time - homing_period;
            desired_pose.linear() = left_arm_initial_pose.linear();
            desired_pose.translation() = left_arm_initial_pose.translation() + Eigen::Vector3d ( 1,0,0 ) *0.3*length* ( 1-std::cos ( 2*3.1415/period*t ) );
            left_arm_task->setReference ( desired_pose.matrix() );

            desired_pose.linear() = right_arm_initial_pose.linear();
            desired_pose.translation() = right_arm_initial_pose.translation() + Eigen::Vector3d ( 1,0,0 ) *0.3*length* ( 1-std::cos ( 2*3.1415/period*t ) );
            right_arm_task->setReference ( desired_pose.matrix() );

            std::map<std::string, KDL::Frame> kdl_frames;
            tf::transformEigenToKDL ( collision_pose, kdl_frames["env"] );
            environment_collsion_constraint->updateEnvironmentCollisionObjects ( kdl_frames );

            autostack_->update ( q );
            solver->solve ( dq );
            q += dq;

            model->setJointPosition ( q );
            model->update();

            robot->setReferenceFrom ( *model, XBot::Sync::Position );
            robot->move();

            marker_pub.publish ( cube );
        }

        loop_rate.sleep();
    }

}
