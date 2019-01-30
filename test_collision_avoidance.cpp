#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <XBotInterface/RobotInterface.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/MinimumVelocity.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
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

//     auto postural_task = boost::make_shared<OpenSoT::tasks::velocity::Postural> ( qh_ );
//     postural_task->setLambda ( 0.1 );
// //     std::list<uint> indices = {30};
// //     std::list<uint> indices = {30, 38, 39, 40, 41, 42, 43, 44};
//     std::list<uint> indices = {38, 39, 40, 41, 42, 43, 44, 45, 46, 47};

    auto velocity_task = boost::make_shared<OpenSoT::tasks::velocity::MinimumVelocity> ( qh.size() );
    velocity_task->setLambda ( 0.1 );

    Eigen::VectorXd q_min, q_max;
    model->getJointLimits ( q_min, q_max );
    auto joint_limit_constraint = boost::make_shared<OpenSoT::constraints::velocity::JointLimits> ( qh, q_max, q_min );

    auto autostack_ = boost::make_shared<OpenSoT::AutoStack> ( left_arm_task + 0.1*velocity_task ); // + 0.2*postural_task%indices
    autostack_ << joint_limit_constraint;

    /* Create solver */
    double eps_regularization = 1e12;
    OpenSoT::solvers::solver_back_ends solver_backend = OpenSoT::solvers::solver_back_ends::qpOASES;
    auto solver = boost::make_shared<OpenSoT::solvers::iHQP> ( autostack_->getStack(),
                  autostack_->getBounds(),
                  eps_regularization,
                  solver_backend );

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
        desired_pose.translation() = left_arm_initial_pose.translation() + Eigen::Vector3d ( 0,1,0 ) *0.5*length* ( 1-std::cos ( 2*3.1415/period*t ) );
        left_arm_task->setReference ( desired_pose.matrix() );

        autostack_->update ( q );
        solver->solve ( dq );
        q += dq;

        model->setJointPosition ( q );
        model->update();

        robot->setReferenceFrom ( *model, XBot::Sync::Position );
        robot->move();
        }

        loop_rate.sleep();
    }

}
