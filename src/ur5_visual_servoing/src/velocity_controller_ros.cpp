#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <string>
#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <ur5_visual_servoing/joint_vel.h>
#include <ur5_visual_servoing/joint_angles.h>
#include <ur5_visual_servoing/sim_variables.h>
#include <ur5_visual_servoing/joint_states.h>

#define NUM_OF_JOINTS 6

namespace gazebo {
    typedef const boost::shared_ptr<const ur5_visual_servoing::joint_vel> JointVelocityPtr;
    typedef const boost::shared_ptr<const ur5_visual_servoing::joint_angles> JointAnglePtr;
    typedef const boost::shared_ptr<const ur5_visual_servoing::sim_variables> SimVariablePtr;

    class VelocityControllerPlugin : public ModelPlugin {
    public:
        VelocityControllerPlugin() {}

    public:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            this->world = _model->GetWorld();
            this->world->SetGravity(ignition::math::Vector3d(0, 0, 0));

            std::cout << "Loading VelocityController Plugin \n";

            // Store the model pointer for convenience.
            this->model = _model;

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&VelocityControllerPlugin::OnUpdate, this, _1));

            this->joints.push_back(_model->GetJoint("ur5::shoulder_pan_joint"));
            this->joints.push_back(_model->GetJoint("ur5::shoulder_lift_joint"));
            this->joints.push_back(_model->GetJoint("ur5::elbow_joint"));
            this->joints.push_back(_model->GetJoint("ur5::wrist_1_joint"));
            this->joints.push_back(_model->GetJoint("ur5::wrist_2_joint"));
            this->joints.push_back(_model->GetJoint("ur5::wrist_3_joint"));

            // Setup a P-controller, with a gain of 0.1.
            this->pid = common::PID(10, 0, 0);

            // Apply the P-controller to the joint.
//            for(int i=0;i<NUM_OF_JOINTS;i++)
//            {
//                this->model->GetJointController()->SetVelocityPID(this->joints[i]->GetScopedName(), this->pid);
//                this->model->GetJointController()->SetVelocityTarget(this->joints[i]->GetScopedName(), 0);
//            }
            this->model->GetJointController()->SetVelocityPID(this->joints[0]->GetScopedName(), this->pid);
            this->model->GetJointController()->SetVelocityPID(this->joints[1]->GetScopedName(), this->pid);
            this->model->GetJointController()->SetVelocityPID(this->joints[2]->GetScopedName(), this->pid);
            this->model->GetJointController()->SetVelocityPID(this->joints[3]->GetScopedName(), this->pid);
            this->model->GetJointController()->SetVelocityPID(this->joints[4]->GetScopedName(), this->pid);
            this->model->GetJointController()->SetVelocityPID(this->joints[5]->GetScopedName(), this->pid);

            this->model->GetJointController()->SetVelocityTarget(this->joints[0]->GetScopedName(), 0);
            this->model->GetJointController()->SetVelocityTarget(this->joints[1]->GetScopedName(), 0);
            this->model->GetJointController()->SetVelocityTarget(this->joints[2]->GetScopedName(), 0);
            this->model->GetJointController()->SetVelocityTarget(this->joints[3]->GetScopedName(), 0);
            this->model->GetJointController()->SetVelocityTarget(this->joints[4]->GetScopedName(), 0);
            this->model->GetJointController()->SetVelocityTarget(this->joints[5]->GetScopedName(), 0);


            this->joints[0]->SetPosition(0, 0);
            this->joints[1]->SetPosition(0, -1);
            this->joints[2]->SetPosition(0, 2);
            this->joints[3]->SetPosition(0, -1);
            this->joints[4]->SetPosition(0, 0);
            this->joints[5]->SetPosition(0, 0);

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_ros_client_plugin",
                          ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_ros_client_plugin"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions soJointVel =
                    ros::SubscribeOptions::create<ur5_visual_servoing::joint_vel>(
                            "/joint_vel_cmd",
                            1,
                            boost::bind(&VelocityControllerPlugin::jointVelocityCallback, this, _1),
                            ros::VoidPtr(), &this->rosQueue);
            this->rosSubJointVel = this->rosNode->subscribe(soJointVel);

            ros::SubscribeOptions soJointPos =
                    ros::SubscribeOptions::create<ur5_visual_servoing::joint_angles>(
                            "/joint_angles_cmd",
                            1,
                            boost::bind(&VelocityControllerPlugin::jointAngleCallback, this, _1),
                            ros::VoidPtr(), &this->rosQueue);
            this->rosSubJointPos = this->rosNode->subscribe(soJointPos);

            ros::SubscribeOptions soSimVar =
                    ros::SubscribeOptions::create<ur5_visual_servoing::sim_variables>(
                            "/sim_variables_cmd",
                            1,
                            boost::bind(&VelocityControllerPlugin::simVariableCallback, this, _1),
                            ros::VoidPtr(), &this->rosQueue);
            this->rosSubSimVar = this->rosNode->subscribe(soSimVar);

            this->rosJointStatePub = this->rosNode->advertise<ur5_visual_servoing::joint_states>("/my_joint_states", 1);

            // Spin up the queue helper thread.
            this->rosQueueThread =
                    std::thread(std::bind(&VelocityControllerPlugin::QueueThread, this));
        }

    public:
        void OnUpdate(const common::UpdateInfo &) {
            ur5_visual_servoing::joint_states msg;
            msg.ang0.data = this->joints[0]->Position(0);
            msg.ang1.data = this->joints[1]->Position(0);
            msg.ang2.data = this->joints[2]->Position(0);
            msg.ang3.data = this->joints[3]->Position(0);
            msg.ang4.data = this->joints[4]->Position(0);
            msg.ang5.data = this->joints[5]->Position(0);

            msg.vel0.data = this->joints[0]->GetVelocity(0);
            msg.vel1.data = this->joints[1]->GetVelocity(0);
            msg.vel2.data = this->joints[2]->GetVelocity(0);
            msg.vel3.data = this->joints[3]->GetVelocity(0);
            msg.vel4.data = this->joints[4]->GetVelocity(0);
            msg.vel5.data = this->joints[5]->GetVelocity(0);

            // added shivam
//            for(int i=0;i<NUM_OF_JOINTS;i++)
//            {
//                msg.robot_angles.angles.data.push_back(this->joints[i]->Position(0)) ;
//                msg.robot_velocities.velocities.data.push_back(this->joints[i]->GetVelocity(0));
//            }

            this->rosJointStatePub.publish(msg);
        }

    private:
        void simVariableCallback(SimVariablePtr &msg) {

            this->world->SetPhysicsEnabled(msg->sim_enable.data);
        }

    private:
        void jointAngleCallback(JointAnglePtr &msg) {
            //Shivam : change this to for loop
//            for(int i=0;i<NUM_OF_JOINTS;i++)
//            {
//                this->joints[0]->SetPosition(0, msg->angles.data[i]);
//            }

            this->joints[0]->SetPosition(0, msg->ang0.data);
            this->joints[1]->SetPosition(0, msg->ang1.data);
            this->joints[2]->SetPosition(0, msg->ang2.data);
            this->joints[3]->SetPosition(0, msg->ang3.data);
            this->joints[4]->SetPosition(0, msg->ang4.data);
            this->joints[5]->SetPosition(0, msg->ang5.data);

        }

    private:
        void jointVelocityCallback(JointVelocityPtr &msg) {
            // Shivam : change this to for loop
//            for(int i=0;i<NUM_OF_JOINTS;i++)
//            {
//                this->model->GetJointController()->SetVelocityTarget(this->joints[i]->GetScopedName(), msg->velocities.data[i]);
//            }
            this->model->GetJointController()->SetVelocityTarget(this->joints[0]->GetScopedName(), msg->vel0.data);
            this->model->GetJointController()->SetVelocityTarget(this->joints[1]->GetScopedName(), msg->vel1.data);
            this->model->GetJointController()->SetVelocityTarget(this->joints[2]->GetScopedName(), msg->vel2.data);
            this->model->GetJointController()->SetVelocityTarget(this->joints[3]->GetScopedName(), msg->vel3.data);
            this->model->GetJointController()->SetVelocityTarget(this->joints[4]->GetScopedName(), msg->vel4.data);
            this->model->GetJointController()->SetVelocityTarget(this->joints[5]->GetScopedName(), msg->vel5.data);
        }

        // ROS helper function that processes messages
    private:
        void QueueThread() {
            static const double timeout = 0.01;
            while (this->rosNode->ok()) {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;

    private:
        gazebo::physics::WorldPtr world;

        //  A node use for ROS transport
    private:
        std::unique_ptr <ros::NodeHandle> rosNode;

        //  A ROS subscriber
    private:
        ros::Subscriber rosSubJointVel;
    private:
        ros::Subscriber rosSubJointPos;
    private:
        ros::Subscriber rosSubSimVar;
    private:
        ros::Publisher rosJointStatePub;

        // A ROS callbackqueue that helps process messages
    private:
        ros::CallbackQueue rosQueue;

        // \brief A thread the keeps running the rosQueue
    private:
        std::thread rosQueueThread;

        // Pointer to the model.
    private:
        physics::ModelPtr model;

        // vector of joints
    private:
        std::vector <physics::JointPtr> joints;

        // PID controller for the joint.
    private:
        common::PID pid;
    };

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(VelocityControllerPlugin)
}

