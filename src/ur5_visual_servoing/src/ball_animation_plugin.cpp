#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>

//http://gazebosim.org/tutorials?tut=animated_box#Animateboxcode

#define PI 3.14159265

namespace gazebo {
    class AnimatedBox : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            this->model = _parent;

            // create the animation
            gazebo::common::PoseAnimationPtr anim(
                    //name the animation "test",
                    //make it last 12 seconds,
                    // and set it on a repeat loop
                    new gazebo::common::PoseAnimation("test", 12.0, true));

            gazebo::common::PoseKeyFrame *key;
            //set the starting location
            key = anim->CreateKeyFrame(0.0);
            key->Translation(ignition::math::Vector3d(0.75, 0.975, 1.2));
            key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));
            double angle = 30.0;
            for (int i = 1; i <= 12; i++) {
                double x_new = 0.5 + 0.25 * cos(angle * i * PI / 180);
                double z_new = 1.2 + 0.25 * sin(angle * i * PI / 180);
                key = anim->CreateKeyFrame(i);
                key->Translation(ignition::math::Vector3d(x_new, 0.975, z_new));
                key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));
            }

/*
        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(0.8, 0.975, 1.4)); // top right corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(2);
        key->Translation(ignition::math::Vector3d(0.2, 0.975, 1.4)); // top left corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(4);
        key->Translation(ignition::math::Vector3d(0.2, 0.975, 1.0)); // bottom left corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(6);
        key->Translation(ignition::math::Vector3d(0.8, 0.975, 1.0)); // bottom right corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // go back to starting location
        key = anim->CreateKeyFrame(8);
        key->Translation(ignition::math::Vector3d(0.8, 0.975, 1.4)); // bottom right corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        */

            // set the animation
            _parent->SetAnimation(anim);
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}

