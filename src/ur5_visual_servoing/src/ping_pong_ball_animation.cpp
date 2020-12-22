//
// Created by vcr on 2020-12-15.
//

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include <random>

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
                    new gazebo::common::PoseAnimation("ping_pong_test", 30.0, true));

            gazebo::common::PoseKeyFrame *key;
//            //set the starting location
//            key = anim->CreateKeyFrame(0.0);
//            key->Translation(ignition::math::Vector3d(0.75, 0.975, 1.2));
//            key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));
//            double angle = 30.0;
//            for (int i = 1; i <= 12; i++) {
//                double x_new = 0.5 + 0.25 * cos(angle * i * PI / 180);
//                double z_new = 1.2 + 0.25 * sin(angle * i * PI / 180);
//                key = anim->CreateKeyFrame(i);
//                key->Translation(ignition::math::Vector3d(0.75, 0.975, 1.2));
//                key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));
//               // key->Translation(ignition::math::Vector3d(x_new, 0.975, z_new));
//               // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));
//            }

        const int range_from  = 20;
        const int range_to    = 80;
        std::random_device                  rand_dev;
        std::mt19937                        generator(rand_dev());
        std::uniform_int_distribution<int>  distr(range_from, range_to);

        double rand_num = distr(generator);
        std::cout<<rand_num<<std::endl;
        rand_num = 77;

        double z_height = 0.81;
        double z_offset = 0.08;
        double y_height = 1.5;
        double y_offset = 0.45;
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d((rand_num/100.0), y_height, z_height)); // top right corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        z_height+=z_offset;
        y_height-=y_offset;
        key = anim->CreateKeyFrame(10);
        key->Translation(ignition::math::Vector3d((rand_num/100.0), y_height, z_height)); // top right corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
//
        z_height+=z_offset;
        y_height-=y_offset;
        key = anim->CreateKeyFrame(20);
        key->Translation(ignition::math::Vector3d((rand_num/100.0), y_height, z_height)); // top right corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
//
        z_height+=z_offset;
        y_height-=y_offset;
        key = anim->CreateKeyFrame(30);
        key->Translation(ignition::math::Vector3d((rand_num/100.0), y_height, z_height)); // top right corner
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
//
//        z_height+=z_offset;
//        y_height-=y_offset;
////        key = anim->CreateKeyFrame(8);
//        key->Translation(ignition::math::Vector3d((rand_num/100.0), y_height, z_height)); // top right corner
//        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
////
//        z_height+=z_offset;
//        y_height-=y_offset;
//        key = anim->CreateKeyFrame(10);
//        key->Translation(ignition::math::Vector3d((rand_num/100.0), y_height, z_height)); // top right corner
//        key->Rotation(ignition::math::Quaterniond(0, 0, 0));




            // set starting location of the box
//        key = anim->CreateKeyFrame(0);
//        key->Translation(ignition::math::Vector3d(0.8, 0.975, 1.4)); // top right corner
//        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

//        key = anim->CreateKeyFrame(2);
//        key->Translation(ignition::math::Vector3d(0.2, 0.975, 1.4)); // top left corner
//        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
//
//        key = anim->CreateKeyFrame(4);
//        key->Translation(ignition::math::Vector3d(0.2, 0.975, 1.0)); // bottom left corner
//        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
//
//        key = anim->CreateKeyFrame(6);
//        key->Translation(ignition::math::Vector3d(0.8, 0.975, 1.0)); // bottom right corner
//        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
//
//        // go back to starting location
//        key = anim->CreateKeyFrame(8);
//        key->Translation(ignition::math::Vector3d(0.8, 0.975, 1.4)); // bottom right corner
//        key->Rotation(ignition::math::Quaterniond(0, 0, 0));


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

