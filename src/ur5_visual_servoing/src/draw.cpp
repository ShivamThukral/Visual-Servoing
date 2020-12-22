//
// Created by vcr on 2020-12-21.
//

#include <iostream>
#include <ur5_visual_servoing/gnuplot_ci.h>
#include <stdlib.h>

/*--------plot-------*/

using namespace gnuplot_ci;
using namespace std;

//double xb[3] = {0,0,0};
//double dx[3] = {1, 1, 1};

int main(int argc, char** argv)
{

    //Plot of tracked features
    GP_handle G1("/usr/bin/", "Time (s)", "Image Features");
    G1.gnuplot_cmd("set terminal wxt");
    G1.gnuplot_cmd("plot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/trackedfeatures.txt'  u 1:2 w l t 'x'");
    G1.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/trackedfeatures.txt'  u 1:3 w l t 'y'");
    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1 font 'Arial,22'");
    G1.gnuplot_cmd("set output '/home/vcr/UBC/Research/simulation/ibvs_ws/results/trackedfeatures.eps'");
    G1.gnuplot_cmd("replot");

    //Plot of Joint velocity
    GP_handle G2("/usr/bin/", "Time (s)", "Joint velocity (rad/s)");
    G2.gnuplot_cmd("set terminal wxt");
    G2.gnuplot_cmd("plot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/joint_velocities.txt'  u 1:2 w l t '{/Symbol q}_1'");
    G2.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/joint_velocities.txt'  u 1:3 w l t '{/Symbol q}_2'");
    G2.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/joint_velocities.txt'  u 1:4 w l t '{/Symbol q}_3'");
    G2.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/joint_velocities.txt'  u 1:5 w l t '{/Symbol q}_4'");
    G2.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/joint_velocities.txt'  u 1:6 w l t '{/Symbol q}_5'");
    G2.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/joint_velocities.txt'  u 1:7 w l t '{/Symbol q}_6'");
    G2.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1 font 'Arial,22'");
    G2.gnuplot_cmd("set output '/home/vcr/UBC/Research/simulation/ibvs_ws/results/joint_velocities.eps'");
    G2.gnuplot_cmd("replot");

    //Plot of Joint velocity
    GP_handle G4("/usr/bin/", "time (s)", "Camera velocity (m/s)");
    G4.gnuplot_cmd("set terminal wxt");
    G4.gnuplot_cmd("plot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/camera_vel.txt'  u 1:2 w l t 'V_1'");
    G4.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/camera_vel.txt'  u 1:3 w l t 'V_2'");
    G4.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/camera_vel.txt'  u 1:4 w l t 'V_3'");
    G4.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/camera_vel.txt'  u 1:5 w l t 'V_4'");
    G4.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/camera_vel.txt'  u 1:6 w l t 'V_5'");
    G4.gnuplot_cmd("replot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/camera_vel.txt'  u 1:7 w l t 'V_6'");
    G4.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1 font 'Arial,22'");
    G4.gnuplot_cmd("set output '/home/vcr/UBC/Research/simulation/ibvs_ws/results/camera_vel.eps'");
    G4.gnuplot_cmd("replot");

    //Plot of Error in x
    GP_handle G31("/usr/bin/", "time (s)", "Error");
    G31.gnuplot_cmd("set terminal wxt");
    G31.gnuplot_cmd("plot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/error.txt'  u 1:2 w l t 'e_x'");
    G31.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1.5 font 'Arial,22'");
    G31.gnuplot_cmd("set output '/home/vcr/UBC/Research/simulation/ibvs_ws/results/error_x.eps'");
    G31.gnuplot_cmd("replot");

    //Plot of Error in y
    GP_handle G32("/usr/bin/", "time (s)", "Error");
    G32.gnuplot_cmd("set terminal wxt");
    G32.gnuplot_cmd("plot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/error.txt'  u 1:3 w l t 'e_y'");
    G32.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1.5 font 'Arial,22'");
    G32.gnuplot_cmd("set output '/home/vcr/UBC/Research/simulation/ibvs_ws/results/error_y.eps'");
    G32.gnuplot_cmd("replot");

//    //    //Plot of Error in z
//    GP_handle G33("/usr/bin/", "time (s)", "Error");
//    G33.gnuplot_cmd("set terminal wxt");
//    G33.gnuplot_cmd("plot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/error.txt'  u 1:4 w l t 'norm error'");
//    G33.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1.5 font 'Arial,22'");
//    G33.gnuplot_cmd("set output '/home/vcr/UBC/Research/simulation/ibvs_ws/results/norm-error.eps'");
//    G33.gnuplot_cmd("replot");

    //Plot of Error in z
    GP_handle G33("/usr/bin/", "time (s)", "Error");
    G33.gnuplot_cmd("set terminal wxt");
    G33.gnuplot_cmd("plot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/error.txt'  u 1:4 w l t 'e_z'");
    G33.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1.5 font 'Arial,22'");
    G33.gnuplot_cmd("set output '/home/vcr/UBC/Research/simulation/ibvs_ws/results/error_z.eps'");
    G33.gnuplot_cmd("replot");

    //Plot of norm-error
    GP_handle G34("/usr/bin/", "time (s)", "Error (pixels)");
    G34.gnuplot_cmd("set terminal wxt");
    G34.gnuplot_cmd("plot '/home/vcr/UBC/Research/simulation/ibvs_ws/results/error.txt'  u 1:5 w l t 'norm error");
    G34.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1.5 font 'Arial,22'");
    G34.gnuplot_cmd("set output '/home/vcr/UBC/Research/simulation/ibvs_ws/results/norm-error.eps'");
    G34.gnuplot_cmd("replot");

    cout << "Press enter to save plots" << endl;
    getchar();
    return 0;
}
