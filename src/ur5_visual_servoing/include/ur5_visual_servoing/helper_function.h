//
// Created by shivam on 19/11/20.
//

#ifndef IBVS_WS_HELPER_FUNCTION_H
#define IBVS_WS_HELPER_FUNCTION_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <camshift_tracker/image_data.h>
#include <ur5_visual_servoing/joint_angles.h>
#include <ur5_visual_servoing/joint_states.h>

using namespace std;
using namespace cv;

//function to get transform from camera_link to base frame
void getCameraToBase(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(1));
    transform_listener.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform);
    return;
}

//function to get transform from ee_link to base frame
void getEEToBase(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(1));
    transform_listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), transform);
    return;
}

//function to get transform from base_link to ee_link
void getBaseToEE(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform("/shoulder_link", "/base_link", ros::Time(0), ros::Duration(1));
    transform_listener.lookupTransform("/shoulder_link", "/base_link", ros::Time(0), transform);
    return;
}

//function to get transform from ee_link to camera_link
void getEETocamera(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform("/camera_link","/ee_link",  ros::Time(0), ros::Duration(1));
    transform_listener.lookupTransform( "/camera_link","/ee_link", ros::Time(0), transform);
    return;
}

//function to get transform from camera_link to ee_link
void getCameraToEE(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform("/ee_link","/camera_link",  ros::Time(0), ros::Duration(2));
    transform_listener.lookupTransform("/ee_link","/camera_link", ros::Time(0), transform);
    return;
}

//function for image features callback
double x_point, y_point;
float centroid_depth;
double area;
double points_corner[8];
void imgFeatureCallback(const camshift_tracker::image_data  msg)
{
    x_point = msg.detected_point_x.data;
    y_point = msg.detected_point_y.data;
    centroid_depth = msg.detected_point_depth.data;
    area = msg.area.data;
}

//function for joint state callback
double jp1, jp2, jp3, jp4, jp5, jp6;
double jv1, jv2, jv3, jv4, jv5, jv6;
void jointStateCallback(const sensor_msgs::JointState & msg)
{
    jp1=0;jp2=0;jp3=0;jp4=0;jp5=0;jp6=0;
    double count;
    for (int i=0; i<50; i++)
    {
        jp1 = jp1+(double)msg.position[0];
        jp2 = jp2+(double)msg.position[1];
        jp3 = jp3+(double)msg.position[2];
        jp4 = jp4+(double)msg.position[3];
        jp5 = jp5+(double)msg.position[4];
        jp6 = jp6+(double)msg.position[5];

        jv1 = jv1+(double)msg.velocity[0];
        jv2 = jv2+(double)msg.velocity[1];
        jv3 = jv3+(double)msg.velocity[2];
        jv4 = jv4+(double)msg.velocity[3];
        jv5 = jv5+(double)msg.velocity[4];
        jv6 = jv6+(double)msg.velocity[5];

        count=i;
    }
    count++;
    jp1=jp1/count;
    jp2=jp2/count;
    jp3=jp3/count;
    jp4=jp4/count;
    jp5=jp5/count;
    jp6=jp6/count;

    jv1=jv1/count;
    jv2=jv2/count;
    jv3=jv3/count;
    jv4=jv4/count;
    jv5=jv5/count;
    jv6=jv6/count;
}

//function for joint state callback - Gazebo
double jp1g, jp2g, jp3g, jp4g, jp5g, jp6g;
double jv1g, jv2g, jv3g, jv4g, jv5g, jv6g;
void jointStateCallbackGazebo(const ur5_visual_servoing::joint_states & msg)
{
    jp1g = (double)msg.ang0.data;
    jp2g = (double)msg.ang1.data;
    jp3g = (double)msg.ang2.data;
    jp4g = (double)msg.ang3.data;
    jp5g = (double)msg.ang4.data;
    jp6g = (double)msg.ang5.data;

    jv1g = (double)msg.vel0.data;
    jv2g = (double)msg.vel1.data;
    jv3g = (double)msg.vel2.data;
    jv4g = (double)msg.vel3.data;
    jv5g = (double)msg.vel4.data;
    jv6g = (double)msg.vel5.data;
}

//function to get Jacobian of UR-Robot
void getRobotJacobian(double th[], cv::Mat &J)
{
    double d1,a2,a3,d4,d5,d6;
    d1=0.0895; /*base*/
    a2=0.425; /*upper arm*/
    a3=0.3923; /*lower arm*/
    d4=0.1092; /*wrist 2*/
    d5=0.0947; /*wrist 3*/
    d6=0.0823; /*tool mounting bracket*/

    double th1, th2, th3, th4, th5, th6;
    th1=th[0];
    th2=th[1];
    th3=th[2];
    th4=th[3];
    th5=th[4];
    th6=th[5];
    //        th1=-0.4238;
    //        th2=-2.2241;
    //        th3=1.7404;
    //        th4=1.6614;
    //        th5=-1.7116;
    //        th6=1.05697;

    //Row1
    double j11, j12, j13, j14, j15, j16;
    j11=d5*sin(th2 + th3 + th4)*sin(th1) - d4*cos(th1) - a2*cos(th2)*sin(th1) - d6*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) - a3*cos(th2)*cos(th3)*sin(th1) + a3*sin(th1)*sin(th2)*sin(th3);
    j12=-cos(th1)*(a3*sin(th2 + th3) - d5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) + a2*sin(th2) + d6*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));
    j13=-cos(th1)*(a3*sin(th2 + th3) + d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));
    j14=-cos(th1)*(d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));
    j15=d6*sin(th1)*sin(th5) - d6*cos(th1)*cos(th2)*cos(th5)*sin(th3)*sin(th4) - d6*cos(th1)*cos(th3)*cos(th5)*sin(th2)*sin(th4) - d6*cos(th1)*cos(th4)*cos(th5)*sin(th2)*sin(th3) + d6*cos(th1)*cos(th2)*cos(th3)*cos(th4)*cos(th5);
    j16=0;

    int i=0;
    J.at<double>(i,0) = j11;
    J.at<double>(i,1) = j12;
    J.at<double>(i,2) = j13;
    J.at<double>(i,3) = j14;
    J.at<double>(i,4) = j15;
    J.at<double>(i,5) = j16;

    //Row2
    double j21, j22, j23, j24, j25, j26;

    j21=a2*cos(th1)*cos(th2) - d4*sin(th1) - d6*cos(th5)*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3) + d6*cos(th2 + th3 + th4)*cos(th1)*sin(th5);
    j22=-sin(th1)*(a3*sin(th2 + th3) - d5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) + a2*sin(th2) + d6*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));
    j23=-sin(th1)*(a3*sin(th2 + th3) + d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));
    j24=-sin(th1)*(d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));
    j25=d6*cos(th2)*cos(th3)*cos(th4)*cos(th5)*sin(th1) - d6*cos(th1)*sin(th5) - d6*cos(th2)*cos(th5)*sin(th1)*sin(th3)*sin(th4) - d6*cos(th3)*cos(th5)*sin(th1)*sin(th2)*sin(th4) - d6*cos(th4)*cos(th5)*sin(th1)*sin(th2)*sin(th3);
    j26=0;

    //int i=1;
    J.at<double>(i+1,0) = j21;
    J.at<double>(i+1,1) = j22;
    J.at<double>(i+1,2) = j23;
    J.at<double>(i+1,3) = j24;
    J.at<double>(i+1,4) = j25;
    J.at<double>(i+1,5) = j26;

    //Row3
    double j31, j32, j33, j34, j35, j36;

    j31=0;
    j32=(d6*sin(th2 + th3 + th4 - th5))/2 - a3*cos(th2 + th3) - a2*cos(th2) - (d6*sin(th2 + th3 + th4 + th5))/2 + d5*sin(th2 + th3 + th4);
    j33=(d6*sin(th2 + th3 + th4 - th5))/2 - a3*cos(th2 + th3) - (d6*sin(th2 + th3 + th4 + th5))/2 + d5*sin(th2 + th3 + th4);
    j34=(d6*sin(th2 + th3 + th4 - th5))/2 - (d6*sin(th2 + th3 + th4 + th5))/2 + d5*sin(th2 + th3 + th4);
    j35=-d6*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2);
    j36=0;

    //int i=2;
    J.at<double>(i+2,0) = j31;
    J.at<double>(i+2,1) = j32;
    J.at<double>(i+2,2) = j33;
    J.at<double>(i+2,3) = j34;
    J.at<double>(i+2,4) = j35;
    J.at<double>(i+2,5) = j36;


    //Row4
    double j41, j42, j43, j44, j45, j46;
    j41=0;
    j42=-sin(th1);
    j43=-sin(th1);
    j44=-sin(th1);
    j45=-sin(th2 + th3 + th4)*cos(th1);
    j46=cos(th2 + th3 + th4)*cos(th1)*sin(th5) - cos(th5)*sin(th1);

    //   int i=3;
    J.at<double>(i+3,0) = j41;
    J.at<double>(i+3,1) = j42;
    J.at<double>(i+3,2) = j43;
    J.at<double>(i+3,3) = j44;
    J.at<double>(i+3,4) = j45;
    J.at<double>(i+3,5) = j46;

    //Row5
    double j51, j52, j53, j54, j55, j56;
    j51=0;
    j52=cos(th1);
    j53=cos(th1);
    j54=cos(th1);
    j55=-sin(th2 + th3 + th4)*sin(th1);
    j56=cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5);

    J.at<double>(i+4,0) = j51;
    J.at<double>(i+4,1) = j52;
    J.at<double>(i+4,2) = j53;
    J.at<double>(i+4,3) = j54;
    J.at<double>(i+4,4) = j55;
    J.at<double>(i+4,5) = j56;

    //Row6
    double j61, j62, j63, j64, j65, j66;
    j61=1;
    j62=0;
    j63=0;
    j64=0;
    j65=-cos(th2 + th3 + th4);
    j66=-sin(th2 + th3 + th4)*sin(th5);

    //    int i=5;
    J.at<double>(i+5,0) = j61;
    J.at<double>(i+5,1) = j62;
    J.at<double>(i+5,2) = j63;
    J.at<double>(i+5,3) = j64;
    J.at<double>(i+5,4) = j65;
    J.at<double>(i+5,5) = j66;

}

//function to get Image Jacobian
void computeImageJacobian(double u[], double z[], cv::Mat &L)
{
    double lambda=531.15; //531.15
    int i=0;
    L.at<double>(i,0) = -lambda / z[i];
    L.at<double>(i,1) = 0.0;
    L.at<double>(i,2) = u[i] / z[i];
    L.at<double>(i,3) = (u[i] * u[i+1]) / lambda;
    L.at<double>(i,4) = -((lambda * lambda) + (u[i] * u[i])) / lambda;
    L.at<double>(i,5) = u[i+1];

    L.at<double>(i+1,0) = 0;
    L.at<double>(i+1,1) = -lambda / z[i];
    L.at<double>(i+1,2) = u[i+1] / z[i];
    L.at<double>(i+1,3) = ((lambda * lambda) + (u[i+1] * u[i+1])) / lambda;
    L.at<double>(i+1,4) = (-u[i] * u[i+1]) / lambda;
    L.at<double>(i+1,5) = -u[i];
}

//function to get Jacobian from end-effector to camera
void JacobianEEtoCamera( cv::Mat vector_ec,cv::Mat New_Rot,cv::Mat &mul_Jacob)
{
    cv::Mat S_aec = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat S_aec_mul = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat tNew_Rot = cv::Mat(3, 3, CV_64F, 0.0);

    S_aec.at<double>(0,1) =  - vector_ec.at<double>(0,2);
    S_aec.at<double>(0,2) =    vector_ec.at<double>(0,1);
    S_aec.at<double>(1,0) =    vector_ec.at<double>(0,2);
    S_aec.at<double>(1,2) =  - vector_ec.at<double>(0,0);
    S_aec.at<double>(2,0) =  - vector_ec.at<double>(0,1);
    S_aec.at<double>(2,1) =    vector_ec.at<double>(0,0);

    //cout << "S_aec " << S_aec << endl;
    cv::transpose(New_Rot,tNew_Rot);
    //cout << "tNew_Rot " << tNew_Rot << endl;
    S_aec_mul = -S_aec*tNew_Rot;

    mul_Jacob.at<double>(0,0) =  1;
    mul_Jacob.at<double>(1,1) =  1;
    mul_Jacob.at<double>(2,2) =  1;

    mul_Jacob.at<double>(0,3) =  S_aec_mul.at<double>(0,0); mul_Jacob.at<double>(0,4) = S_aec_mul.at<double>(0,1); mul_Jacob.at<double>(0,5) = S_aec_mul.at<double>(0,2);
    mul_Jacob.at<double>(1,3) =  S_aec_mul.at<double>(1,0); mul_Jacob.at<double>(1,4) = S_aec_mul.at<double>(1,1); mul_Jacob.at<double>(1,5) = S_aec_mul.at<double>(1,2);
    mul_Jacob.at<double>(2,3) =  S_aec_mul.at<double>(2,0); mul_Jacob.at<double>(2,4) = S_aec_mul.at<double>(2,1); mul_Jacob.at<double>(2,5) = S_aec_mul.at<double>(2,2);

    mul_Jacob.at<double>(3,3) = tNew_Rot.at<double>(0,0); mul_Jacob.at<double>(3,4)=tNew_Rot.at<double>(0,1); mul_Jacob.at<double>(3,5)=tNew_Rot.at<double>(0,2);
    mul_Jacob.at<double>(4,3) = tNew_Rot.at<double>(1,0); mul_Jacob.at<double>(4,4)=tNew_Rot.at<double>(1,1); mul_Jacob.at<double>(4,5)=tNew_Rot.at<double>(1,2);
    mul_Jacob.at<double>(5,3) = tNew_Rot.at<double>(2,0); mul_Jacob.at<double>(5,4)=tNew_Rot.at<double>(2,1); mul_Jacob.at<double>(5,5)=tNew_Rot.at<double>(2,2);

}

void convertVelCameratoEEframe(cv::Mat Vc,cv::Mat Ve)
{
    tf::StampedTransform transform;
    tf::Vector3 tfOrigin;
    tf::Matrix3x3 tfRow;
    tf::Vector3 tfRowVec;

    double time = ros::Time::now().toSec();
    getCameraToEE(transform);
    double duration = ros::Time::now().toSec() - time;
    cout << "-------------------------teetocamera="<<duration<<"---------------------" << endl;

    Mat dce = Mat(3, 1, CV_64F, 0.0);
    tfOrigin = transform.getOrigin();
    dce.at<double>(0,0)=tfOrigin.getX();
    dce.at<double>(1,0)=tfOrigin.getY();
    dce.at<double>(2,0)=tfOrigin.getZ();
    //cout << "dce " << dce << endl;

    tfRow = transform.getBasis();
    Mat Rot = Mat(3, 3, CV_64F, 0.0);
    tfRowVec = tfRow.getRow(0);
    Rot.at<double>(0,0)=tfRowVec.getX();  Rot.at<double>(0,1)=tfRowVec.getY(); Rot.at<double>(0,2)=tfRowVec.getZ();
    tfRowVec = tfRow.getRow(1);
    Rot.at<double>(1,0)=tfRowVec.getX();  Rot.at<double>(1,1)=tfRowVec.getY(); Rot.at<double>(1,2)=tfRowVec.getZ();
    tfRowVec = tfRow.getRow(2);
    Rot.at<double>(2,0)=tfRowVec.getX();  Rot.at<double>(2,1)=tfRowVec.getY(); Rot.at<double>(2,2)=tfRowVec.getZ();
    //cout << "Rot " << Rot << endl;

    //Skew-symmetric matrix of dce vector
    cv::Mat S_dce = cv::Mat(3, 3, CV_64F, 0.0);
    S_dce.at<double>(0,1) =  -dce.at<double>(2,0);
    S_dce.at<double>(0,2) =   dce.at<double>(1,0);
    S_dce.at<double>(1,0) =   dce.at<double>(2,0);
    S_dce.at<double>(1,2) =  -dce.at<double>(0,0);
    S_dce.at<double>(2,0) =  -dce.at<double>(1,0);
    S_dce.at<double>(2,1) =   dce.at<double>(0,0);
    //cout << "S_ce " << S_ce << endl;

    //transformation 2nd term
    cv::Mat trans_2ndterm = cv::Mat(3, 3, CV_64F, 0.0);
    trans_2ndterm = S_dce*Rot;
    //cout << "trans2ndterm " << trans_2ndterm << endl;

    //transformation to convert camera velocity to ee-velocity
    cv::Mat trans = cv::Mat(6, 6, CV_64F, 0.0);

    Rot.copyTo(trans(Rect(0,0,Rot.cols,Rot.rows)));
    trans_2ndterm.copyTo(trans(Rect(3,0,trans_2ndterm.cols,trans_2ndterm.rows)));
    Rot.copyTo(trans(Rect(3,3,Rot.cols,Rot.rows)));
    //cout << "trans " << trans << endl;

    Ve = trans*Vc; //end-effector velocity
}

void newconvertVelCameratoEEframe(cv::Mat Vc,cv::Mat Ve)
{

    Mat dce = Mat(3, 1, CV_64F, 0.0);
    dce.at<double>(0,0)=-0.00277013;
    dce.at<double>(1,0)=0.0131105;
    dce.at<double>(2,0)=0.06704;
    //cout << "dce " << dce << endl;

    Mat Rot = Mat::eye(3,3,CV_64F);

    //Skew-symmetric matrix of dce vector
    cv::Mat S_dce = cv::Mat(3, 3, CV_64F, 0.0);
    S_dce.at<double>(0,1) =  -dce.at<double>(2,0);
    S_dce.at<double>(0,2) =   dce.at<double>(1,0);
    S_dce.at<double>(1,0) =   dce.at<double>(2,0);
    S_dce.at<double>(1,2) =  -dce.at<double>(0,0);
    S_dce.at<double>(2,0) =  -dce.at<double>(1,0);
    S_dce.at<double>(2,1) =   dce.at<double>(0,0);
    //cout << "S_ce " << S_ce << endl;

    //transformation 2nd term
    cv::Mat trans_2ndterm = cv::Mat(3, 3, CV_64F, 0.0);
    trans_2ndterm = S_dce*Rot;
    //cout << "trans2ndterm " << trans_2ndterm << endl;

    //transformation to convert camera velocity to ee-velocity
    cv::Mat trans = cv::Mat(6, 6, CV_64F, 0.0);

    Rot.copyTo(trans(Rect(0,0,Rot.cols,Rot.rows)));
    trans_2ndterm.copyTo(trans(Rect(3,0,trans_2ndterm.cols,trans_2ndterm.rows)));
    Rot.copyTo(trans(Rect(3,3,Rot.cols,Rot.rows)));
    //cout << "trans " << trans << endl;

    Ve = trans*Vc; //end-effector velocity
}

//function to calculate forward kinematics as per URDriver frames. Input will be current joint angles (th)
//and identity matrix (homoTransform).
void getFKnumerical(double th[], cv::Mat &homoTransform, bool isGripper){

    //*******DH-Parameters of UR5*******//
    //link length (a)
    double a[6]={0,-0.42500,-0.39225,0,0,0};
    //joint offset (d)
    double d[6]={0.089459,0,0,0.10915,0.09465,0.0823};
    //link twist (alpha)
    double alp[6]={M_PI/2,0,0,M_PI/2,-M_PI/2,0};

    //    double DH[6][4]={{th[0],d[0],a[0],alp[0]},
    //                     {th[1],d[1],a[1],alp[1]},
    //                     {th[2],d[2],a[2],alp[2]},
    //                     {th[3],d[3],a[3],alp[3]},
    //                     {th[4],d[4],a[4],alp[4]},
    //                     {th[5],d[5],a[5],alp[5]}};

    Mat T = Mat::eye(4, 4, CV_64F);

    for (int i=0; i<6; i++)
    {
        T.at<double>(0,0)= cos(th[i]);
        T.at<double>(0,1)=-sin(th[i])*cos(alp[i]);
        T.at<double>(0,2)=sin(th[i])*sin(alp[i]);
        T.at<double>(0,3)=a[i]*cos(th[i]);

        T.at<double>(1,0)= sin(th[i]);
        T.at<double>(1,1)=cos(th[i])*cos(alp[i]);
        T.at<double>(1,2)=-cos(th[i])*sin(alp[i]);
        T.at<double>(1,3)=a[i]*sin(th[i]);

        T.at<double>(2,0)= 0;
        T.at<double>(2,1)=sin(alp[i]);
        T.at<double>(2,2)=cos(alp[i]);
        T.at<double>(2,3)=d[i];

        T.at<double>(3,0)=0;
        T.at<double>(3,1)=0;
        T.at<double>(3,2)=0;
        T.at<double>(3,3)=1;

        homoTransform = homoTransform*T;

        //cout << "Transform base_to_ee_link = " << endl << homoTransform << endl;

    }
    if(isGripper){
        //cout << "with Gripper" << endl;
        T.at<double>(0,0)= 0.0112915;
        T.at<double>(0,1)=-0.9999222;
        T.at<double>(0,2)=0.0052930;
        T.at<double>(0,3)=-0.02367;

        T.at<double>(1,0)= 0.9999362;
        T.at<double>(1,1)=0.0112925;
        T.at<double>(1,2)=0.0001614;
        T.at<double>(1,3)=-0.02147;

        T.at<double>(2,0)= -0.0002212;
        T.at<double>(2,1)=0.0052909;
        T.at<double>(2,2)=0.9999860;
        T.at<double>(2,3)=0.42685;

        T.at<double>(3,0)=0;
        T.at<double>(3,1)=0;
        T.at<double>(3,2)=0;
        T.at<double>(3,3)=1;

        homoTransform = homoTransform*T;
    }
    //cout << "Transform base_to_ee_link = " << endl << homoTransform << endl;
}

void newgetFKnumerical(double th[], cv::Mat &homoTransform, bool isGripper){

    //*******DH-Parameters of UR5*******//
    //link length (a)
    double a[6]={0,0.42500,0.39225,0,0,0};
    //joint offset (d)
    double d[6]={0.089459,0,0,0.10915,0.09465,0.0823};
    //link twist (alpha)
    double alp[6]={M_PI/2,0,0,M_PI/2,-M_PI/2,0};

    Mat T = Mat::eye(4, 4, CV_64F);

    for (int i=0; i<6; i++)
    {
        T.at<double>(0,0)= cos(th[i]);
        T.at<double>(0,1)=-sin(th[i])*cos(alp[i]);
        T.at<double>(0,2)=sin(th[i])*sin(alp[i]);
        T.at<double>(0,3)=a[i]*cos(th[i]);

        T.at<double>(1,0)= sin(th[i]);
        T.at<double>(1,1)=cos(th[i])*cos(alp[i]);
        T.at<double>(1,2)=-cos(th[i])*sin(alp[i]);
        T.at<double>(1,3)=a[i]*sin(th[i]);

        T.at<double>(2,0)= 0;
        T.at<double>(2,1)=sin(alp[i]);
        T.at<double>(2,2)=cos(alp[i]);
        T.at<double>(2,3)=d[i];

        T.at<double>(3,0)=0;
        T.at<double>(3,1)=0;
        T.at<double>(3,2)=0;
        T.at<double>(3,3)=1;

        homoTransform = homoTransform*T;

        //cout << "Transform base_to_ee_link = " << endl << homoTransform << endl;

    }
    if(isGripper){
        //cout << "with Gripper" << endl;
        T.at<double>(0,0)= 0.0112915;
        T.at<double>(0,1)=-0.9999222;
        T.at<double>(0,2)=0.0052930;
        T.at<double>(0,3)=-0.02367;

        T.at<double>(1,0)= 0.9999362;
        T.at<double>(1,1)=0.0112925;
        T.at<double>(1,2)=0.0001614;
        T.at<double>(1,3)=-0.02147;

        T.at<double>(2,0)= -0.0002212;
        T.at<double>(2,1)=0.0052909;
        T.at<double>(2,2)=0.9999860;
        T.at<double>(2,3)=0.42685;

        T.at<double>(3,0)=0;
        T.at<double>(3,1)=0;
        T.at<double>(3,2)=0;
        T.at<double>(3,3)=1;

        homoTransform = homoTransform*T;
    }
    //cout << "Transform base_to_ee_link = " << endl << homoTransform << endl;
}


void convertVelEEtoBaseframe(cv::Mat Ve,cv::Mat Vb)
{
    //linear(V1) and angular(V2) camera velocity
    Mat V1 = Mat(3, 1, CV_64F, 0.0);
    Mat V2 = Mat(3, 1, CV_64F, 0.0);
    V1.at<double>(0,0)=Ve.at<double>(0,0);
    V1.at<double>(1,0)=Ve.at<double>(1,0);
    V1.at<double>(2,0)=Ve.at<double>(2,0);
    V2.at<double>(0,0)=Ve.at<double>(3,0);
    V2.at<double>(1,0)=Ve.at<double>(4,0);
    V2.at<double>(2,0)=Ve.at<double>(5,0);

    //get rotation matrix from base frame to
    tf::StampedTransform transform;
    tf::Matrix3x3 tfRow;
    tf::Vector3 tfRowVec;

    double time = ros::Time::now().toSec();
    getEEToBase(transform);
    //getBaseToEE(transform);

    //cout << "transform Origin "<< transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " " << transform.getOrigin().getZ() << endl;

    tfRow = transform.getBasis();
    Mat Rot = Mat(3, 3, CV_64F, 0.0);
    tfRowVec = tfRow.getRow(0);
    Rot.at<double>(0,0)=tfRowVec.getX();  Rot.at<double>(0,1)=tfRowVec.getY(); Rot.at<double>(0,2)=tfRowVec.getZ();
    tfRowVec = tfRow.getRow(1);
    Rot.at<double>(1,0)=tfRowVec.getX();  Rot.at<double>(1,1)=tfRowVec.getY(); Rot.at<double>(1,2)=tfRowVec.getZ();
    tfRowVec = tfRow.getRow(2);
    Rot.at<double>(2,0)=tfRowVec.getX();  Rot.at<double>(2,1)=tfRowVec.getY(); Rot.at<double>(2,2)=tfRowVec.getZ();
    //cout << "Rot " << Rot << endl;

    //Mat homoTransform = Mat::eye(4, 4, CV_64F);
    //homoTransform.at<double>(0,0)=-1;
    //homoTransform.at<double>(1,1)=-1;
    double th[6]={jp1, jp2, jp3, jp4, jp5, jp6};
    //cout << "current joint angles " << jp1 << " " << jp2 << " " << jp3 << " " << jp4 << " " << jp5 << " " << jp6 << endl;
    //getFKnumerical(th,homoTransform,false);
    //Mat R = Mat(3, 3, CV_64F, 0.0);
    //R = homoTransform.colRange(0,3).rowRange(0,3);

    double duration = ros::Time::now().toSec() - time;
    cout << "-------------------------teetobase="<<duration<<"---------------------" << endl;

    //getchar();
    Mat V11 = Mat(3, 1, CV_64F, 0.0);
    Mat V22 = Mat(3, 1, CV_64F, 0.0);
    V11=Rot*V1;
    V22=Rot*V2;
    //    V11=R*V1;
    //    V22=R*V2;

    Vb.at<double>(0,0)=V11.at<double>(0,0);
    Vb.at<double>(1,0)=V11.at<double>(1,0);
    Vb.at<double>(2,0)=V11.at<double>(2,0);
    Vb.at<double>(3,0)=V22.at<double>(0,0);
    Vb.at<double>(4,0)=V22.at<double>(1,0);
    Vb.at<double>(5,0)=V22.at<double>(2,0);
}

void newConvertVelEEtoBaseframe(cv::Mat Ve,cv::Mat Vb)
{
    //double time = ros::Time::now().toSec();
    //linear(V1) and angular(V2) camera velocity
    Mat V1 = Mat(3, 1, CV_64F, 0.0);
    Mat V2 = Mat(3, 1, CV_64F, 0.0);
    V1.at<double>(0,0)=Ve.at<double>(0,0);
    V1.at<double>(1,0)=Ve.at<double>(1,0);
    V1.at<double>(2,0)=Ve.at<double>(2,0);
    V2.at<double>(0,0)=Ve.at<double>(3,0);
    V2.at<double>(1,0)=Ve.at<double>(4,0);
    V2.at<double>(2,0)=Ve.at<double>(5,0);

    //cout << "end-eff vel " << Ve << endl;

    Mat homoTransform = Mat::eye(4, 4, CV_64F);
    double th[6]={jp1, jp2, jp3, jp4, jp5, jp6};
    //cout << "current joint angles " << jp1 << " " << jp2 << " " << jp3 << " " << jp4 << " " << jp5 << " " << jp6 << endl;
    newgetFKnumerical(th,homoTransform,false);
    Mat Rot = Mat(3, 3, CV_64F, 0.0);
    Rot = homoTransform.colRange(0,3).rowRange(0,3);
    //cout << "Rot " << Rot << endl;
    //getchar();
    Mat Rot_inv = Mat::eye(4, 4, CV_64F);
    cv::invert(Rot, Rot_inv, cv::DECOMP_SVD);

    //getchar();
    Mat V11 = Mat(3, 1, CV_64F, 0.0);
    Mat V22 = Mat(3, 1, CV_64F, 0.0);
    V11=Rot_inv*V1;
    V22=Rot_inv*V2;

    Vb.at<double>(0,0)=V11.at<double>(0,0);
    Vb.at<double>(1,0)=V11.at<double>(1,0);
    Vb.at<double>(2,0)=V11.at<double>(2,0);
    Vb.at<double>(3,0)=V22.at<double>(0,0);
    Vb.at<double>(4,0)=V22.at<double>(1,0);
    Vb.at<double>(5,0)=V22.at<double>(2,0);

    //double duration = ros::Time::now().toSec() - time;
    //cout << "-------------------------teetobase="<<duration<<"---------------------" << endl;
}

//function to get Jacobian of UR-Robot
void newgetRobotJacobian(double th[], cv::Mat &J)
{
    double d1,a2,a3,d4,d5,d6;
    d1=0.0895; /*base*/
    a2=-0.425; /*upper arm*/
    a3=-0.3923; /*lower arm*/
    d4=0.1092; /*wrist 2*/
    d5=0.0947; /*wrist 3*/
    d6=0.0823; /*tool mounting bracket*/

    double th1, th2, th3, th4, th5, th6;
    th1=th[0];
    th2=th[1];
    th3=th[2];
    th4=th[3];
    th5=th[4];
    th6=th[5];
    //        th1=-0.4238;
    //        th2=-2.2241;
    //        th3=1.7404;
    //        th4=1.6614;
    //        th5=-1.7116;
    //        th6=1.05697;

    //Row1
    double j11, j12, j13, j14, j15, j16;
    j11=d6*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + d4*cos(th1) - a2*cos(th2)*sin(th1) - d5*sin(th2 + th3 + th4)*sin(th1) - a3*cos(th2)*cos(th3)*sin(th1) + a3*sin(th1)*sin(th2)*sin(th3);
    j12=-cos(th1)*(d5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) + a3*sin(th2 + th3) + a2*sin(th2) - d6*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));
    j13=cos(th1)*(d5*cos(th2 + th3 + th4) - a3*sin(th2 + th3) + d6*sin(th2 + th3 + th4)*sin(th5));
    j14=cos(th1)*(d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));
    j15=d6*cos(th1)*cos(th2)*cos(th5)*sin(th3)*sin(th4) - d6*sin(th1)*sin(th5) + d6*cos(th1)*cos(th3)*cos(th5)*sin(th2)*sin(th4) + d6*cos(th1)*cos(th4)*cos(th5)*sin(th2)*sin(th3) - d6*cos(th1)*cos(th2)*cos(th3)*cos(th4)*cos(th5);
    j16=0;

    int i=0;
    J.at<double>(i,0) = j11;
    J.at<double>(i,1) = j12;
    J.at<double>(i,2) = j13;
    J.at<double>(i,3) = j14;
    J.at<double>(i,4) = j15;
    J.at<double>(i,5) = j16;

    //Row2
    double j21, j22, j23, j24, j25, j26;
    j21=d6*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + d4*sin(th1) + a2*cos(th1)*cos(th2) + d5*sin(th2 + th3 + th4)*cos(th1) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3);
    j22=-sin(th1)*(d5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) + a3*sin(th2 + th3) + a2*sin(th2) - d6*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));
    j23= sin(th1)*(d5*cos(th2 + th3 + th4) - a3*sin(th2 + th3) + d6*sin(th2 + th3 + th4)*sin(th5));
    j24=-sin(th1)*(d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));
    j25=d6*cos(th1)*sin(th5) - d6*cos(th2)*cos(th3)*cos(th4)*cos(th5)*sin(th1) + d6*cos(th2)*cos(th5)*sin(th1)*sin(th3)*sin(th4) + d6*cos(th3)*cos(th5)*sin(th1)*sin(th2)*sin(th4) + d6*cos(th4)*cos(th5)*sin(th1)*sin(th2)*sin(th3);
    j26=0;

    //int i=1;
    J.at<double>(i+1,0) = j21;
    J.at<double>(i+1,1) = j22;
    J.at<double>(i+1,2) = j23;
    J.at<double>(i+1,3) = j24;
    J.at<double>(i+1,4) = j25;
    J.at<double>(i+1,5) = j26;

    //Row3
    double j31, j32, j33, j34, j35, j36;
    j31=0;
    j32=a3*cos(th2 + th3) - (d6*sin(th2 + th3 + th4 + th5))/2 + a2*cos(th2) + (d6*sin(th2 + th3 + th4 - th5))/2 + d5*sin(th2 + th3 + th4);
    j33=a3*cos(th2 + th3) - (d6*sin(th2 + th3 + th4 + th5))/2 + (d6*sin(th2 + th3 + th4 - th5))/2 + d5*sin(th2 + th3 + th4);
    j34=a3*cos(th2 + th3) - (d6*sin(th2 + th3 + th4 + th5))/2 + a2*cos(th2) + (d6*sin(th2 + th3 + th4 - th5))/2 + d5*sin(th2 + th3 + th4), a3*cos(th2 + th3) - (d6*sin(th2 + th3 + th4 + th5))/2 + (d6*sin(th2 + th3 + th4 - th5))/2 + d5*sin(th2 + th3 + th4);
    j35=(d6*sin(th2 + th3 + th4 - th5))/2 - (d6*sin(th2 + th3 + th4 + th5))/2 + d5*sin(th2 + th3 + th4);
    j36=0;

    //int i=2;
    J.at<double>(i+2,0) = j31;
    J.at<double>(i+2,1) = j32;
    J.at<double>(i+2,2) = j33;
    J.at<double>(i+2,3) = j34;
    J.at<double>(i+2,4) = j35;
    J.at<double>(i+2,5) = j36;


    //Row4
    double j41, j42, j43, j44, j45, j46;
    j41=0;
    j42=sin(th1);
    j43=sin(th1);
    j44=sin(th1);
    j45=sin(th2 + th3 + th4)*cos(th1);
    j46=cos(th5)*sin(th1) + cos(th2 + th3 + th4)*cos(th1)*sin(th5);

    //   int i=3;
    J.at<double>(i+3,0) = j41;
    J.at<double>(i+3,1) = j42;
    J.at<double>(i+3,2) = j43;
    J.at<double>(i+3,3) = j44;
    J.at<double>(i+3,4) = j45;
    J.at<double>(i+3,5) = j46;

    //Row5
    double j51, j52, j53, j54, j55, j56;
    j51=0;
    j52=-cos(th1);
    j53=-cos(th1);
    j54=-cos(th1);
    j55=sin(th2 + th3 + th4)*sin(th1);
    j56=-cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5);

    J.at<double>(i+4,0) = j51;
    J.at<double>(i+4,1) = j52;
    J.at<double>(i+4,2) = j53;
    J.at<double>(i+4,3) = j54;
    J.at<double>(i+4,4) = j55;
    J.at<double>(i+4,5) = j56;

    //Row6
    double j61, j62, j63, j64, j65, j66;
    j61=1;
    j62=0;
    j63=0;
    j64=0;
    j65=-cos(th2 + th3 + th4);
    j66=-sin(th2 + th3 + th4)*sin(th5);

    //    int i=5;
    J.at<double>(i+5,0) = j61;
    J.at<double>(i+5,1) = j62;
    J.at<double>(i+5,2) = j63;
    J.at<double>(i+5,3) = j64;
    J.at<double>(i+5,4) = j65;
    J.at<double>(i+5,5) = j66;

}

//function to get real-time camera velocity
void getRealCameraVel(cv::Mat &Vc_real)
{
    Mat dth = Mat::eye(6, 1, CV_64F);
    dth.at<double>(0,0) = jv1g;
    dth.at<double>(1,0) = jv2g;
    dth.at<double>(2,0) = jv3g;
    dth.at<double>(3,0) = jv4g;
    dth.at<double>(4,0) = jv5g;
    dth.at<double>(5,0) = jv6g;

    double th[6]={jp1g, jp2g, jp3g, jp4g, jp5g, jp6g};
    Mat Jg_real = Mat::eye(6, 6, CV_64F);
    getRobotJacobian(th,Jg_real);
    Vc_real = Jg_real*dth;

}

//function to calculate area-interaction-matrix
/*void area_interaction_matrix(double u[], double z[],double &area, cv::Mat &L)
{
    int i=0;
    double lambda=531.15;//531.15;
    double P1[3], P2[3], P3[3],vector1[3], vector2[3], normal[3];

    P1[0] = u[i];
    P1[1] = u[i+1];
    P1[2] = z[i];
    P2[0] = u[i+2];
    P2[1] = u[i+3];
    P2[2] = z[i+1];
    P3[0] = u[i+4];
    P3[1] = u[i+5];
    P3[2] = z[i+2];

    vector1[0] = P1[0] - P2[0];
    vector1[1] = P1[1] - P2[1];
    vector1[2] = P1[2] - P2[2];
    vector2[0] = P1[0] - P3[0];
    vector2[1] = P1[1] - P3[1];
    vector2[2] = P1[2] - P3[2];

    //Cross product formula
    normal[0] = (vector1[1] * vector2[2]) - (vector1[2] * vector2[1]);
    normal[1] = (vector1[2] * vector2[0]) - (vector1[0] * vector2[2]);
    normal[2] = (vector1[0] * vector2[1]) - (vector1[1] * vector2[0]);

    double A = normal[0];
    double B = normal[1];
    double C = normal[2];
    double D = (normal[0]* (-P1[0])) + (normal[1] * (-P1[1])) + ( normal[2] * (-P1[2]));

    // calculate moments
    double sum1=0;
    double sum2=0;
    double sum3=0;
    double sum4=0;
    double sum5=0;
    double sum6=0;
    int n = 4;
    double x[5] = {u[0], u[2], u[4], u[6], u[0]};
    double y[5] = {u[1], u[3], u[5], u[7], u[1]};
    //  %-----------  calculation---------------------------%
    for (int j=0; j < n; j++){
        sum1 = sum1 + ( x[j] * y[j+1] - x[j+1] * y[j]);    // % a calculation
        sum2 = sum2 + (( x[j]*y[j+1] - x[j+1]*y[j] ) * ( x[j] + x[j+1] ) );  //% alp_10 calc
        sum3 = sum3 + (( x[j] *y[j+1] - x[j+1] * y[j] ) * ( y[j] + y[j+1] ) );   //% alp_01 calc
        sum4 = sum4 + (( x[j]*y[j+1] - x[j+1]*y[j] ) * ( (2*x[j]*y[j]) + (x[j]*y[j+1]) + (x[j+1]*y[j]) + (2*x[j+1]*y[j+1] )) );   //% alp_11 calc
        sum5 = sum5 + (( x[j]*y[j+1] - x[j+1]*y[j] ) * ( x[j] * x[j] + (x[j]*x[j+1]) + x[j+1] * x[j+1] ) ); // /% alp_20 calc
        sum6 = sum6 + (( x[j]*y[j+1] - x[j+1]*y[j] ) * ( y[j] * y[j] + (y[j]*y[j+1]) + y[j+1] * y[j+1] ) );  // % alp_02 calc
    }

    double  alp_00 = sum1/2.0;
    area = alp_00; //0.008

    double  alp_10 = ( 1/(6*area) )  * sum2;
    double  alp_01 = ( 1/(6*area) )  * sum3;
    double  alp_11 = ( 1/(24*area) ) * sum4;
    double  alp_20 = ( 1/(12*area) ) * sum5;
    double  alp_02 = ( 1/(12*area) ) * sum6;

    double mu_11 = alp_11 - alp_10 * alp_01;
    double mu_20 = alp_20 - alp_10 * alp_10;
    double mu_02 = alp_02 - alp_01 * alp_01;

    double Xg = alp_10;
    double Yg = alp_01;
    double Zg = z[i];
    Zg =1/(A*Xg + B*Yg + C);

    double n11 = mu_11;
    double n20 = mu_20;
    double n02 = mu_02;

    double Xg_vz = (Xg/Zg) + 4 *( A * n20 + B * n11 );
    double Yg_vz = (Yg/Zg) + 4 *( A * n11 + B * n02 );
    double Xg_wx = ( Xg * Yg ) + 4 * n11;
    double Yg_wy = -Xg_wx;
    double Xg_wy = -((lambda * lambda)+ Xg * Xg + 4 * n20 );
    double Yg_wx = ((lambda * lambda) + Yg * Yg + 4 * n02 );

    L.at<double>(i,0) = (-lambda/Zg);
    L.at<double>(i,1) = 0;
    L.at<double>(i,2) = Xg_vz;
    L.at<double>(i,3) = Xg_wx/lambda;
    L.at<double>(i,4) = Xg_wy/lambda;
    L.at<double>(i,5) = Yg;

    L.at<double>(i+1,0) = 0;
    L.at<double>(i+1,1) = (-lambda/Zg);
    L.at<double>(i+1,2) = Yg_vz;
    L.at<double>(i+1,3) = Yg_wx/lambda;
    L.at<double>(i+1,4) = Yg_wy/lambda;
    L.at<double>(i+1,5) = -Xg;

    L.at<double>(i+2,0) = -area*A;
    L.at<double>(i+2,1) = -area*B;
    L.at<double>(i+2,2) = area*((3/Zg)-C);
    L.at<double>(i+2,3) = 3 * area * Yg;
    L.at<double>(i+2,4) = - 3 * area * Xg;
    L.at<double>(i+2,5) = 0;
}*/


#endif //IBVS_WS_HELPER_FUNCTION_H
