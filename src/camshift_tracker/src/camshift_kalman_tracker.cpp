//
// Created by shivam on 11/11/20.
//

//Mean-Shift-Algorithim for object tracking
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <camshift_tracker/image_data.h>
#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;


int iLastX = 0;
int iLastY = 0;
double xc, yc = 0.0;

static void help() {
    std::cout << "\nThis is a demo that shows mean-shift based tracking using Transparent API\n"
                 "You select a color objects such as your face and it tracks it.\n"
                 "This reads from video camera (0 by default, or the camera number the user enters\n"
                 "Usage: \n"
                 "   ./camshiftdemo [camera number]\n";

    std::cout << "\n\nHot keys: \n"
                 "\tESC - quit the program\n"
                 "\ts - stop the tracking\n"
                 "\tb - switch to/from backprojection view\n"
                 "\th - show/hide object histogram\n"
                 "\tp - pause video\n"
                 "\tc - use OpenCL or not\n"
                 "To initialize tracking, select the object with mouse\n";
}


void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}


void
pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud) {
    pcl::fromROSMsg(*input, *ptCloud);
}

int main(int argc, char **argv) {
    Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0, 180};
    const float *phranges = hranges;
    int vmin = 47, vmax = 255, smin = 118;


    // Kalman Filter
    int STATE_SIZE = 6;             // size of the state vector
    int MEASUREMENT_SIZE = 4;       // states which we can measure
    int CONTROL_SIZE = 0;           // control input size

    cv::KalmanFilter kf(STATE_SIZE, MEASUREMENT_SIZE, CONTROL_SIZE, CV_32F);
    cv::Mat state(STATE_SIZE, 1, CV_32F);                       // Actual state vector [x,y,v_x,v_y,w,h]
    cv::Mat estimatedState(STATE_SIZE, 1, CV_32F);              // Estimate State vector by kalman filter

    cv::Mat measurement(MEASUREMENT_SIZE, 1, CV_32F);    // [z_x,z_y,z_w,z_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(MEASUREMENT_SIZE, STATE_SIZE, CV_32F);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    //Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // Kalman Filter end

    ros::init(argc, argv, "object_tracking_kalman");
    ros::NodeHandle nh;
    image_transport::ImageTransport img_trans(nh);
    image_transport::Subscriber image_subscriber = img_trans.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1,
                                                                 boost::bind(pointCloudCallback, _1,
                                                                             boost::ref(ptcloud)));
    //Let the nodes subcribe to image and PCD data
    sleep(1);
    ros::spinOnce();

    ros::Publisher pub = nh.advertise<camshift_tracker::image_data>("/object_points", 10);
    camshift_tracker::image_data data;

    namedWindow("CamShift Demo", 0);

    while (image.empty() || ptcloud->empty()) {
        ros::spinOnce();
        cout << "empty image or pcd ... going to sleep...." << endl;
        sleep(0.5);
    }


    Mat display_image, hsv, hue, mask, mask_contour, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    bool paused = false;

    int notFoundCount = 0;
    double ticks = 0.0;
    bool found = false;
    double area;
    double processedImages = 0;
    bool startProcessingImage = false;
    double time, total_time = 10.0;


    while (ros::ok()) {
        pcl::PointXYZRGB pt;
        image.copyTo(display_image); // this image is used for plotting the results

        //cacluations for dT
        double prev_tick = ticks;
        ticks = (double) cv::getTickCount();
        double dT = (ticks - prev_tick) / cv::getTickFrequency(); //get dt in seconds

        if (!paused) {
            if (image.empty())
                break;
        }

        /* if the ball is found - predict for kalman filter and plot the results*/
        if (found) {
            //cout << "Inside Prediction: dT value set. " << endl;
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A
            cout << "dT:" << dT << endl;
            state = kf.predict();
            cout << "State ([x,y,v_x,v_y,w,h]) : " << state.at<float>(0, 0) << " "
                             << estimatedState.at<float>(1, 0) << " " << state.at<float>(2, 0) << " "
                             << state.at<float>(3, 0) << " " << state.at<float>(4, 0) << " "
                             << state.at<float>(5, 0) << endl;
            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            cv::Point center(state.at<float>(0),state.at<float>(1));
            cv::circle(display_image, center, 5, CV_RGB(255, 255, 0), -1);
            cv::rectangle(display_image, predRect, CV_RGB(255, 255, 0), 2);
        }



        // >>>>> Noise smoothing
        cv::Mat blur;
        cv::GaussianBlur(image, blur, cv::Size(5, 5), 3.0, 3.0);
        // <<<<< Noise smoothing

        // >>>>> HSV conversion
        cvtColor(blur, hsv, COLOR_BGR2HSV);     // convert BGR to HSV
        // <<<<< HSV conversion


        //int _vmin = vmin, _vmax = vmax;
        inRange(hsv, Scalar(0, smin, MIN(vmin, vmax)), Scalar(180, 256, MAX(vmin, vmax)),
                mask);        // filter out blue color as mask

        // >>>>> Improving the result
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        // <<<<< Improving the result

        cv::imshow("Mask", mask);
        cv::waitKey(20);

        // >>>>> Contours detection
        vector<vector<cv::Point> > contours;
        cv::findContours(mask, contours, CV_RETR_EXTERNAL,
                         CV_CHAIN_APPROX_NONE);
        // <<<<< Contours detection

        // >>>>> Filtering
        vector<vector<cv::Point> > balls;
        vector<cv::Rect> ballsBox;
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Rect bBox;
            bBox = cv::boundingRect(contours[i]);

            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                ratio = 1.0f / ratio;

            // Searching for a bBox almost square
            if (ratio > 0.75 && bBox.area() >= 400)
            {
                balls.push_back(contours[i]);
                ballsBox.push_back(bBox);
            }
        }
        // <<<<< Filtering

        cout << "Balls found:" << ballsBox.size() << endl;

        // >>>>> Detection result
        for (size_t i = 0; i < balls.size(); i++)
        {
            cout<<"check"<<endl;
            cv::drawContours(display_image, balls, i, CV_RGB(20,150,20), 1);
            cv::rectangle(display_image, ballsBox[i], CV_RGB(0,255,0), 2);

            cv::Point center;
            center.x = ballsBox[i].x + ballsBox[i].width / 2;
            center.y = ballsBox[i].y + ballsBox[i].height / 2;
            cv::circle(display_image, center, 2, CV_RGB(20,150,20), -1);

            stringstream sstr;
            sstr << "(" << center.x << "," << center.y << ")";
            cv::putText(display_image, sstr.str(),
                        cv::Point(center.x + 3, center.y - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
        }
        // <<<<< Detection result

        cout<<"check2"<<endl;
        // >>>>> Kalman Update
        if (balls.size() == 0) {

            notFoundCount++;
            //cout << "notFoundCount:" << notFoundCount << endl;
            if (notFoundCount >= 200) {
                ROS_ERROR("Object estimate is lost after 100 attempts.....");
                found = false;
            }
            ROS_WARN("Object Not in Frame .....");

        } else {
            notFoundCount = 0;

            measurement.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
            measurement.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
            measurement.at<float>(2) = (float)ballsBox[0].width;
            measurement.at<float>(3) = (float)ballsBox[0].height;

            if (!found) // First detection!
            {
                //getchar();
                // >>>> Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px

                state.at<float>(0) = measurement.at<float>(0);
                state.at<float>(1) = measurement.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = measurement.at<float>(2);
                state.at<float>(5) = measurement.at<float>(3);

                // <<<< Initialization

                kf.statePost = state;
                found = true;
            } else {
                kf.correct(measurement); // Kalman Correction
                estimatedState = kf.correct(measurement);
                cout << "Kalman Estimate ([x,y,v_x,v_y,w,h]) : " << estimatedState.at<float>(0, 0) << " "
                     << estimatedState.at<float>(1, 0) << " " << estimatedState.at<float>(2, 0) << " "
                     << estimatedState.at<float>(3, 0) << " " << estimatedState.at<float>(4, 0) << " "
                     << estimatedState.at<float>(5, 0) << endl;
            }

            cout << "Measure matrix [z_x,z_y,z_w,z_h] : " << measurement.at<float>(0, 0) << " "
                 << measurement.at<float>(1, 0) << " " << measurement.at<float>(2, 0) << " "
                 << measurement.at<float>(3, 0) << endl;
        }
        imshow("CamShift Demo", display_image);
        imshow("Histogram", histimg);
        cv::waitKey(20);


        if (balls.size() == 0 ) {
            xc = estimatedState.at<float>(0);
            yc = estimatedState.at<float>(1);
        } else {
            xc = ballsBox[0].x + ballsBox[0].width / 2;
            yc = ballsBox[0].y + ballsBox[0].height / 2;
        }

        // if the pixel coordinates are out of bounds return default value
        if ((xc < 0 || xc > 640) || (yc < 0 || yc > 480)) {
            cout << "out of range" << endl;
            pt.z = 0.7;
        } else
            pt = ptcloud->at(xc, yc);


        //Centroid in image coordinates
        double image_coordinates[2];
        image_coordinates[0] = (xc - 320) / (531.15 / 640);    //(u-u0)/px
        image_coordinates[1] = (yc - 240) / (531.15 / 480);    //(v-v0)/py50
        double depth = double(pt.z);
        cout << "depth = " << float(depth) << endl;
        cout << "centroid (pixel) " << xc << " " << yc << endl;
        cout << "centroid (image) " << image_coordinates[0] << " " << image_coordinates[1] << endl;

        //writing to a msg
        data.detected_point_x.data = xc;
        data.detected_point_y.data = yc;
        data.detected_point_depth.data = (float) pt.z;

        pub.publish(data);
        cout << "Published Image data" << endl;
        ros::spinOnce();

    }
    return 0;
}

