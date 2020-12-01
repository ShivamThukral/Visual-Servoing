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
RNG rng(12345);


//User draws box around object to track. This triggers CAMShift to start tracking
static void onMouse(int event, int x, int y, int, void *) {
    if (selectObject) {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch (event) {
        case EVENT_LBUTTONDOWN:
            origin = Point(x, y);
            selection = Rect(x, y, 0, 0);
            selectObject = true;
            break;
        case EVENT_LBUTTONUP:
            selectObject = false;
            if (selection.width > 0 && selection.height > 0)
                trackObject = -1;   //Set up CAMShift properties in main() loop
            break;
    }
}

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
    vector <vector<cv::Point>> contours;
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

    namedWindow("Histogram", 0);
    namedWindow("CamShift Demo", 0);
    namedWindow("Control", CV_WINDOW_AUTOSIZE);
    setMouseCallback("CamShift Demo", onMouse, 0);
    createTrackbar("Vmin", "Control", &vmin, 255);
    createTrackbar("Vmax", "Control", &vmax, 255);
    createTrackbar("Smin", "Control", &smin, 255);

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
            cv::circle(display_image, center, 5, CV_RGB(0, 255, 0), -1);
            cv::rectangle(display_image, predRect, CV_RGB(0, 255, 0), 2);
        }

        if (!paused) {

            cvtColor(image, hsv, COLOR_BGR2HSV);     // convert BGR to HSV

            if (trackObject) {
                //int _vmin = vmin, _vmax = vmax;
                inRange(hsv, Scalar(0, smin, MIN(vmin, vmax)), Scalar(180, 256, MAX(vmin, vmax)),
                        mask);        // filter out blue color as mask
                cv::imshow("Mask", mask);
                cv::waitKey(20);


                cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
                mixChannels(&hsv, 1, &hue, 1, ch, 1);   //separate out hue channel

                if (trackObject < 0) {
                    // Object has been selected by user, set up CAMShift search properties once
                    Mat roi(hue, selection), maskroi(mask, selection);
                    calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                    normalize(hist, hist, 0, 255, NORM_MINMAX);

                    trackWindow = selection;
                    trackObject = 1; // Don't set up again, unless user selects new ROI
                    startProcessingImage = true;
                    time = ros::Time::now().toSec();

                    histimg = Scalar::all(0);
                    int binW = histimg.cols / hsize;
                    Mat buf(1, hsize, CV_8UC3);
                    for (int i = 0; i < hsize; i++)
                        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i * 180. / hsize), 255, 255);
                    cvtColor(buf, buf, COLOR_HSV2BGR);

                    for (int i = 0; i < hsize; i++) {
                        int val = saturate_cast<int>(hist.at<float>(i) * histimg.rows / 255);
                        rectangle(histimg, Point(i * binW, histimg.rows),
                                  Point((i + 1) * binW, histimg.rows - val),
                                  Scalar(buf.at<Vec3b>(i)), -1, 8);
                    }
                }

                if (contours.size() != 0) {

                    // Perform CAMShift
                    calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                    backproj &= mask;

                    RotatedRect trackBox = CamShift(backproj, trackWindow,
                                                    TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));

                    iLastX = trackBox.center.x;
                    iLastY = trackBox.center.y;
                    cv::circle(display_image, Point(iLastX, iLastY), 4, Scalar(0, 0, 255), 5, 8);

                    if (trackWindow.area() <= 1) {
                        int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
                        trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                           trackWindow.x + r, trackWindow.y + r) &
                                      Rect(0, 0, cols, rows);
                        ROS_INFO("out of camera scope");
                    }

                    if (backprojMode)
                        cvtColor(backproj, image, COLOR_GRAY2BGR);
                    ellipse(display_image, trackBox, Scalar(0, 0, 255), 3, CV_AA);
                }
            }
        } else if (trackObject < 0)
            paused = false;

        if (selectObject && selection.width > 0 && selection.height > 0) {
            Mat roi(image, selection);
            bitwise_not(roi, roi);
        }

        // >>>>> Kalman Update
        if (contours.size() == 0) {

            notFoundCount++;
            //cout << "notFoundCount:" << notFoundCount << endl;
            if (notFoundCount >= 200) {
                ROS_ERROR("Object estimate is lost after 100 attempts.....");
                found = false;
            }
            ROS_WARN("Object Not in Frame .....");

        } else {
            notFoundCount = 0;

            measurement.at<float>(0) = iLastX;
            measurement.at<float>(1) = iLastY;
            measurement.at<float>(2) = selection.width;
            measurement.at<float>(3) = selection.height;

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


        if (contours.size() == 0 && trackWindow.area() <= 1) {
            xc = estimatedState.at<float>(0);
            yc = estimatedState.at<float>(1);
        } else {
            xc = iLastX;
            yc = iLastY;
        }

        char c = (char) waitKey(10);
        if (c == 27)
            break;
        switch (c) {
            case 'b':
                backprojMode = !backprojMode;
                break;
            case 'c':
                trackObject = 0;
                histimg = Scalar::all(0);
                break;
            case 'h':
                showHist = !showHist;
                if (!showHist)
                    destroyWindow("Histogram");
                else
                    namedWindow("Histogram", 1);
                break;
            case 'p':
                paused = !paused;
                break;
            default:;
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

