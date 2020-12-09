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
#include "opencv/cv.hpp"
#include <camshift_tracker/image_data.h>
#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

// global variables
Mat image;
bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 47, vmax = 255, smin = 118; //
int iLastX = 0; int iLastY = 0;

//User draws box around object to track. This triggers CAMShift to start tracking
static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);
    }

    switch( event )
    {
        case EVENT_LBUTTONDOWN:
            origin = Point(x,y);
            selection = Rect(x,y,0,0);
            selectObject = true;
            break;
        case EVENT_LBUTTONUP:
            selectObject = false;
            if( selection.width > 0 && selection.height > 0 )
                trackObject = -1;   //Set up CAMShift properties in main() loop
            break;
    }
}

static void help()
{
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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

//void drawCircle( Mat img, Point center , int r,int g, int b, int thickness = 5, int lineType = 8 )
//{
//    circle( img, center, 4, Scalar( r, g, b ), thickness, lineType );
//}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud, bool &flag)
{
    pcl::fromROSMsg( *input, *ptCloud );
    flag = true;
}



int main( int argc, char** argv )
{
    cv::Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;

    // ros initialisations
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle nh;


    bool flag_ptcloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    ros::Publisher pub = nh.advertise<camshift_tracker::image_data>("/object_points", 10);
    camshift_tracker::image_data data;

    image_transport::ImageTransport img_trans(nh);
    image_transport::Subscriber sub_image = img_trans.subscribe("/camera/rgb/image_raw",1,imageCallback);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, boost::bind(pointCloudCallback, _1, boost::ref(ptcloud),boost::ref(flag_ptcloud)));

    sleep(1);
    ros::spinOnce();


    namedWindow( "Histogram", 0 );
    namedWindow( "CamShift Demo", 0 );
    setMouseCallback( "CamShift Demo", onMouse, 0 );

    namedWindow("Control", CV_WINDOW_AUTOSIZE);
    createTrackbar("Vmin", "Control", &vmin,255);
    createTrackbar("Vmax", "Control", &vmax, 255);
    createTrackbar("Smin", "Control", &smin,255);

    while(image.empty())
    {
        ros::spinOnce();
        cout << "empty image" << endl;
    }



    Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
    bool paused = false;

    double area;
    double processedImages = 0;
    bool startProcessingImage = false;
    double time, total_time = 10.0;

    while(ros::ok())
    {
        pcl::PointXYZRGB pt;

        if( !paused )
        {
            if( image.empty() )
                break;
        }

        if( !paused )
        {
            cvtColor(image, hsv, COLOR_BGR2HSV);

            if( trackObject )
            {
                int _vmin = vmin, _vmax = vmax;
                inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);



                cv::imshow("Mask", mask);

                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
                mixChannels(&hsv, 1, &hue, 1, ch, 1);

                if( trackObject < 0 )
                {
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
                    for( int i = 0; i < hsize; i++ )
                        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cvtColor(buf, buf, COLOR_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
                        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
                        rectangle( histimg, Point(i*binW,histimg.rows),
                                   Point((i+1)*binW,histimg.rows - val),
                                   Scalar(buf.at<Vec3b>(i)), -1, 8 );
                    }
                }

                // Perform CAMShift
                calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                backproj &= mask;

                RotatedRect trackBox = CamShift(backproj, trackWindow,
                                                TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));

                cout << "Object centre from cam shift " << trackBox.center.x << " " << trackBox.center.y << endl;
                cout << "Object area from cam shift " << trackBox.size.area() << endl;

                iLastX = trackBox.center.x;
                iLastY = trackBox.center.y;
                area = trackBox.size.area();
                cv::circle( image, Point(iLastX, iLastY), 4, Scalar( 0, 0, 255 ), 5, 8 );
                //drawCircle( image, Point(iLastX, iLastY),0,0,255 );

                if( trackWindow.area() <= 1 )
                {
                    int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                    trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                       trackWindow.x + r, trackWindow.y + r) &
                                  Rect(0, 0, cols, rows);
                    ROS_INFO("out of camera scope");
                }

                if( backprojMode )
                    cvtColor( backproj, image, COLOR_GRAY2BGR );
                ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );

            }
        }
        else if(trackObject < 0)
            paused = false;

        if( selectObject && selection.width > 0 && selection.height > 0 )
        {
            Mat roi(image, selection);
            bitwise_not(roi, roi);
        }



        cv::line(image,Point(300,240),Point(340,240),Scalar(0,0,0),2,8);
        cv::line(image,Point(320,220),Point(320,260),Scalar(0,0,0),2,8);

        imshow( "CamShift Demo", image );
        imshow( "Histogram", histimg );


        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch(c)
        {
            case 'b':
                backprojMode = !backprojMode;
                break;
            case 'c':
                trackObject = 0;
                histimg = Scalar::all(0);
                break;
            case 'h':
                showHist = !showHist;
                if( !showHist )
                    destroyWindow( "Histogram" );
                else
                    namedWindow( "Histogram", 1 );
                break;
            case 'p':
                paused = !paused;
                break;
            default:
                ;
        }

        //Centroid in image coordinates
        double m[2];
        m[0] = (iLastX-320)/(531.15/640);    //(u-u0)/px
        m[1] = (iLastY-240)/(531.15/480);    //(v-v0)/py50

        pt = ptcloud ->at(iLastX, iLastY);
        cout << "depth = " << float(pt.z) << endl;
        cout << "centroid (pixel) " << iLastX << " " << iLastY << endl;
        cout << "centroid (image) " << m[0] << " " << m[1] << endl;

        //writing to a msg
        data.detected_point_x.data = iLastX;
        data.detected_point_y.data = iLastY;
        data.detected_point_depth.data =(float)pt.z;
        data.area.data = area;


        pub.publish(data);
        cout <<"Published Image data"<<endl;
        ros::spinOnce();
    }
    return 0;
}

