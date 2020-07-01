#include <iostream>
#include <lib.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
using namespace cv;
using namespace std;

cv::Mat undistort(cv::Mat &img, int camera=0) {
    cv::Mat ret;
    int fx, fy, cx, cy, k1, k2, k3, k4;
    if (camera == 0) {
    	fx = 505.8652709936199 ; cx = 320.2859180341229; fy = 503.73693683318965; cy = 238.25068717115033;
    	k1 = 0.49461074454866766; k2 = 0.013263509533947193; k3 = -0.9780146810775383; k4 = 2.680813230288828;
    }
    if (camera == 1) {
        	fx = 801.4746867140315 ; cx = 619.8317196836534; fy = 802.4712648758165; cy = 354.26222652943864;
        	k1 = -0.13180633556637164; k2 = -0.02478291422213951; k3 = 0.021812495319081183; k4 = -0.007840254293128806;
    }
    cv::Mat K = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat D = (cv::Mat1d(1, 4) << k1, k2, k3, k4);

    cv::fisheye::undistortImage(img, ret, K, D, 1.0 * K);
    return ret;
}

Point transform_gaze(Point &pt, cv::Mat &R, cv::Mat &T) {
	//Point pt2;
	cv::Mat_<double> src(3/*rows*/,1 /* cols */);

    src(0,0)=pt.x;
    src(1,0)=pt.y;
    src(2,0)=1.0;

    int fx, fy, cx, cy, k1, k2, k3, k4;
    fx = 505.8652709936199 ; cx = 320.2859180341229; fy = 503.73693683318965; cy = 238.25068717115033;
    k1 = 0.49461074454866766; k2 = 0.013263509533947193; k3 = -0.9780146810775383; k4 = 2.680813230288828;
    cv::Mat K0 = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    fx = 801.4746867140315 ; cx = 619.8317196836534; fy = 802.4712648758165; cy = 354.26222652943864;
    k1 = -0.13180633556637164; k2 = -0.02478291422213951; k3 = 0.021812495319081183; k4 = -0.007840254293128806;
    cv::Mat K1 = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);


	cv::Mat_<double> dst = K1*R*K0.inv()*src+K1*R*K0.inv()*T;
	return Point(dst(0,0),dst(1,0));
}

int main(int argc, char** argv)
{
//dummy();
    Mat image0, image1;
    std::string cam0_path = "/work/CV/summer2020/new/eyetracker/cam_0/070000000.png";
    std::string cam1_path = "/work/CV/summer2020/new/eyetracker/cam_1/070000000.png";
    //image = imread(argv[1], IMREAD_COLOR); // Read the file

    image0 = imread(cam0_path, IMREAD_COLOR);
    image1 = imread(cam1_path, IMREAD_COLOR);

    if( image0.empty() || image1.empty() ) // Check for invalid input
    {
    	cout << "Could not open or find the image" << std::endl ;
    	return -1;
    }
    namedWindow( "distorted image", WINDOW_AUTOSIZE ); // Create a window for display.
    imshow( "distorted image", image0 ); // Show our image inside it.
    Mat output0 = image0; //undistort(image0,0);
    Point pt(250,250);
    cv::circle(output0, pt, 5, Scalar(0,255,0), -1, 8, 0);
    namedWindow( "undistorted image", WINDOW_AUTOSIZE ); // Create a window for display.
    imshow( "undistorted image", output0); // Show our image inside it.
    waitKey(0);

    //tranforming gaze
    cv::Mat R = (cv::Mat1d(3, 3) << 0.9994566023782989, 0.01851288622649493, -0.027272202074431786,// 0.019393923070310046,
    		-0.018026869041005898, 0.9996761766481317, 0.017960340607817062, //-0.018166250744346878,
			0.02759586844080487, -0.017458948586190227, 0.9994666843668475 );//-0.009808854206112775,
			//0.0, 0.0, 0.0, 1.0);
    cv::Mat T = (cv::Mat1d(3,1) << 0.019393923070310046, -0.018166250744346878, -0.009808854206112775);
    //Point pt2;
	//pt2 = transform_gaze(pt, T);
    std:: cout << transform_gaze(pt, R, T);
    namedWindow( "distorted image", WINDOW_AUTOSIZE ); // Create a window for display.
    imshow( "distorted image", image1 ); // Show our image inside it.
    Mat output1 = undistort(image1,1);
    cv::circle(output1, transform_gaze(pt, R, T), 5, Scalar(0,255,0), -1, 8, 0);
    namedWindow( "undistorted image", WINDOW_AUTOSIZE ); // Create a window for display.
    imshow( "undistorted image", output1); // Show our image inside it.
    waitKey(0); // Wait for a keystroke in the window


    return 0;
}
