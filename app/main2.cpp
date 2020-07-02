#include <iostream>
#include <lib.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

#include <glob.h>
#include <string>
#include <vector>
using std::vector;
using namespace cv;
using namespace std;

vector<string> globVector(const string& pattern){
    glob_t glob_result;
    glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
    vector<string> files;
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        files.push_back(string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return files;
}

cv::Point transform_gaze1(Point &pt, cv::Mat &R, cv::Mat &T) {
	//Point pt2;
	cv::Mat_<double> src(3/*rows*/,1 /* cols */);
	float Z = 1;
    src(0,0)=pt.x;
    src(1,0)=pt.y;
    src(2,0)=Z;

    float fx, fy, cx, cy, k1, k2, k3, k4;
    fx = 505.8652709936199 ; cx = 320.2859180341229; fy = 503.73693683318965; cy = 238.25068717115033;
    k1 = 0.49461074454866766; k2 = 0.013263509533947193; k3 = -0.9780146810775383; k4 = 2.680813230288828;
    cv::Mat K0 = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    fx = 801.4746867140315 ; cx = 619.8317196836534; fy = 802.4712648758165; cy = 354.26222652943864;
    k1 = -0.13180633556637164; k2 = -0.02478291422213951; k3 = 0.021812495319081183; k4 = -0.007840254293128806;
    cv::Mat K1 = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);


	cv::Mat_<double> dst = K1*R*K0.inv()*src+K1*R*K0.inv()*T;
	cout << "gaze point1 is :" << dst(0,0)<< ", "<< dst(1,0)<<endl;
	return Point(dst(0,0),dst(1,0));
}

cv::Point transform_gaze(int px, int py, cv::Mat &Rvec, cv::Mat &T, Mat &K0, Mat &D0, Mat &K1, Mat &D1) {
	cv::Mat dst;
    cv::Mat src = cv::Mat(1, 1, CV_32FC2, cv::Scalar(px, py));
    // eq model
    cv::fisheye::undistortPoints(src, dst, K0, D0);

    float Z = 1;
    auto p = cv::Point3f(dst.at<cv::Vec2f>(0, 0)[0], dst.at<cv::Vec2f>(0, 0)[1], 1) * Z;
    std::vector<cv::Point3f> p_vec = {p};
                                                                                                  
    cv::Mat pix;

    //rt model
    cv::projectPoints(p_vec, Rvec, T, K1, D1, pix);
    //cv::fisheye::projectPoints(p_vec, pix, Rvec, T, e_K, e_D);
                                                                                                  
    uint32_t gaze_match_px = pix.at<cv::Vec2f>(0, 0)[0]; 
    uint32_t gaze_match_py = pix.at<cv::Vec2f>(0, 0)[1];
    cout << "\ngaze point : "<< gaze_match_px<<", " <<gaze_match_py<<endl;
    return cv::Point(gaze_match_px, gaze_match_py);
	//return cv::Point(150,150);
}    

class Camera {
    public:
        std::string camera_model, distortion_model;
 
        cv::Mat K, D;
    public:
        Camera(std::string dist_model, float fx, float fy, float cx, float cy, float k1, float k2, float k3, float k4) {
            this->distortion_model = dist_model;
            this->K = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            this->D = (cv::Mat1d(1, 4) << k1, k2, k3, k4);
        }
        
        cv::Mat undistort(cv::Mat &img) {
            cv::Mat ret;
            /*int fx, fy, cx, cy, k1, k2, k3, k4;
            if (camera == 0) {
            	fx = 505.8652709936199 ; cx = 320.2859180341229; fy = 503.73693683318965; cy = 238.25068717115033;
            	k1 = 0.49461074454866766; k2 = 0.013263509533947193; k3 = -0.9780146810775383; k4 = 2.680813230288828;
            }
            if (camera == 1) {
                	fx = 801.4746867140315 ; cx = 619.8317196836534; fy = 802.4712648758165; cy = 354.26222652943864;
                	k1 = -0.13180633556637164; k2 = -0.02478291422213951; k3 = 0.021812495319081183; k4 = -0.007840254293128806;
            }*/
            
            if (this->distortion_model == "radtan") {
                cv::undistort(img, ret, this->K, this->D, 1.0*this->K);
            }
            else if (this->distortion_model == "equidistant") {
            cv::fisheye::undistortImage(img, ret, this->K, this->D, 1.0 * this->K);
            } 
            else {
                std::cout << "Unknown distortion model!" << this->distortion_model << std::endl;
            }
            return ret;
        }
};


int main(int argc, char** argv)
{
    // read images from both cameras
    Mat image0, image1;

    std::string cam0_path = "/work/CV/summer2020/new/eyetracker/cam_0/0210000000.png";
    std::string cam1_path = "/work/CV/summer2020/new/eyetracker/cam_1/0210000000.png";
    
    Camera cam0_dvs("radtan", 506.4692274538, 504.1712838890729, 325.79676532421803, 238.70837103100686, 0.13140489691973775, -0.17447930179658452, 0.00044504317597171593,
    0.004727272184495526);
    Camera cam1_rgb("equidistant", 801.6947924812483, 802.6485823902558, 620.2055872431376, 354.3864350612314, -0.13229634176002464, -0.023644289230288832, 0.019704866760515385,
    -0.0065800357834113);
    image0 = imread(cam0_path, IMREAD_COLOR);
    image1 = imread(cam1_path, IMREAD_COLOR);

    cout << "read image";
    if( image0.empty() || image1.empty() ) // Check for invalid input
    {
    	cout << "Could not open or find the image" << std::endl ;
    	return -1;
    }

    Mat output0 = image0; //undistort(image0,0);
    int px = 450; int py = 150;
    //const Point pt(px,py);
    //cv::circle(output0, pt, 5, Scalar(0,255,0), -1, 8, 0);
    cv::circle(output0, Point(px, py), 5, Scalar(0,0, 255), -1, 8, 0);
    //imshow( "undistorted image0", output0); // Show our image inside it.
    //waitKey(0);

    //tranformation from cam1 to cam0
    cv::Mat R = (cv::Mat1d(3, 3) << 0.9994566023782989, 0.01851288622649493, -0.027272202074431786,// 0.019393923070310046,
    		-0.018026869041005898, 0.9996761766481317, 0.017960340607817062, //-0.018166250744346878,
			0.02759586844080487, -0.017458948586190227, 0.9994666843668475 );//-0.009808854206112775,
			//0.0, 0.0, 0.0, 1.0);
    cv::Mat T = (cv::Mat1d(3,1) << 0.019393923070310046, -0.018166250744346878, -0.009808854206112775);
    cv::Mat Rvec;
    cv::Rodrigues(R, Rvec);
   
    //Point pt2;
    Point pt(450,150);
	cv::Point gaze_match1 = transform_gaze1(pt, R, T);
    cv:: Point gaze_match = transform_gaze(px, py, Rvec, T, cam0_dvs.K, cam0_dvs.D, cam1_rgb.K, cam1_rgb.D);
    //imshow( "distorted image", image1 ); // Show our image inside it.
    //Mat output1 = undistort(image1,1);
    cv:: Mat output2 = cam1_rgb.undistort(image1);
    cv::circle(output2, gaze_match1, 5, Scalar(0,0, 255), -1, 8, 0);
    //cout<<"image0 size is "<< output0.size()<<endl;
    cv::resize(output2, output2, output0.size());
    //cout<<"image1 size is "<< output2.size()<<endl;
    hconcat(output0,output2, output2);
    //namedWindow( "undistorted image", WINDOW_AUTOSIZE ); // Create a window for display.
    //imshow( "undistorted image", output2); // Show our image inside it.
    
    cv:: Mat output1 = cam1_rgb.undistort(image1);
    cv::circle(output1, gaze_match, 5, Scalar(0,0, 255), -1, 8, 0);
    cout<<"image0 size is "<< output0.size()<<endl;
    cv::resize(output1, output1, output0.size());
    cout<<"image1 size is "<< output1.size()<<endl;
    hconcat(output2,output1, output1);
    namedWindow( "undistorted image1", WINDOW_AUTOSIZE ); // Create a window for display.
    imshow( "dvs|custom function|visualizer", output1); // Show our image inside it.
    imwrite("/work/CV/summer2020/new/eyetracker/0210000000_1.png", output1);
    waitKey(0); // Wait for a keystroke in the window


    return 0;
}
