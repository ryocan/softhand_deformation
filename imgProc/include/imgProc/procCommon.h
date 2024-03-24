/*******************************************************
ProcCommon.h
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// standard libraries
#include <iostream>
#include <math.h>

// about image processing
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// about ros
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>

//-----------------------------------------------------
// CLASS
//-----------------------------------------------------
class procCommon
{

    private:
        /*--- ros setup ---*/
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_sub_color;
        image_transport::Subscriber image_sub;
        image_transport::Publisher image_pub;

        /*--- online or offline ---*/
        int Flag;   // 0: online, 1: offline

        /*--- deque ---*/
        std::deque<cv::Point2f> dequeBaseL;
        std::deque<cv::Point2f> dequeTipL;
        std::deque<cv::Point2f> dequeBaseR;
        std::deque<cv::Point2f> dequeTipR;
                       
        /*--- function ---*/
        // image callback. get image
        void imgCb(const sensor_msgs::ImageConstPtr& msg);

        // image callback. get image
        void imgCbColor(const sensor_msgs::ImageConstPtr& msg);

        // get parameter from yaml
        bool getParam();

       

    public:
        procCommon()
        :it(nh)
        {
            getParam();
            image_sub = it.subscribe("/kai_processed_image", 1, &procCommon::imgCb, this);
            image_sub_color = it.subscribe("/camera/color/image_raw", 1, &procCommon::imgCbColor, this);
        }

        ~procCommon()
        {
        }

        /*--- string ---*/
        std::string pathInput, pathInput2, pathOutput;

        /*--- variables for the mask images  ---*/
        int H_MIN_HAND, H_MAX_HAND, S_MIN_HAND, S_MAX_HAND, V_MIN_HAND, V_MAX_HAND; 
        int H_MIN_OBJ, H_MAX_OBJ, S_MIN_OBJ, S_MAX_OBJ, V_MIN_OBJ, V_MAX_OBJ; 
        
        int dequeTh;

        /*--- images ---*/
        cv::Mat imgSrc;
        cv::Mat imgSrcL, imgSrcR;  // left and right finger
        cv::Mat imgColor;
        cv::Mat imgMaskHandL, imgMaskHandR;
        cv::Mat imgMaskObjL, imgMaskObjR; 
        cv::Mat imgOutputL, imgOutputR;
        

        /*--- soft hand information ---*/
        int numSegments;    // number of segment
        double bendAngleL, bendAngleR;   // bending angle [rad]

        /*--- points ---*/
        cv::Point2f pointBaseL, pointBaseR;     // base point of the finger
        cv::Point2f pointTipL, pointTipR;       // tip point of the finger
        cv::Point2f pointBaseValL, pointBaseValR; // for validation

        /*--- contours ---*/
        std::vector<cv::Point> contourHandL, contourHandR;
        std::vector<cv::Point> contourObjL, contourObjR;

        /*--- function ---*/
        // crop the image based on the image size
        void cropImage();

        // create mask image from cropImage()
        void maskImage();

        // preprocessing for obtainContour
        std::vector<cv::Point> getLargestContour(cv::Mat img_mask);

        // obtain contours from the maskImage()
        void obtainContour();

        // find base and tip in the hand
        void findBaseAndTip();

};