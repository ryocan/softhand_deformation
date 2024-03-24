/*******************************************************
ProcCommon.cpp
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "imgProc/procCommon.h"

/**************************************************************************
 * @Function
 *  imgCb
 * 
 * @Details
 *    subscribe image from /kai_processed_image
 *    processed_image is an image generated considering depth threshold
**************************************************************************/
void procCommon::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    if(Flag == 0)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            imgSrc = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    else
    {
        imgSrc = cv::imread(pathInput);
    }
}

/**************************************************************************
 * @Function
 *  imgCbColor
 * 
 * @Details
 *    subscribe image from /camera/color/image_raw
 *    this image is subscribed from realsense
**************************************************************************/
void procCommon::imgCbColor(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    if(Flag == 0)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            imgColor = cv_ptr->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    else
    {
        imgColor = cv::imread("/home/umelab-pgi5g/softhand_data/imgColor.jpg");
    }
}


/**************************************************************************
 * @Function
 *  getParam
 * 
 * @Details
 *    get parameter from yaml
**************************************************************************/
bool procCommon::getParam()
{
    // On-line or Off-line
    nh.getParam("image_proc/Flag", Flag);

    // Calibration Parameter
    nh.getParam("image_proc/Bending_Angle_Left", bendAngleL);
    nh.getParam("image_proc/Bending_Angle_Right", bendAngleR);  

    // number of segments
    nh.getParam("image_proc/Number_of_segment", numSegments);  

    // threshold for deque
    nh.getParam("image_proc/dequeTh", dequeTh);  

    // HSV HAND
    nh.getParam("image_proc/H_MIN_HAND", H_MIN_HAND);
    nh.getParam("image_proc/H_MAX_HAND", H_MAX_HAND);
    nh.getParam("image_proc/S_MIN_HAND", S_MIN_HAND);
    nh.getParam("image_proc/S_MAX_HAND", S_MAX_HAND);
    nh.getParam("image_proc/V_MIN_HAND", V_MIN_HAND);
    nh.getParam("image_proc/V_MAX_HAND", V_MAX_HAND);

    // HSV OBJ
    nh.getParam("image_proc/H_MIN_OBJ", H_MIN_OBJ);
    nh.getParam("image_proc/H_MAX_OBJ", H_MAX_OBJ);
    nh.getParam("image_proc/S_MIN_OBJ", S_MIN_OBJ);
    nh.getParam("image_proc/S_MAX_OBJ", S_MAX_OBJ);
    nh.getParam("image_proc/V_MIN_OBJ", V_MIN_OBJ);
    nh.getParam("image_proc/V_MAX_OBJ", V_MAX_OBJ);

    nh.getParam("image_proc/PATH_INPUT", pathInput);
    nh.getParam("image_proc/PATH_INPUT2", pathInput2);
    nh.getParam("image_proc/PATH_OUTPUT", pathOutput);

    return true;
}

/**************************************************************************
 * @Function
 *  cropImage
 * 
 * @Details
 *    crop the image based on the image size 
**************************************************************************/
void procCommon::cropImage()
{
    // image size
    int img_cols = imgSrc.cols;
    int img_rows = imgSrc.rows;

    // crop the image
    imgSrcL = cv::Mat(imgSrc, cv::Rect(0, 0, img_cols/2, img_rows));
    imgSrcR = cv::Mat(imgSrc, cv::Rect(img_cols/2, 0, img_cols/2, img_rows));
    // imshow("imgSrcL", imgSrcL);
    // imshow("imgSrcR", imgSrcR);

    // for IROS2024 validation on one finger
    // imgSrcL = cv::Mat(imgSrc, cv::Rect(0, 0, 3 * img_cols/5, img_rows));
    // imgSrcR = cv::Mat(imgSrc, cv::Rect(3 * img_cols/5, 0, 2*img_cols/5, img_rows));


    // clone
    imgOutputL = imgSrcL.clone();
    imgOutputR = imgSrcR.clone();
}

/**************************************************************************
 * @Function
 *  maskImage
 * 
 * @Details
 *    create mask image from cropped image
**************************************************************************/
void procCommon::maskImage()
{
    // clone image
    imgMaskHandL = cv::Mat::zeros(imgSrcL.size(), imgSrcL.type());
    imgMaskHandR = cv::Mat::zeros(imgSrcL.size(), imgSrcR.type());
    imgMaskObjL = cv::Mat::zeros(imgSrcR.size(), imgSrcL.type());
    imgMaskObjR = cv::Mat::zeros(imgSrcR.size(), imgSrcR.type());

    // convert to HSV image
    cv::Mat img_hsv_l, img_hsv_r;
    cv::cvtColor(imgSrcL, img_hsv_l, cv::COLOR_BGR2HSV);
    cv::cvtColor(imgSrcR, img_hsv_r, cv::COLOR_BGR2HSV);

    // set HSV range
    cv::Scalar lower_hand(H_MIN_HAND, S_MIN_HAND, V_MIN_HAND);
    cv::Scalar upper_hand(H_MAX_HAND, S_MAX_HAND, V_MAX_HAND);
    cv::Scalar lower_obj(H_MIN_OBJ, S_MIN_OBJ, V_MIN_OBJ);
    cv::Scalar upper_obj(H_MAX_OBJ, S_MAX_OBJ, V_MAX_OBJ);

    // extract region based on HSV parameter
    inRange(img_hsv_l, lower_hand, upper_hand, imgMaskHandL);
    inRange(img_hsv_r, lower_hand, upper_hand, imgMaskHandR);
    inRange(img_hsv_l, lower_obj, upper_obj, imgMaskObjL);
    inRange(img_hsv_r, lower_obj, upper_obj, imgMaskObjR);

    // // // molfo
    // imshow("Before imgMaskHandR", imgMaskHandR);
    cv::morphologyEx(imgMaskHandL, imgMaskHandL, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);


    // imshow("imgMaskHandL", imgMaskHandL);
    // imshow("imgMaskHandR", imgMaskHandR);
    // imshow("imgMaskObjL", imgMaskObjL);
    // imshow("imgMaskObjR", imgMaskObjR);
}

/**************************************************************************
 * @Function
 *  getLargestContour
 * 
 * @Details
 *    obtain contour of the largest area in the image
**************************************************************************/
std::vector<cv::Point> procCommon::getLargestContour(cv::Mat img_mask)
{
    // findContours from img_mask
    std::vector<std::vector<cv::Point>> contours; 
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // detect based on area info
    double area = 0.;
    std::vector<cv::Point> contours_output;

    // change contours calculation method
    double area_prev = 0.;
    for (int i = 0; i < contours.size(); i++)
    {
        area = contourArea(contours[i]);
        if (area > area_prev && area > 50)
        {
            contours_output.clear();
            contours_output = contours[i];
            area_prev = area;
        }
    }

    return contours_output;
}

/**************************************************************************
 * @Function
 *  obtainContour
 * 
 * @Details
 *    obtain contour with using getLargestContour
**************************************************************************/
void procCommon::obtainContour()
{
    // obtain each contour
    contourHandL = getLargestContour(imgMaskHandL);
    contourHandR = getLargestContour(imgMaskHandR);
    contourObjL = getLargestContour(imgMaskObjL);
    contourObjR = getLargestContour(imgMaskObjR);
}

/**************************************************************************
 * @Function
 *  findBaseAndTip
 * 
 * @Details
 *    find base point and tip point of the hand
**************************************************************************/
void procCommon::findBaseAndTip()
{
    // left -----------------------------------------------
    int num_base_l = 0, num_tip_l = 0;
    int num_base_l_update = 0, num_tip_l_update = 0;
    double x_base_l_prev = 0., x_tip_l_prev = 0.;
    double y_base_l_prev = imgSrcL.rows, y_tip_l_prev = 0.;

    // scan: first time
    for(int i = 0; i < contourHandL.size(); i++)
    {
        if(contourHandL[i].y < y_base_l_prev)
        {
            y_base_l_prev = contourHandL[i].y;
            num_base_l = 0;
        }
        if(contourHandL[i].y > y_tip_l_prev)
        {
            y_tip_l_prev = contourHandL[i].y;
            num_tip_l = i;
        }
    }

    // scan: second time. cosider noize influence
    for (int i = 0; i < contourHandL.size(); i++)
    {
        if(contourHandL[i].y < contourHandL[num_base_l].y + 5)
        {
            if(contourHandL[i].x > x_base_l_prev)
            {
                x_base_l_prev = contourHandL[i].x;
                num_base_l_update = i;
            }
        }
        if(contourHandL[num_tip_l].y - 1 < contourHandL[i].y )
        {
            if(contourHandL[i].x > x_tip_l_prev)
            {
                x_tip_l_prev = contourHandL[i].x;
                num_tip_l_update = i;
            }
        }
    }

    // update point
    pointBaseL = contourHandL[num_base_l_update];
    pointTipL = contourHandL[num_tip_l_update];
    
    // right -------------------------------------------------------
    int num_base_r = 0, num_tip_r = 0;
    int num_base_r_update = 0, num_tip_r_update = 0;
    double x_base_r_prev = imgSrcR.cols, x_tip_r_prev = imgSrcR.cols;
    double y_base_r_prev = imgSrcR.rows, y_tip_r_prev = 0.;

    // scan: first time
    for(int i = 0; i < contourHandR.size(); i++)
    {
        if(contourHandR[i].y < y_base_r_prev)
        {
            y_base_r_prev = contourHandR[i].y;
            num_base_r = 0;
        }
        if(contourHandR[i].y > y_tip_r_prev)
        {
            y_tip_r_prev = contourHandR[i].y;
            num_tip_r = i;
        }
    }

    // scan: second time. cosider noize influence
    for (int i = 0; i < contourHandR.size(); i++)
    {
        if(contourHandR[i].y < contourHandR[num_base_r].y + 5)
        {
            if(contourHandR[i].x < x_base_r_prev)
            {
                x_base_r_prev = contourHandR[i].x;
                num_base_r_update = i;
            }
        }
        if(contourHandR[num_tip_r].y - 1 < contourHandR[i].y )     // 30
        {
            if(contourHandR[i].x < x_tip_r_prev)
            {
                x_tip_r_prev = contourHandR[i].x;
                num_tip_r_update = i;
            }
        }
    }

    // update point
    pointBaseR = contourHandR[num_base_r_update];
    pointTipR = contourHandR[num_tip_r_update];

    // deque
    if(dequeBaseL.size() == dequeTh)
    {
        // pop front
        dequeBaseL.pop_front();
        dequeTipL.pop_front();
        dequeBaseR.pop_front();
        dequeTipR.pop_front();

        // push back
        dequeBaseL.push_back(pointBaseL);
        dequeTipL.push_back(pointTipL);
        dequeBaseR.push_back(pointBaseR);
        dequeTipR.push_back(pointTipR);
    }
    else
    {
        // push back
        dequeBaseL.push_back(pointBaseL);
        dequeTipL.push_back(pointTipL);
        dequeBaseR.push_back(pointBaseR);
        dequeTipR.push_back(pointTipR);
    }

    // deque mean
    double baseLx, baseLy;
    double tipLx, tipLy; 
    double baseRx, baseRy;
    double tipRx, tipRy; 
    for(int i = 0; i < dequeBaseL.size(); i++)
    {
        baseLx += dequeBaseL[i].x;
        baseLy += dequeBaseL[i].y;
        tipLx += dequeTipL[i].x;
        tipLy += dequeTipL[i].y;
        baseRx += dequeBaseR[i].x;
        baseRy += dequeBaseR[i].y;
        tipRx += dequeTipR[i].x;
        tipRy += dequeTipR[i].y;
    }

    pointBaseL.x = baseLx / dequeTh;
    pointBaseL.y = baseLy / dequeTh;
    pointTipL.x = tipLx / dequeTh;
    pointTipL.y = tipLy / dequeTh;
    pointBaseR.x = baseRx / dequeTh;
    pointBaseR.y = baseRy / dequeTh;
    pointTipR.x = tipRx / dequeTh;
    pointTipR.y = tipRy / dequeTh;

    
    // show result
    cv::circle(imgOutputL, pointBaseL,  8, cv::Scalar(255,   255,   255), -1);
    cv::circle(imgOutputL, pointBaseL,  6, cv::Scalar(255,   0,   0), -1);
    cv::circle(imgOutputL, pointTipL,  8, cv::Scalar(255,   255,   255), -1);
    cv::circle(imgOutputL, pointTipL,  6, cv::Scalar(255,   0,   0), -1);

    cv::circle(imgOutputR, pointBaseR,  8, cv::Scalar(255,   255,   255), -1);
    cv::circle(imgOutputR, pointBaseR,  6, cv::Scalar(255,   0,   0), -1);
    cv::circle(imgOutputR, pointTipR,  8, cv::Scalar(255,   255,   255), -1);
    cv::circle(imgOutputR, pointTipR,  6, cv::Scalar(255,   0,   0), -1);
    
}
