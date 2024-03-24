//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "imgProc/setup.h"

/**************************************************************************
 * @Function
 *      calcBendAngle
 * 
 * @Details
 *      calculate bending angle
**************************************************************************/
void setup::calcBendAngle()
{
    // right
    double angleRight = atan2((pointTipR.x - pointBaseR.x),(pointTipR.y - pointBaseR.y));
    cv::line(imgOutputR, pointBaseR, pointTipR, cv::Scalar(255, 255, 255), 3, 8);
    cv::line(imgOutputR, pointBaseR, pointTipR, cv::Scalar(30, 255, 30), 2, 8);
    cv::line(imgOutputR, pointBaseR, cv::Point(pointBaseR.x, pointTipR.y), cv::Scalar(255, 255, 255), 3, 8);
    cv::line(imgOutputR, pointBaseR, cv::Point(pointBaseR.x, pointTipR.y), cv::Scalar(30, 255, 30), 2, 8);

    // left
    double angleLeft = atan2((pointTipL.x - pointBaseL.x),(pointTipL.y - pointBaseL.y));
    cv::line(imgOutputL, pointBaseL, pointTipL, cv::Scalar(255, 255, 255), 3, 8);
    cv::line(imgOutputL, pointBaseL, pointTipL, cv::Scalar(30, 255, 30), 2, 8);
    cv::line(imgOutputL, pointBaseL, cv::Point(pointBaseL.x, pointTipL.y), cv::Scalar(255, 255, 255), 3, 8);
    cv::line(imgOutputL, pointBaseL, cv::Point(pointBaseL.x, pointTipL.y), cv::Scalar(30, 255, 30), 2, 8);

    // // for IROS2024 paper
    // cv::Mat imgOutputColorL = cv::Mat(imgColor, cv::Rect(0, 0, imgColor.cols/2, imgColor.rows));
    // cv::imshow("imgOutputColorLBefore", imgOutputColorL);
    // cv::circle(imgOutputColorL, pointBaseL,  8, cv::Scalar(255,   255,   255), -1);
    // cv::circle(imgOutputColorL, pointBaseL,  6, cv::Scalar(255,   0,   0), -1);
    // cv::circle(imgOutputColorL, pointTipL,  8, cv::Scalar(255,   255,   255), -1);
    // cv::circle(imgOutputColorL, pointTipL,  6, cv::Scalar(255,   0,   0), -1);
    // cv::line(imgOutputColorL, pointBaseL, pointTipL, cv::Scalar(255, 255, 255), 3, 8);
    // cv::line(imgOutputColorL, pointBaseL, pointTipL, cv::Scalar(30, 255, 30), 2, 8);
    // cv::line(imgOutputColorL, pointBaseL, cv::Point(pointBaseL.x, pointTipL.y), cv::Scalar(255, 255, 255), 3, 8);
    // cv::line(imgOutputColorL, pointBaseL, cv::Point(pointBaseL.x, pointTipL.y), cv::Scalar(30, 255, 30), 2, 8);
    // std::vector<std::vector<cv::Point>> contours;
    // contours.push_back(contourHandL);
    // cv::drawContours(imgOutputColorL, contours, 0, cv::Scalar(255, 255, 255), 6); // Blue color (BGR format)
    // cv::drawContours(imgOutputColorL, contours, 0, cv::Scalar(255, 0, 0), 4); // Blue color (BGR format)
    // cv::imshow("imgOutputColorL", imgOutputColorL);

    // show the result--------------------------------------------
    // right
    imgOutputAngleR = imgOutputR.clone();
    cv::putText(imgOutputAngleR, "rad: " + std::to_string(angleRight), 
                    cv::Point(imgOutputAngleR.cols - 150, imgOutputAngleR.rows - 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(imgOutputAngleR, "deg: " + std::to_string(angleRight * 180 / 3.14), 
        cv::Point(imgOutputAngleR.cols - 150, imgOutputAngleR.rows - 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);         
    cv::imshow("imgOutputAngleR", imgOutputAngleR);


    // left
    imgOutputAngleL = imgOutputL.clone();
    cv::putText(imgOutputAngleL, "rad: " + std::to_string(angleLeft), 
                    cv::Point(imgOutputAngleL.cols - 150, imgOutputAngleL.rows - 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(imgOutputAngleL, "deg: " + std::to_string(angleLeft * 180 / 3.14), 
        cv::Point(imgOutputAngleL.cols - 150, imgOutputAngleL.rows - 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);         
    cv::imshow("imgOutputAngleL", imgOutputAngleL);
}

/**************************************************************************
 * @Function
 *      setupExec
 * 
 * @Details
 *      execute program for setup.cpp
**************************************************************************/
bool setup::setupExec()
{
    if (!imgSrc.empty())
    {
        // crop the image based on the image size
        cropImage();

        // create mask image
        maskImage();

        // obtain contours from the mask image 
        obtainContour();

        // find base and tip in the hand
        findBaseAndTip();

        // calculate bending angle
        calcBendAngle();

        // show the result
        int key = cv::waitKey(1);
        if (key == 'r')
            cv::imwrite(pathOutput + "bendingAngle_right.jpg", imgOutputAngleR);
        else if(key == 'l')
            cv::imwrite(pathOutput + "bendingAngle_left.jpg", imgOutputAngleL);
        else if(key == 'w')
        {
            cv::imwrite(pathOutput + "setupColor.jpg", imgColor);
            cv::imwrite(pathOutput + "setupSrc.jpg", imgSrc);
        }
        else if (key == 'q')
            return false;
    }

    return true;
}

/**************************************************************************
 * @Function
 *      main
 * 
 * @Details
 *      main for setup
**************************************************************************/
int main(int argc, char** argv)
{
    // node setup
    ros::init(argc, argv, "setup");
    
    // define imgFbController Class as ifc
    setup setup;

    // run
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if(!setup.setupExec())
        {
            ROS_WARN("Program stopped");
            break;
        }
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}