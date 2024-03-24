//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "imgProc/deformation.h"

/**************************************************************************
 * @Function
 *      initializePoint
 * 
 * @Details
 *      initialize point info
**************************************************************************/
void deformation::initializePoint()
{
    pointCurveNoObjL.clear();
    pointCurveNoObjL.shrink_to_fit();

    pointCurveNoObjR.clear();
    pointCurveNoObjR.shrink_to_fit();

    pointCurveObjL.clear();
    pointCurveObjL.shrink_to_fit();

    pointCurveObjR.clear();
    pointCurveObjR.shrink_to_fit();
}

/**************************************************************************
 * @Function
 *      calibration
 * 
 * @Details
 *      calculate parameters of fingers
**************************************************************************/
void deformation::calibration()
{
    // finger length
    lengthL = sqrt(pow(pointTipL.x - pointBaseL.x, 2) + pow(pointTipL.y - pointBaseL.y, 2));
    lengthR = sqrt(pow(pointTipR.x - pointBaseR.x, 2) + pow(pointTipR.y - pointBaseR.y, 2));
    // std::cout << "Length R" << lengthR << std::endl;

    // segment length
    lengthSegL = lengthL / numSegments;
    lengthSegR = lengthR / numSegments;

    // segment angle
    angleSegL = bendAngleL / numSegments;
    angleSegR = bendAngleR / numSegments;
}

/**************************************************************************
 * @Function
 *      curveEstimation
 * 
 * @Details
 *      curve estimation
**************************************************************************/
void deformation::curveEstimationR(int segStart, int segEnd, double angleStart, double angleSeg, std::string mode)
{
    double angleAccumulate = 0;
    double lengthAaccumulate = 0.;

    for (int i = segStart; i < segEnd; i++)  
    {
        // bending angle of each seg
        angleAccumulate = angleStart + angleSeg * (2 * i + 1);

        // calculated slope of the line from bending angle
        double slope = tan(3.14/2 - angleAccumulate);

        // estimate curve end point
        double j = 0.01;
        while(true)
        {
            double y = pointRefR.y + j;
            double x = (y - pointRefR.y) / slope + pointRefR.x;

            double lengthDiff = sqrt(pow(x - pointRefR.x, 2) + pow(y - pointRefR.y, 2));
            
            if( mode == "curve" && lengthDiff > lengthSegR)
            {
                pointCurveObjR.push_back(cv::Point2f(x, y));
                pointRefR = cv::Point2f(x,y);
                break;
            }
            else if(mode == "normal" && lengthDiff > lengthSegR)
            {
                pointCurveNoObjR.push_back(cv::Point2f(x, y));
                pointRefR = cv::Point2f(x,y);
                break;
            }
            j += 0.01;
        }
    }
    angleAccumulatePrevR = angleAccumulate;
}

/**************************************************************************
 * @Function
 *      curveEstimation L
 * 
 * @Details
 *      curve estimation
**************************************************************************/
void deformation::curveEstimationL(int segStart, int segEnd, double angleStart, double angleSeg, std::string mode)
{
    double angleAccumulate = 0;
    double lengthAaccumulate = 0.;

    for (int i = segStart; i < segEnd; i++)  
    {
        // bending angle of each seg
        angleAccumulate = angleStart + angleSeg * (2 * i + 1);

        // calculated slope of the line from bending angle
        double slope = tan(3.14/2 - angleAccumulate);

        // estimate curve end point
        double j = 0.01;
        while(true)
        {
            double y = pointRefL.y + j;
            double x = (y - pointRefL.y) / slope + pointRefL.x;

            double lengthDiff = sqrt(pow(x - pointRefL.x, 2) + pow(y - pointRefL.y, 2));
            
            if( mode == "curve" && lengthDiff > lengthSegL)
            {
                pointCurveObjL.push_back(cv::Point2f(x, y));
                pointRefL = cv::Point2f(x,y);
                break;
            }
            else if(mode == "normal" && lengthDiff > lengthSegL)
            {
                pointCurveNoObjL.push_back(cv::Point2f(x, y));
                pointRefL = cv::Point2f(x,y);
                break;
            }
            j += 0.01;
        }
        angleAccumulatePrevL = angleAccumulate;
    }

}

/**************************************************************************
 * @Function
 *      findFirstContact
 * 
 * @Details
 *      find first contact position
**************************************************************************/
void deformation::drawMyContoursR(std::vector<cv::Point> contours, cv::Mat img_mask, std::string mode, cv::Mat img_black)
{
    // declare for LineIterator
    cv::Point liStart;
    cv::Point liGoal;
    int lineSize = 1;

    cv::Point point_output = cv::Point(0,0);
    
    for (int i = 0; i < contours.size(); i++)
    {
        // specify start and end point for cv::LineIterator
        liStart = contours[i];

        if (i == contours.size() - 1)
            liGoal = contours[0];
        else    
            liGoal = contours[i + 1];

        // using cv::LineIterator
        cv::LineIterator LI(img_mask, liStart, liGoal, 8, false);
    
        // get point on the line
        std::vector<cv::Point> li_point(LI.count);
        for (int l = 0; l < LI.count; l++, ++LI)
            li_point[l] = LI.pos();

        // draw
        for (int k = 0; k < li_point.size(); k++)
        {
            if (mode == "obj")
            {
                img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x) = cv::Vec3b(0, 0, 255);
                for(int j = 1; j <= lineSize; j++)
                {
                    img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x + j) = cv::Vec3b(0, 0, 255);
                    img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x - j) = cv::Vec3b(0, 0, 255);
                }
            }
            else if (mode == "hand")
            {
                img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x)[0] += 255; //overwrite on Hand's line
                for(int j = 1; j <= lineSize; j++)
                {
                    img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x + j)[0] += 255;
                    img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x - j)[0] += 255;
                }
            }

            for(int j = 0; j <= lineSize; j++)
            {
                if ((img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x + j)) == cv::Vec3b(255, 0, 255) ||
                        (img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x - j)) == cv::Vec3b(255, 0, 255))
                {       
                    if( li_point[k].y > point_output.y)
                        point_output = li_point[k];
                }
            }
            
        }
    }

    if (mode == "hand")
        pointContactR = point_output;
}

/**************************************************************************
 * @Function
 *      findFirstContact
 * 
 * @Details
 *      find first contact position
**************************************************************************/
void deformation::drawMyContoursL(std::vector<cv::Point> contours, cv::Mat img_mask, std::string mode, cv::Mat img_black)
{
    // declare for LineIterator
    cv::Point liStart;
    cv::Point liGoal;
    int lineSize = 1;

    cv::Point point_output = cv::Point(0,0);
    
    for (int i = 0; i < contours.size(); i++)
    {
        // specify start and end point for cv::LineIterator
        liStart = contours[i];

        if (i == contours.size() - 1)
            liGoal = contours[0];
        else    
            liGoal = contours[i + 1];

        // using cv::LineIterator
        cv::LineIterator LI(img_mask, liStart, liGoal, 8, false);
    
        // get point on the line
        std::vector<cv::Point> li_point(LI.count);
        for (int l = 0; l < LI.count; l++, ++LI)
            li_point[l] = LI.pos();

        // draw
        for (int k = 0; k < li_point.size(); k++)
        {
            if (mode == "obj")
            {
                img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x) = cv::Vec3b(0, 0, 255);
                for(int j = 1; j <= lineSize; j++)
                {
                    img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x + j) = cv::Vec3b(0, 0, 255);
                    img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x - j) = cv::Vec3b(0, 0, 255);
                }
            }
            else if (mode == "hand")
            {
                img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x)[0] += 255; //overwrite on Hand's line
                for(int j = 1; j <= lineSize; j++)
                {
                    img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x + j)[0] += 255;
                    img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x - j)[0] += 255;
                }
            }

            for(int j = 0; j <= lineSize; j++)
            {
                if ((img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x + j)) == cv::Vec3b(255, 0, 255) ||
                        (img_black.at<cv::Vec3b>(li_point[k].y, li_point[k].x - j)) == cv::Vec3b(255, 0, 255))
                {       
                    if( li_point[k].y > point_output.y)
                        point_output = li_point[k];
                }
            }
            
        }
    }

    if (mode == "hand")
        pointContactL = point_output;
}

/**************************************************************************
 * @Function
 *      findFirstContact
 * 
 * @Details
 *      find first contact position
**************************************************************************/
void deformation::findFirstContactR()
{
    // output
    cv::Point2f pointContact;

    int i = 0;
    while( i <= 100)
    {
        // image for drawing 
        imgContactR = cv::Mat::zeros(imgSrcR.size(), imgSrcR.type());

        // preparation
        cv::Mat imgContactLineR = imgContactR.clone();
        pointRefR = pointBaseR;
        pointCurveNoObjR.push_back(pointRefR);

        // curve estimation
        int lineSize = 2;
        curveEstimationR(0, numSegments, 0, angleSegR * i / 100, "normal");
        for(int j = 0; j < pointCurveNoObjR.size()-1; j++)
            cv::line(imgContactLineR, pointCurveNoObjR[j], pointCurveNoObjR[j+1], cv::Scalar(0, 255, 0), lineSize, cv::LINE_4);
        
        // curve estimation mask
        cv::Mat imgContactLineHsvR, imgContactLineMaskR;
        cv::cvtColor(imgContactLineR, imgContactLineHsvR, cv::COLOR_BGR2HSV);
        cv::Scalar lower_hand(10, 0, 0);
        cv::Scalar upper_hand(179, 255, 255);
        cv::inRange(imgContactLineHsvR, lower_hand, upper_hand, imgContactLineMaskR);

        // get contour from mask
        std::vector<cv::Point> contourContactR;
        contourContactR = getLargestContour(imgContactLineMaskR);

        // Line Iterator
        drawMyContoursR(contourObjR, imgMaskObjR, "obj", imgContactR);
        drawMyContoursR(contourContactR, imgContactLineMaskR, "hand", imgContactR);
        // cv::imshow("Li"+ std::to_string(i), imgContactR);

        if(pointContactR != cv::Point2f(0, 0))
        {
            cv::imshow("contact", imgContactR);
            cv::imshow("mask", imgContactLineMaskR);
            // imwrite(output_path + input_img_num + "_" + to_string(seg_N) + "_input_black_v4" + ".jpg", img_black);
            cv::circle(imgOutputR, pointContactR,  8, cv::Scalar(255,  255,  255), -1);
            cv::circle(imgOutputR, pointContactR,  6, cv::Scalar(0,   0,  255), -1);
            pointCurveNoObjR.clear();
            break;
        }

        i += 5;
        pointCurveNoObjR.clear();

    }
}

/**************************************************************************
 * @Function
 *      findFirstContact
 * 
 * @Details
 *      find first contact position
**************************************************************************/
void deformation::findFirstContactL()
{
    int i = 0;
    while( i <= 100)
    {
        // image for drawing 
        imgContactL = cv::Mat::zeros(imgSrcL.size(), imgSrcL.type());

        // preparation
        cv::Mat imgContactLineL = imgContactL.clone();
        pointRefL = pointBaseL;
        pointCurveNoObjL.push_back(pointRefL);

        // curve estimation
        int lineSize = 2;
        curveEstimationL(0, numSegments, 0, angleSegL * i / 100, "normal");
        for(int j = 0; j < pointCurveNoObjL.size()-1; j++)
            cv::line(imgContactLineL, pointCurveNoObjL[j], pointCurveNoObjL[j+1], cv::Scalar(0, 255, 0), lineSize, cv::LINE_4);
        
        // curve estimation mask
        cv::Mat imgContactLineHsvL, imgContactLineMaskL;
        cv::cvtColor(imgContactLineL, imgContactLineHsvL, cv::COLOR_BGR2HSV);
        cv::Scalar lower_hand(10, 0, 0);
        cv::Scalar upper_hand(179, 255, 255);
        cv::inRange(imgContactLineHsvL, lower_hand, upper_hand, imgContactLineMaskL);

        // get contour from mask
        std::vector<cv::Point> contourContactL;
        contourContactL = getLargestContour(imgContactLineMaskL);

        // Line Iterator
        drawMyContoursL(contourObjL, imgMaskObjL, "obj", imgContactL);
        drawMyContoursL(contourContactL, imgContactLineMaskL, "hand", imgContactL);
        // cv::imshow("Li; L"+ std::to_string(i), imgContactL);

        if(pointContactL != cv::Point2f(0, 0))
        {
            cv::imshow("contactL", imgContactL);
            cv::imshow("maskL", imgContactLineMaskL);
            // imwrite(output_path + input_img_num + "_" + to_string(seg_N) + "_input_black_v4" + ".jpg", img_black);
            cv::circle(imgOutputL, pointContactL,  8, cv::Scalar(255,  255,  255), -1);
            cv::circle(imgOutputL, pointContactL,  6, cv::Scalar(0,   0,  255), -1);
            pointContactLbkup = pointContactL;  // just for paper
            pointCurveNoObjL.clear();
            break;
        }

        i += 5;
        pointCurveNoObjL.clear();

    }
}

/**************************************************************************
 * @Function
 *      firstDeformation
 * 
 * @Details
 *      estimate first deformation
**************************************************************************/
void deformation::firstDeformationR()
{
    // contact length and angle
    lengthContactR = sqrt(pow(pointContactR.x - pointBaseR.x, 2) + pow(pointContactR.y - pointBaseR.y, 2));
    angleContactR = atan2((pointContactR.x - pointBaseR.x),(pointContactR.y - pointBaseR.y));

    // contact segment
    numSegmentContactR = round(lengthContactR / lengthR * numSegments);
    // cv::circle(imgOutputR, cv::Point2f(pointBaseR.x, pointBaseR.y + lengthSegR * numSegmentContactR),  8, cv::Scalar(255,  255,  255), -1);
    // cv::circle(imgOutputR, cv::Point2f(pointBaseR.x, pointBaseR.y + lengthSegR * numSegmentContactR),  6, cv::Scalar(255,   0,  0), -1);

    if(lengthContactR > lengthR + 20 || pointContactR == cv::Point2f(0., 0.))   // if not contact
    {
        // std::cout << "----- No contact:R -----" << std::endl;
        pointRefR = pointBaseR;
        pointCurveObjR.push_back(pointRefR);
        curveEstimationR(0, numSegments, 0, angleSegR, "curve");
        flagContactOnlyOnceR = 0;
    }
    else if (numSegmentContactR == numSegments) // contact at the tip
    {
        // std::cout << "----- Contact at the tip:R -----" << std::endl;
        pointRefR = pointBaseR;
        pointCurveObjR.push_back(pointRefR);
        curveEstimationR(0, numSegments, (-1)*(bendAngleR - angleContactR), angleSegR, "curve");
        flagContactOnlyOnceR = 0;
    }
    else    // contact more than once
        flagContactOnlyOnceR = 1;
}

/**************************************************************************
 * @Function
 *      firstDeformation
 * 
 * @Details
 *      estimate first deformation
**************************************************************************/
void deformation::firstDeformationL()
{
    // contact length and angle
    lengthContactL = sqrt(pow(pointContactL.x - pointBaseL.x, 2) + pow(pointContactL.y - pointBaseL.y, 2));
    angleContactL = atan2((pointContactL.x - pointBaseL.x),(pointContactL.y - pointBaseL.y));

    // contact segment
    numSegmentContactL = round(lengthContactL / lengthL * numSegments);
    cv::circle(imgOutputL, cv::Point2f(pointBaseL.x, pointBaseL.y + lengthSegL * numSegmentContactL),  8, cv::Scalar(255,  255,  255), -1);
    cv::circle(imgOutputL, cv::Point2f(pointBaseL.x, pointBaseL.y + lengthSegL * numSegmentContactL),  6, cv::Scalar(255,   0,  0), -1);

    if(lengthContactL > lengthL + 20 || pointContactL == cv::Point2f(0., 0.))   // if not contact
    {
        // std::cout << "----- No contact:L -----" << std::endl;
        pointRefL = pointBaseL;
        pointCurveObjL.push_back(pointRefL);
        curveEstimationL(0, numSegments, 0, angleSegL, "curve");
        flagContactOnlyOnceL = 0;
    }
    else if (numSegmentContactL == numSegments) // contact at the tip
    {
        // std::cout << "----- Contact at the tip:L -----" << std::endl;
        pointRefL = pointBaseL;
        pointCurveObjL.push_back(pointRefL);
        curveEstimationL(0, numSegments, (-1)*(bendAngleL - angleContactL), angleSegL, "curve");
        flagContactOnlyOnceL = 0;
    }
    else    // contact more than once
        flagContactOnlyOnceL = 1;
}

/**************************************************************************
 * @Function
 *      findSecondContactR
 * 
 * @Details
 *      find second contact position
**************************************************************************/
void deformation::findSecondContactR()
{
    // info about circle
    cv::Point2f pointCircleCenter = pointContactR;
    double radius = (numSegments - numSegmentContactR) * lengthSegR;
    
    // initialize image for drawing 
    imgContactSecondR = cv::Mat::zeros(imgSrcR.size(), imgSrcR.type());

    // draw circle
    cv::Mat imgCircle = imgContactSecondR.clone();
    cv::circle(imgCircle, pointCircleCenter, radius, cv::Scalar(0, 255, 0), 1, cv::LINE_4);
    
    // mask image for circle 
    cv::Mat imgCircleHSV, imgCircleMask;
    cv::cvtColor(imgCircle, imgCircleHSV, cv::COLOR_BGR2HSV);
    cv::Scalar lower(10, 0, 0);
    cv::Scalar upper(179, 255, 255);
    cv::inRange(imgCircleHSV, lower, upper, imgCircleMask);

    // get contour from mask
    std::vector<cv::Point> contourCircle;
    contourCircle = getLargestContour(imgCircleMask);

    // Line Iterator
    drawMyContoursR(contourObjR, imgMaskObjR, "obj", imgContactSecondR);
    drawMyContoursR(contourCircle, imgCircleMask, "hand", imgContactSecondR);
    // cv::imshow("Second Contact R", imgContactSecondR);

    cv::circle(imgOutputR, pointContactR,  8, cv::Scalar(255,  255,  255), -1);
    cv::circle(imgOutputR, pointContactR,  6, cv::Scalar(0,  255,  0), -1);

    pointCurveNoObjR.clear();
}

/**************************************************************************
 * @Function
 *      findSecondContactL
 * 
 * @Details
 *      find second contact position
**************************************************************************/
void deformation::findSecondContactL()
{
    // info about circle
    cv::Point2f pointCircleCenter = pointContactL;
    double radius = (numSegments - numSegmentContactL) * lengthSegL;
    
    // initialize image for drawing 
    imgContactSecondL = cv::Mat::zeros(imgSrcL.size(), imgSrcL.type());

    // draw circle
    cv::Mat imgCircle = imgContactSecondL.clone();
    cv::circle(imgCircle, pointCircleCenter, radius, cv::Scalar(0, 255, 0), 1, cv::LINE_4);
    
    // mask image for circle 
    cv::Mat imgCircleHSV, imgCircleMask;
    cv::cvtColor(imgCircle, imgCircleHSV, cv::COLOR_BGR2HSV);
    cv::Scalar lower(10, 0, 0);
    cv::Scalar upper(179, 255, 255);
    cv::inRange(imgCircleHSV, lower, upper, imgCircleMask);

    // get contour from mask
    std::vector<cv::Point> contourCircle;
    contourCircle = getLargestContour(imgCircleMask);

    // Line Iterator
    drawMyContoursL(contourObjL, imgMaskObjL, "obj", imgContactSecondL);
    drawMyContoursL(contourCircle, imgCircleMask, "hand", imgContactSecondL);
    cv::imshow("Second Contact L", imgContactSecondL);

    pointCurveNoObjL.clear();
}

/**************************************************************************
 * @Function
 *     secondDeformation
 * 
 * @Details
 *      estimate second deformation
**************************************************************************/
void deformation::secondDeformationR()
{
    // contact length and angle
    angleContactSecondR = atan2((pointContactR.x - pointBaseR.x),(pointContactR.y - pointBaseR.y));

    // compare angle
    double angleCompare = angleAccumulatePrevR + (bendAngleR - angleContactR);
    
    // deformation
    if( abs(angleContactSecondR) > abs(angleCompare) || pointContactR == cv::Point2f(0., 0.))
    {
        // std::cout << "----- Not fully adapt:R -----" << std::endl;
        pointRefR = pointBaseR;
        pointCurveObjR.push_back(pointRefR);
        curveEstimationR(0, numSegmentContactR, 0, angleContactR / numSegmentContactR, "curve");    // first
        curveEstimationR(0, (numSegments - numSegmentContactR), angleAccumulatePrevR,(bendAngleR - angleContactR) / (numSegments - numSegmentContactR), "curve");
    }
    else
    {
        // std::cout << "----- Fully adapt:R -----" << std::endl;
        pointRefR = pointBaseR;
        pointCurveObjR.push_back(pointRefR);
        curveEstimationR(0, numSegmentContactR, 0, angleContactR / numSegmentContactR, "curve");    // first
        // curveEstimationR(0, (numSegments - numSegmentContactR), angleAccumulatePrevR, angleCompare / (numSegments - numSegmentContactR), "curve");
        curveEstimationR(0, (numSegments - numSegmentContactR), angleAccumulatePrevR, angleSegR, "curve");

    }
}

/**************************************************************************
 * @Function
 *     secondDeformation
 * 
 * @Details
 *      estimate second deformation
**************************************************************************/
void deformation::secondDeformationL()
{
    // contact length and angle
    angleContactSecondL = atan2((pointContactL.x - pointBaseL.x),(pointContactL.y - pointBaseL.y));

    // compare angle
    double angleCompare = angleAccumulatePrevL + (bendAngleL - angleContactL);
    
    // deformation
    if( abs(angleContactSecondL) > abs(angleCompare) || pointContactL == cv::Point2f(0., 0.))
    {
        // std::cout << "----- Not fully adapt:L -----" << std::endl;
        pointRefL = pointBaseL;
        pointCurveObjL.push_back(pointRefL);
        curveEstimationL(0, numSegmentContactL, 0, angleContactL / numSegmentContactL, "curve");    // first
        curveEstimationL(0, (numSegments - numSegmentContactL), angleAccumulatePrevL,(bendAngleL - angleContactL) / (numSegments - numSegmentContactL), "curve");
    }
    else
    {
        // std::cout << "----- Fully adapt:L -----" << std::endl;
        pointRefL = pointBaseL;
        pointCurveObjL.push_back(pointRefL);
        curveEstimationL(0, numSegmentContactL, 0, angleContactL / numSegmentContactL, "curve");    // first
        // curveEstimationL(0, (numSegments - numSegmentContactL), angleAccumulatePrevL, angleCompare / (numSegments - numSegmentContactL), "curve");
        curveEstimationL(0, (numSegments - numSegmentContactL), angleAccumulatePrevL, angleSegL, "curve");

    }
}

/**************************************************************************
 * @Function
 *     visualize result
 * 
 * @Details
 *      visualize result
**************************************************************************/
void deformation::visualizeResult()
{
    // prepare image
    imgOutputEst = imgSrc.clone();
    imgOutputEstColor = imgColor.clone();
    imgOutputEstL = imgSrcL.clone();
    imgOutputEstR = imgSrcR.clone();
    // imgOutputEstL = cv::Mat(imgSrcL, cv::Rect(0, 0, 2 * imgColor.cols/5, imgColor.rows));
    // imgOutputEstR = cv::Mat(imgSrcR, cv::Rect(2 * imgColor.cols/5, 0, 3*imgColor.cols/5, imgColor.rows));

    // draw result
    for (int i = 0; i < pointCurveObjL.size(); i++)
    {
        cv::circle(imgOutputEstL, pointCurveObjL[i],  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputEstL, pointCurveObjL[i],  smallD, cv::Scalar(255,   0,   0), -1);
        cv::circle(imgOutputEst, pointCurveObjL[i],  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputEst, pointCurveObjL[i],  smallD, cv::Scalar(255,   0,   0), -1);
        cv::circle(imgOutputEstColor, pointCurveObjL[i],  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputEstColor, pointCurveObjL[i],  smallD, cv::Scalar(255,   0,   0), -1);
    }
    for (int i = 0; i < pointCurveObjR.size(); i++)
    {
        cv::circle(imgOutputEstR, pointCurveObjR[i],  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputEstR, pointCurveObjR[i],  smallD, cv::Scalar(255,   0,   0), -1);
        cv::circle(imgOutputEst, cv::Point(pointCurveObjR[i].x + imgOutputEstL.cols, pointCurveObjR[i].y),  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputEst, cv::Point(pointCurveObjR[i].x + imgOutputEstL.cols, pointCurveObjR[i].y),  smallD, cv::Scalar(255,   0,   0), -1);
        cv::circle(imgOutputEstColor, cv::Point(pointCurveObjR[i].x + imgOutputEstL.cols, pointCurveObjR[i].y),  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputEstColor, cv::Point(pointCurveObjR[i].x + imgOutputEstL.cols, pointCurveObjR[i].y),  smallD, cv::Scalar(255,   0,   0), -1);
    }

    // show
    cv::imshow("imgOutputEst", imgOutputEst);
    cv::imshow("imgOutputEstL", imgOutputEstL);
    cv::imshow("imgOutputEstR", imgOutputEstR);
    cv::imshow("imgOutputEstColor", imgOutputEstColor);
}


/**************************************************************************
 * @Function
 *     validation
 * 
 * @Details
 *      validation
**************************************************************************/
void deformation::validation()
{
    // prepare image
    imgOutputCompare = imgColor.clone();
    // imgOutputCompareL = cv::Mat(imgSrcL, cv::Rect(0, 0, 2 * imgColor.cols/5, imgColor.rows));
    // imgOutputCompareR = cv::Mat(imgSrcR, cv::Rect(2 * imgColor.cols/5, 0, 3*imgColor.cols/5, imgColor.rows));

    imgOutputCompareL = cv::Mat(imgOutputCompare , cv::Rect(0, 0, 3 * imgOutputCompare.cols/5, imgOutputCompare.rows));
    cv::Mat imgSrcCValL = imgOutputCompareL.clone();

    // imgOutputCompareL = imgSrcL.clone();
    imgOutputCompareR = imgSrcR.clone();

    // imgOutputCompare = cv::imread(pathInput2);
    // imgOutputCompareL = cv::Mat(imgOutputCompare , cv::Rect(0, 0, imgOutputCompare.cols/2, imgOutputCompare.rows));
    // imgOutputCompareR = cv::Mat(imgOutputCompare , cv::Rect(imgOutputCompare.cols/2, 0, imgOutputCompare.cols/2, imgOutputCompare.rows));


    // draw estimation result
    int bigD = 6;
    int smallD = 4;
    for (int i = 0; i < pointCurveObjCompL.size(); i++)
    {
        cv::circle(imgOutputCompareL, pointCurveObjCompL[i],  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputCompareL, pointCurveObjCompL[i],  smallD, cv::Scalar(255,   0,   0), -1);
        cv::circle(imgOutputCompare, pointCurveObjCompL[i],  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputCompare, pointCurveObjCompL[i],  smallD, cv::Scalar(255,   0,   0), -1);
    }
    for (int i = 0; i < pointCurveObjCompR.size(); i++)
    {
        cv::circle(imgOutputCompareR, pointCurveObjCompR[i],  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputCompareR, pointCurveObjCompR[i],  smallD, cv::Scalar(255,   0,   0), -1);
        cv::circle(imgOutputCompare, cv::Point(pointCurveObjCompR[i].x + imgOutputCompareL.cols, pointCurveObjCompR[i].y),  bigD, cv::Scalar(255,   255,   255), -1);
        cv::circle(imgOutputCompare, cv::Point(pointCurveObjCompR[i].x + imgOutputCompareL.cols, pointCurveObjCompR[i].y),  smallD, cv::Scalar(255,   0,   0), -1);
    }

    // create mask image
    cv::Mat imgMaskL, imgHsvL, imgMaskR, imgHsvR;
    cv::cvtColor(imgSrcCValL, imgHsvL, cv::COLOR_BGR2HSV);
    cv::cvtColor(imgSrcR, imgHsvR, cv::COLOR_BGR2HSV);

    cv::Scalar lower_handl(H_MIN_HAND, S_MIN_HAND, V_MIN_HAND);
    cv::Scalar upper_handl(H_MAX_HAND, S_MAX_HAND, V_MAX_HAND);
    cv::Scalar lower_handr(H_MIN_HAND, S_MIN_HAND, V_MIN_HAND);
    cv::Scalar upper_handr(H_MAX_HAND, S_MAX_HAND, V_MAX_HAND - 6);
    
    cv::inRange(imgHsvL, lower_handl, upper_handl, imgMaskL);
    cv::inRange(imgHsvR, lower_handr, upper_handr, imgMaskR);
    cv::morphologyEx(imgMaskR, imgMaskR, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 2);
    cv::morphologyEx(imgMaskL, imgMaskL, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);
    // imshow("imgMaskR", imgMaskR);
    imshow("imgMaskL", imgMaskL);

    // find contours
    std::vector<cv::Point> contourValidL, contourValidR;
    contourValidL = getLargestContour(imgMaskL);
    contourValidR = getLargestContour(imgMaskR);

    // find tip and base
        // L
        int afford = 50;
        int num_base_l = 0, num_tip_l = 0;
        int num_base_l_update = 0, num_tip_l_update = 0;
        double x_base_l_prev = 0., x_tip_l_prev = 0.;
        double y_base_l_prev = imgSrcL.rows, y_tip_l_prev = 0.;

        // scan: first time
        for(int i = 0; i < contourValidL.size(); i++)
        {
            if(contourValidL[i].y < y_base_l_prev)
            {
                y_base_l_prev = contourValidL[i].y;
                num_base_l = i;
            }
            if(contourValidL[i].y > y_tip_l_prev)
            {
                y_tip_l_prev = contourValidL[i].y;
                num_tip_l = i;
            }
        }

        // scan: second time. cosider noize influence
        double minDiffBaseL = 100.;
        for (int i = 0; i < contourValidL.size(); i++)
        {
            double baseX = pow(pointBaseValL.x - contourValidL[i].x, 2);
            double baseY = pow(pointBaseValL.y - contourValidL[i].y, 2);
            double diffBase = sqrt(baseX + baseY);

            if( diffBase < minDiffBaseL)
            {
                minDiffBaseL = diffBase;
                num_base_l_update = i;
            }

            if(contourValidL[num_tip_l].y - afford < contourValidL[i].y )
            {
                if(contourValidL[i].x > x_tip_l_prev)
                {
                    x_tip_l_prev = contourValidL[i].x;
                    num_tip_l_update = i;
                }
            }
        }

        // --R 
        int num_base_r = 0, num_tip_r = 0;
        int num_base_r_update = 0, num_tip_r_update = 0;
        double x_base_r_prev = imgSrcR.cols, x_tip_r_prev = imgSrcR.cols;
        double y_base_r_prev = imgSrcR.rows, y_tip_r_prev = 0.;

        // scan: first time
        for(int i = 0; i < contourValidR.size(); i++)
        {
            if(contourValidR[i].y < y_base_r_prev)
            {
                y_base_r_prev = contourValidR[i].y;
                num_base_r = i;
            }
            if(contourValidR[i].y > y_tip_r_prev)
            {
                y_tip_r_prev = contourValidR[i].y;
                num_tip_r = i;
            }
        }

        // scan: second time. cosider noize influence
        double minDiffBaseR = 100.;
        for (int i = 0; i < contourValidR.size(); i++)
        {
            double baseX = pow(pointBaseValR.x - contourValidR[i].x, 2);
            double baseY = pow(pointBaseValR.y - contourValidR[i].y, 2);
            double diffBase = sqrt(baseX + baseY);

            if( diffBase < minDiffBaseR)
            {
                minDiffBaseR = diffBase;
                num_base_r_update = i;
            }

            if(contourValidR[num_tip_r].y - 10 < contourValidR[i].y )   // 75->30, 95->40, 115->30
            {
                if(contourValidR[i].x < x_tip_r_prev)
                {
                    x_tip_r_prev = contourValidR[i].x;
                    num_tip_r_update = i;
                }
            }
        }

    // calculate contour length
    double lengthContourL = 0., lengthContourSegmentL;
    for(int i = num_tip_l_update; i < num_base_l_update; i++)
    {
        double x = pow(contourValidL[i+1].x - contourValidL[i].x, 2);
        double y = pow(contourValidL[i+1].y - contourValidL[i].y, 2);
        lengthContourL += sqrt(x + y);
    }
    lengthContourSegmentL = lengthContourL / (numSegments + 1);

    double lengthContourR = 0., lengthContourSegmentR;
    for(int i = num_base_r_update; i < num_tip_r_update; i++)
    {
        double x = pow(contourValidR[i+1].x - contourValidR[i].x, 2);
        double y = pow(contourValidR[i+1].y - contourValidR[i].y, 2);
        lengthContourR += sqrt(x + y);
    }
    lengthContourSegmentR = lengthContourR / (numSegments + 1);

    // valification
    pointValL.push_back(pointBaseValL);
    cv::Point2f pointValStartL = pointBaseValL;
    
    for(int i = num_base_l_update; i >= num_tip_l_update; i--)
    {
        cv::Point pointValitRef = contourValidL[i];
        
        double valid_length = sqrt(pow(pointValStartL.x - pointValitRef.x ,2) + pow(pointValStartL.y - pointValitRef.y, 2));
        // if(valid_length >= lengthContourSegmentL )
        if(valid_length >= lengthSegL - 0.5)
        {
            pointValStartL = contourValidL[i];
            pointValL.push_back(contourValidL[i]);
        }
    }

    if(pointValL.size() < pointCurveObjCompL.size())
        pointValL.push_back(contourValidL[num_tip_l_update]);

    // ---- R
    pointValR.push_back(pointBaseValR);
    cv::Point2f pointValStartR = pointBaseValR;

    // for(int i = 0; i <= num_tip_r_update; i++)
    for(int i = num_base_r_update; i <= num_tip_r_update; i++)
    {
        cv::Point pointValitRef = contourValidR[i];
        
        double valid_length = sqrt(pow(pointValStartR.x - pointValitRef.x ,2) + pow(pointValStartR.y - pointValitRef.y, 2));
        // if(valid_length >= lengthContourSegmentR )
        if(valid_length >= lengthSegR - 0.)
        {
            pointValStartR = contourValidR[i];
            pointValR.push_back(contourValidR[i]);
        }
    }

    if(pointValR.size() < pointCurveObjCompR.size())
        pointValR.push_back(contourValidR[num_tip_r_update]);


    // plot
    for(int i = 0; i < pointValL.size(); i++)
    {
        cv::circle(imgOutputCompareL, pointValL[i],  bigD, cv::Scalar(255,  255,   255), -1);
        cv::circle(imgOutputCompareL, pointValL[i],  smallD, cv::Scalar(0,  255,   0), -1);
        cv::circle(imgOutputCompare, pointValL[i],  bigD, cv::Scalar(255,  255,   255), -1);
        cv::circle(imgOutputCompare, pointValL[i],  smallD, cv::Scalar(0,  255,   0), -1);
    }

    for(int i = 0; i < pointValR.size(); i++)
    {
        cv::circle(imgOutputCompareR, pointValR[i],  8, cv::Scalar(255,  255,   255), -1);
        cv::circle(imgOutputCompareR, pointValR[i],  6, cv::Scalar(0,  255,   0), -1);
        cv::circle(imgOutputCompare, cv::Point(pointValR[i].x + imgOutputCompareL.cols, pointValR[i].y),  bigD, cv::Scalar(255,  255,   255), -1);
        cv::circle(imgOutputCompare, cv::Point(pointValR[i].x + imgOutputCompareL.cols, pointValR[i].y),  smallD, cv::Scalar(0,  255,   0), -1);
    }


    // show
    cv::imshow("imgOutputCompare", imgOutputCompare);
    cv::imshow("imgOutputCompareL", imgOutputCompareL);
    cv::imshow("imgOutputCompareR", imgOutputCompareR);
    cv::imwrite(pathOutput + "imgColorCompare.jpg", imgColor);
    cv::imwrite(pathOutput + "imgSrcCompare.jpg", imgSrc);
    cv::imwrite(pathOutput + "imgOutputCompare.jpg", imgOutputCompare);
    cv::imwrite(pathOutput + "imgOutputCompareL.jpg", imgOutputCompareL);
    cv::imwrite(pathOutput + "imgOutputCompareR.jpg", imgOutputCompareR);

    // csv
    std::ofstream csv_writing_file;
    
    std::string filenameL = pathOutput + "resultL.csv";
    csv_writing_file.open(filenameL, std::ios::app);
    csv_writing_file << "estimation.x" << "," << "estimation.y" << "," << "valification.x" << "," << "valification.y" << "," << "diff" << std::endl;
    for(int i = 0; i <= numSegments; i++)
    {
        if((i == numSegments/2 ) || (i == numSegments))
        {
            std::cout << "i: " << i << std::endl;
            std::cout << "pointCurveObjCompL[i].x : " << pointCurveObjCompL[i].x << ":: pointValL[i].x : " << pointValL[i].x  << std::endl;
            std::cout << "pointCurveObjCompL[i].y : " << pointCurveObjCompL[i].y << ":: pointValL[i].y : " << pointValL[i].y  << std::endl;
            double diff = sqrt(pow(pointCurveObjCompL[i].x - pointValL[i].x ,2) + pow(pointCurveObjCompL[i].y - pointValL[i].y ,2));

            csv_writing_file << pointCurveObjCompL[i].x 
                                << "," 
                                << pointCurveObjCompL[i].y 
                                << "," 
                                << pointValL[i].x 
                                << "," 
                                << pointValL[i].y
                                << ","
                                << diff        
                                << std::endl; 
        }
    }
    csv_writing_file.close();


    std::string filenameR = pathOutput + "resultR.csv";
    csv_writing_file.open(filenameR, std::ios::app);
    csv_writing_file << "estimation.x" << "," << "estimation.y" << "," << "valification.x" << "," << "valification.y" << "," << "diff" << std::endl;
    for(int i = 0; i <= numSegments; i++)
    {
        if((i == numSegments/2 ) || (i == numSegments))
        {
            double diff = sqrt(pow(pointCurveObjCompR[i].x - pointValR[i].x ,2) + pow(pointCurveObjCompR[i].y - pointValR[i].y ,2));

            csv_writing_file << pointCurveObjCompR[i].x 
                                << "," 
                                << pointCurveObjCompR[i].y 
                                << "," 
                                << pointValR[i].x 
                                << "," 
                                << pointValR[i].y
                                << ","
                                << diff        
                                << std::endl; 
        }
    }
    csv_writing_file.close();

}

/**************************************************************************
 * @Function
 *     saveImage
 * 
 * @Details
 *      save image
**************************************************************************/
void deformation::saveImage()
{
    cv::imwrite(pathOutput + "imgColor.jpg", imgColor);
    cv::imwrite(pathOutput + "imgSrc.jpg", imgSrc);
    cv::imwrite(pathOutput + "imgContactL.jpg", imgContactL);
    cv::imwrite(pathOutput + "imgContactR.jpg", imgContactR);
    // cv::imwrite(pathOutput + "imgContactSecondL.jpg", imgContactSecondL);
    // cv::imwrite(pathOutput + "imgContactSecondR.jpg", imgContactSecondR);
    cv::imwrite(pathOutput + "imgOutputEst.jpg", imgOutputEst);
    cv::imwrite(pathOutput + "imgOutputEstL.jpg", imgOutputEstL);
    cv::imwrite(pathOutput + "imgOutputEstR.jpg", imgOutputEstR);
    cv::imwrite(pathOutput + "imgOutputEstColor.jpg", imgOutputEstColor);

    // // for IROS 2024 paper
    // cv::Mat imgOutputColorL = cv::Mat(imgColor, cv::Rect(0, 0, imgColor.cols/2, imgColor.rows));
    // cv::imshow("imgOutputColorLBefore", imgOutputColorL);
    // // std::vector<std::vector<cv::Point>> contoursHand, contoursObj;
    // // contoursHand.push_back(contourHandL);
    // // contoursObj.push_back(contourObjL);
    // // cv::drawContours(imgOutputColorL, contoursHand, 0, cv::Scalar(255, 255, 255), 6);
    // // cv::drawContours(imgOutputColorL, contoursHand, 0, cv::Scalar(255, 0, 0), 4);
    // // cv::drawContours(imgOutputColorL, contoursObj, 0, cv::Scalar(255, 255, 255), 6);
    // // cv::drawContours(imgOutputColorL, contoursObj, 0, cv::Scalar(0, 0, 255), 4);
    // cv::circle(imgOutputColorL, pointBaseL,  8, cv::Scalar(255,   255,   255), -1);
    // cv::circle(imgOutputColorL, pointBaseL,  6, cv::Scalar(255,   0,   0), -1);
    // cv::circle(imgOutputColorL, pointTipL,  8, cv::Scalar(255,   255,   255), -1);
    // cv::circle(imgOutputColorL, pointTipL,  6, cv::Scalar(255,   0,   0), -1);
    // cv::circle(imgOutputColorL, pointContactLbkup,  8, cv::Scalar(255,   255,   255), -1);
    // cv::circle(imgOutputColorL, pointContactLbkup,  6, cv::Scalar(0,   0,   255), -1);
    // // cv::circle(imgOutputColorL, pointContactL,  8, cv::Scalar(255,   255,   255), -1);
    // // cv::circle(imgOutputColorL, pointContactL,  6, cv::Scalar(0,  255,  0), -1);
    // cv::circle(imgOutputColorL, cv::Point2f(pointBaseL.x, pointBaseL.y + lengthSegL * numSegmentContactL),  8, cv::Scalar(255,  255,  255), -1);
    // cv::circle(imgOutputColorL, cv::Point2f(pointBaseL.x, pointBaseL.y + lengthSegL * numSegmentContactL),  6, cv::Scalar(255,   0,  0), -1);
    // cv::imshow("imgOutputColorL", imgOutputColorL);
}

/**************************************************************************
 * @Function
 *     saveImage
 * 
 * @Details
 *      save image
**************************************************************************/
void deformation::onlineDemo()
{
    imgDemo = imgColor.clone();

    int key = cv::waitKey(1);
    if (key == 'd')
    {
        pointCurveObjCompL = pointCurveObjL;
        pointCurveObjCompR = pointCurveObjR;
        flagDemo = 1;
    }

    if (flagDemo == 1)
    {
        for (int i = 0; i < pointCurveObjCompL.size(); i++)
        {
            cv::circle(imgDemo, pointCurveObjCompL[i],  bigD, cv::Scalar(255,   255,   255), -1);
            cv::circle(imgDemo, pointCurveObjCompL[i],  smallD, cv::Scalar(255,   0,   0), -1);
        }
        for (int i = 0; i < pointCurveObjCompR.size(); i++)
        {
            cv::circle(imgDemo, cv::Point(pointCurveObjCompR[i].x + imgDemo.cols / 2, pointCurveObjCompR[i].y),  bigD, cv::Scalar(255,   255,   255), -1);
            cv::circle(imgDemo, cv::Point(pointCurveObjCompR[i].x + imgDemo.cols / 2, pointCurveObjCompR[i].y),  smallD, cv::Scalar(255,   0,   0), -1);
        }
    }

    if (key == 'r')
        flagDemo = 10;

    imshow("Deformation Estimation", imgDemo);
}

/**************************************************************************
 * @Function
 *      setupExec
 * 
 * @Details
 *      execute program for setup.cpp
**************************************************************************/
bool deformation::deformationExec()
{
    if (!imgSrc.empty())
    {
        // ROS_INFO("Time stanp");

        // crop the image based on the image size
        cropImage();

        // create mask image
        maskImage();

        // obtain contours from the mask image 
        obtainContour();

        // find base and tip in the hand
        findBaseAndTip();

        //---------< for deformation estimation >----------
        // initialize
        initializePoint();

        // calibration
        calibration();

        // contact position
        findFirstContactL();
        findFirstContactR();

        //
        firstDeformationR();
        firstDeformationL();

        // 
        if(flagContactOnlyOnceR == 1)
        {
            findSecondContactR();
            secondDeformationR();
        }
        if(flagContactOnlyOnceL == 1)
        {
            findSecondContactL();
            secondDeformationL();
        }

        //
        visualizeResult();

        onlineDemo();

        cv::imshow("imgSrc", imgSrc);
        cv::imshow("imgOutputL", imgOutputL);
        cv::imshow("imgOutputR", imgOutputR);
        int key = cv::waitKey(1);
        if (key == 'v')
            validation();
        else if (key == 's')
        {
            saveImage();
            pointCurveObjCompL = pointCurveObjL;
            pointCurveObjCompR = pointCurveObjR;
            pointBaseValL = pointBaseL;
            pointBaseValR = pointBaseR;
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
    ros::init(argc, argv, "deformation");
    
    // define deformation class as deform
    deformation deform;

    // run
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        if(!deform.deformationExec())
        {
            ROS_WARN("Program stopped");
            break;
        }
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}