/*******************************************************
deformation.h
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "imgProc/procCommon.h"

//-----------------------------------------------------
// CLASS
//-----------------------------------------------------
class deformation : public procCommon
{
    private:
        /*--- information of the finger*/
        double lengthL, lengthR; // length of each finger
        double lengthSegL, lengthSegR;  // length of the segment
        double angleSegL, angleSegR;    // angle of the segment [rad]

        /*--- points for estimation: reference*/
        cv::Point2f pointRefL, pointRefR;
        std::vector<cv::Point2f> pointCurveNoObjL, pointCurveNoObjR;
        std::vector<cv::Point2f> pointCurveObjL, pointCurveObjR;
        std::vector<cv::Point2f> pointCurveObjCompL, pointCurveObjCompR;    // for validation
        std::vector<cv::Point2f> pointValL, pointValR;
        cv::Point2f pointContactL, pointContactR;
        cv::Point2f pointContactLbkup;

        /*--- first contact ---*/
        double lengthContactL, lengthContactR; 
        double angleContactL, angleContactR;
        int numSegmentContactL, numSegmentContactR;

        /*--- second contact ---*/
        double angleAccumulatePrevL, angleAccumulatePrevR;  
        double angleContactSecondL, angleContactSecondR;

        /*--- images ---*/
        cv::Mat imgContactL, imgContactR;
        cv::Mat imgContactSecondL, imgContactSecondR;
        cv::Mat imgOutputEst, imgOutputEstL, imgOutputEstR, imgOutputEstColor;
        cv::Mat imgOutputCompare, imgOutputCompareL, imgOutputCompareR;
        cv::Mat imgDemo;

        /*--- Flag ---*/
        int flagContactOnlyOnceL, flagContactOnlyOnceR;    // 0: true, 1: false
        int bigD = 6;
        int smallD = 4;
        int flagDemo = 0;

        /*--- functions ---*/
        void initializePoint();

        // calculate parameter of fingers
        void calibration();

        // 
        void curveEstimationR(int segStart, int segEnd, double angleStart, double angleSeg, std::string mode);
        void curveEstimationL(int segStart, int segEnd, double angleStart, double angleSeg, std::string mode);

        void drawMyContoursL(std::vector<cv::Point> contours, cv::Mat img_mask, std::string mode, cv::Mat img_black);
        void drawMyContoursR(std::vector<cv::Point> contours, cv::Mat img_mask, std::string mode, cv::Mat img_black);

        // 
        void findFirstContactR();
        void findFirstContactL();

        void firstDeformationR();
        void firstDeformationL();

        void findSecondContactR();
        void findSecondContactL();

        void secondDeformationR();
        void secondDeformationL();

        void visualizeResult();

        void validation();

        void saveImage();

        void onlineDemo();

    public:
        deformation()
        {
        }

        ~deformation()
        {
        }

        // execute defromation program
        bool deformationExec();
};