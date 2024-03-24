/*******************************************************
setup.h
*******************************************************/
//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "imgProc/procCommon.h"

//-----------------------------------------------------
// CLASS
//-----------------------------------------------------
class setup : public procCommon
{
    private:
        /*--- images ---*/
        cv::Mat imgOutputAngleR, imgOutputAngleL;

        /*--- functions ---*/
        void calcBendAngle();

    public:
        setup()
        {
        }

        ~setup()
        {
        }

        bool setupExec();
};