
#include "cvgui.h"

#define CVUI_IMPLEMENTATION
#include "cvui.h"

const char * WINDOW_NAME = "DispModule Parameters";

/******************************************************************************/
GUI::GUI()
{
//    this->updated = false;
}

/******************************************************************************/
bool GUI::isUpdated()
{
    return this->updated;
}

/******************************************************************************/
void GUI::initializeGUI(int &minDisparity, int &numberOfDisparities, int &SADWindowSize,
                       int &disp12MaxDiff, int &preFilterCap, int &uniquenessRatio,
                       int &speckleWindowSize, int &speckleRange, double &sigmaColorBLF,
                       double &sigmaSpaceBLF, double &wls_lambda, double &wls_sigma,
                       SM_BLF_FILTER &BLFfiltering, SM_WLS_FILTER &WLSfiltering,
                       SM_MATCHING_ALG &stereo_matching)
{

    std::cout << "GUI - initializeGUI\n";

//    this->params.minDisparity = &minDisparity;
//    this->params.numberOfDisparities = &numberOfDisparities;
//    this->params.SADWindowSize = &SADWindowSize;
//    this->params.disp12MaxDiff = &disp12MaxDiff;
//    this->params.preFilterCap = &preFilterCap;
//    this->params.uniquenessRatio = &uniquenessRatio;
//    this->params.speckleWindowSize = &speckleWindowSize;
//    this->params.speckleRange = &speckleRange;
//    this->params.sigmaColorBLF = &sigmaColorBLF;
//    this->params.sigmaSpaceBLF = &sigmaSpaceBLF;
//    this->params.wls_lambda = &wls_lambda;
//    this->params.wls_sigma = &wls_sigma;
//    this->params.BLFfiltering = BLFfiltering;
//    this->params.WLSfiltering = WLSfiltering;
//    this->params.stereo_matching = stereo_matching;

    this->params.minDisparity = minDisparity;
    this->params.numberOfDisparities = numberOfDisparities;
    this->params.SADWindowSize = SADWindowSize;
    this->params.disp12MaxDiff = disp12MaxDiff;
    this->params.preFilterCap = preFilterCap;
    this->params.uniquenessRatio = uniquenessRatio;
    this->params.speckleWindowSize = speckleWindowSize;
    this->params.speckleRange = speckleRange;
    this->params.sigmaColorBLF = sigmaColorBLF;
    this->params.sigmaSpaceBLF = sigmaSpaceBLF;
    this->params.wls_lambda = wls_lambda;
    this->params.wls_sigma = wls_sigma;
    this->params.BLFfiltering = BLFfiltering;
    this->params.WLSfiltering = WLSfiltering;
    this->params.stereo_matching = stereo_matching;

    std::cout << "GUI - initializeGUI end\n";
    std::cout << this->params.sigmaColorBLF << std::endl;

    this->convertEnumToID();
    this->initializeGUI();
}

/******************************************************************************/
void GUI::initializeGUI()
{

    this->done = false;
    this->updated = false;
    this->recalibrate = false;
    this->save_calibration = false;
    this->set_default = false;
    this->save_parameters = false;

    frame = cv::Mat(cv::Size(450, cvuimine::estimateHeight(10,4,2)), CV_8UC3);

    cvui::init(WINDOW_NAME, 3);

    // TODO conversione da ENUMs a parametri id interni
}

/******************************************************************************/
void GUI::killGUI()
{

}

/******************************************************************************/
void GUI::updateGUI()
{

    std::cout << "GUI UPDATE - top\n";

    if(cv::getWindowProperty(WINDOW_NAME, 0) < 0)
    {
        this->done = false;
        return;
    }

    std::cout << "GUI UPDATE - top 2\n";
    std::cout << this->params.sigmaColorBLF << std::endl;


    frame = cv::Scalar(20, 22, 23);

    // 0. Resets the local state of the GUI

    cvuimine::reset();

    this->updated = false;

    // GUI definition

    // 1. Stereo Matching and filtering parameters

    this->updated |= cvuimine::trackbar<int>("SADWindowSize", frame, &(this->params.SADWindowSize), 3, 31, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuimine::trackbar<int>("disp12MaxDiff", frame, &(this->params.disp12MaxDiff), 0, 30, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuimine::trackbar<int>("preFilterCap", frame, &(this->params.preFilterCap), 0, 100, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuimine::trackbar<int>("uniquenessRatio", frame, &(this->params.uniquenessRatio), 5, 20, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuimine::trackbar<int>("speckleWindowSize", frame, &(this->params.speckleWindowSize), 0, 200, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuimine::trackbar<int>("speckleRange", frame, &(this->params.speckleRange), 1, 16, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuimine::trackbar<double>("sigmaColorBLF", frame, &(this->params.sigmaColorBLF), 1.0, 30.0, 1, "%.1Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuimine::trackbar<double>("sigmaSpaceBLF", frame, &(this->params.sigmaSpaceBLF), 1.0, 30.0, 1, "%.1Lf", cvui::TRACKBAR_HIDE_LABELS);
    this->updated |= cvuimine::trackbar<double>("WLS lambda", frame, &(this->params.wls_lambda), 500.0, 30000.0, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS | cvui::TRACKBAR_DISCRETE, 500.0);
    this->updated |= cvuimine::trackbar<double>("WLS sigma", frame, &(this->params.wls_sigma), 0.1, 10.0, 1, "%1.Lf", cvui::TRACKBAR_HIDE_LABELS);

    // 2. Multiple choice controls

    this->updated |= cvuimine::radioButtons(frame, "Num. of Disparities:", {"32", "64", "96", "128"}, {20, 65, 110, 155}, 1);
    this->updated |= cvuimine::radioButtons(frame, "Stereo Matching Alg.:", {"SGBM", "SGBM_CUDA", "LibElas"}, {20, 90, 190}, 0);
    this->updated |= cvuimine::radioButtons(frame, "Bilateral Filtering:", {"No BLF", "Original BLF", "CUDA BLF"}, {20, 90, 190}, 0);
    this->updated |= cvuimine::radioButtons(frame, "Weigthed LS Filtering:", {"No WLS", "WLS", "WLS w/ lr cons."}, {20, 100, 155}, 0);

    // 3. Button to trigger the recalibration of the system

    this->recalibrate = cvuimine::button(frame, "&Recalibrate", 10, 0);
    this->updated |= this->recalibrate;

    // 4. Button to save the calibration parameters to the local file

    this->save_calibration = cvuimine::button(frame, "&Save Calibration", 130, 0);
    this->updated |= this->save_calibration;


    std::cout << "GUI UPDATE - before buttons\n";

    // 5. Button to close the GUI

    if(cvuimine::button(frame, "&Quit", 282, 0))
    {
        this->done = true;
        cv::destroyWindow(WINDOW_NAME);
        return;
    }

    // 6. Button to set the default parameters

    this->set_default = cvuimine::button(frame, "&Default Param.", 10, 1);
    this->updated |= this->set_default;

    // 7. Save the matching/filtering parameters to a local configuration file

    this->save_parameters = cvuimine::button(frame, "Save &Parameters", 155, 1);
    this->updated |= this->save_parameters;

    // acquire the selected radiobuttons indexes..

    this->stereo_matching_id = cvuimine::getRadioIndex(1);
    this->BLFfiltering_id = cvuimine::getRadioIndex(2);
    this->WLSfiltering_id = cvuimine::getRadioIndex(3);

    // .. then converts it to the corresponding ENUM elements

    this->convertIDToEnum();

    // Since cvui::init() received a param regarding waitKey,
    // there is no need to call cv::waitKey() anymore. cvui::update()
    // will do it automatically.
    cvui::update();

    std::cout << "GUI UPDATE - before imshow\n";

    cv::imshow(WINDOW_NAME, frame);

    return;

}

//void GUI::getParams(int& minDisparity, int& numberOfDisparities, int& SADWindowSize,
//                    int& disp12MaxDiff, int& preFilterCap, int& uniquenessRatio,
//                    int& speckleWindowSize, int& speckleRange, double& sigmaColorBLF,
//                    double& sigmaSpaceBLF, double& wls_lambda, double& wls_sigma,
//                    SM_BLF_FILTER& BLFfiltering, SM_WLS_FILTER& WLSfiltering,
//                    SM_MATCHING_ALG& stereo_matching)
//{
//    this->convertIDToEnum();

//    minDisparity = *(this->params.minDisparity);
//    numberOfDisparities = *(this->params.numberOfDisparities);
//    SADWindowSize = *(this->params.SADWindowSize);
//    disp12MaxDiff = *(this->params.disp12MaxDiff);
//    preFilterCap = *(this->params.preFilterCap);
//    uniquenessRatio = *(this->params.uniquenessRatio);
//    speckleWindowSize = *(this->params.speckleWindowSize);
//    speckleRange = *(this->params.speckleRange);
//    sigmaColorBLF = *(this->params.sigmaColorBLF);
//    sigmaSpaceBLF = *(this->params.sigmaSpaceBLF);
//    wls_lambda = *(this->params.wls_lambda);
//    wls_sigma = *(this->params.wls_sigma);
//    BLFfiltering = this->params.BLFfiltering;
//    WLSfiltering = this->params.WLSfiltering;
//    stereo_matching = this->params.stereo_matching;
//}

/******************************************************************************/
void GUI::convertIDToEnum()
{
    switch(this->stereo_matching_id)
    {
        case 0:
            this->params.stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
            break;
        case 1:
            this->params.stereo_matching = SM_MATCHING_ALG::SGBM_CUDA;
            break;
        case 2:
            this->params.stereo_matching = SM_MATCHING_ALG::LIBELAS;
            break;
    }

    switch(this->BLFfiltering_id)
    {
        case 0:
            this->params.BLFfiltering = SM_BLF_FILTER::BLF_DISABLED;
            break;
        case 1:
            this->params.BLFfiltering = SM_BLF_FILTER::BLF_ORIGINAL;
            break;
        case 2:
            this->params.BLFfiltering = SM_BLF_FILTER::BLF_CUDA;
            break;
    }

    switch(this->WLSfiltering_id)
    {
        case 0:
            this->params.WLSfiltering = SM_WLS_FILTER::WLS_DISABLED;
            break;
        case 1:
            this->params.WLSfiltering = SM_WLS_FILTER::WLS_ENABLED;
            break;
        case 2:
            this->params.WLSfiltering = SM_WLS_FILTER::WLS_LRCHECK;
            break;
    }
}

/******************************************************************************/
void GUI::convertEnumToID()
{
    switch(this->stereo_matching)
    {
        case SM_MATCHING_ALG::SGBM_OPENCV:
            this->stereo_matching_id = 0;
            break;
        case SM_MATCHING_ALG::SGBM_CUDA:
            this->stereo_matching_id = 1;
            break;
        case SM_MATCHING_ALG::LIBELAS:
            this->stereo_matching_id = 2;
            break;

    }

    switch(this->BLFfiltering)
    {
        case SM_BLF_FILTER::BLF_DISABLED:
            this->BLFfiltering_id = 0;
            break;
        case SM_BLF_FILTER::BLF_ORIGINAL:
            this->BLFfiltering_id = 1;
            break;
        case SM_BLF_FILTER::BLF_CUDA:
            this->BLFfiltering_id = 2;
            break;

    }

    switch(this->WLSfiltering)
    {
        case SM_WLS_FILTER::WLS_DISABLED:
            this->WLSfiltering_id = 0;
            break;
        case SM_WLS_FILTER::WLS_ENABLED:
            this->WLSfiltering_id = 1;
            break;
        case SM_WLS_FILTER::WLS_LRCHECK:
            this->WLSfiltering_id = 2;
            break;

    }
}

/******************************************************************************/
bool GUI::isDone()
{
    return this->done;
}

/******************************************************************************/
GUI::~GUI()
{
    this->killGUI();
}

/******************************************************************************/
bool GUI::toRecalibrate()
{
    return this->recalibrate;
}

/******************************************************************************/
bool GUI::toSaveCalibration()
{
    return this->save_calibration;
}


/******************************************************************************/
void GUI::setUpdated(bool v)
{
    this->updated = v;
    this->save_calibration = false;
}

/******************************************************************************/
void GUI::setUpdated(bool v1, bool v2)
{
    this->updated = v1;
    this->recalibrate = v2;
    this->save_calibration = false;
}
