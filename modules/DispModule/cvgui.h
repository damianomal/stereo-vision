#ifndef _GUI_H_
#define _GUI_H_

//#include "StereoMatcher_enums.h"
#include "StereoMatcher.h"
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// the struct containing the parameters handled by the GUI

//typedef struct {

//    int * uniquenessRatio;
//    int * speckleWindowSize;
//    int * speckleRange;
//    int * numberOfDisparities;
//    int * SADWindowSize;
//    int * minDisparity;
//    int * preFilterCap;
//    int * disp12MaxDiff;

//    double * sigmaColorBLF;
//    double * sigmaSpaceBLF;
//    double * wls_lambda;
//    double * wls_sigma;

//    SM_BLF_FILTER BLFfiltering;
//    SM_WLS_FILTER WLSfiltering;
//    SM_MATCHING_ALG stereo_matching;

//} Params;

typedef struct {

    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int numberOfDisparities;
    int SADWindowSize;
    int minDisparity;
    int preFilterCap;
    int disp12MaxDiff;

    double sigmaColorBLF;
    double sigmaSpaceBLF;
    double wls_lambda;
    double wls_sigma;

    SM_BLF_FILTER BLFfiltering;
    SM_WLS_FILTER WLSfiltering;
    SM_MATCHING_ALG stereo_matching;

} Params;



class GUI
{

private:

    //
    bool done;

    //
    bool updated;

    //
    bool recalibrate;

    //
    bool save_calibration;

    //
    bool set_default;

    //
    bool save_parameters;

    Params params;

    cv::Mat frame;

//    bool useWLS;
//    bool useBLF;
//    bool left_right;

    SM_BLF_FILTER BLFfiltering;
    SM_WLS_FILTER WLSfiltering;
    SM_MATCHING_ALG stereo_matching;

    int BLFfiltering_id;
    int WLSfiltering_id;
    int stereo_matching_id;

public:

    /**
    * XXXXXXXXXXXXXXX
    *
    */
    void killGUI();

    /**
    * Initialize the internal GUI parameters
    *
    */
    void initializeGUI();

    /**
    * XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    *
    */
    void initializeGUI(int &minDisparity, int &numberOfDisparities, int &SADWindowSize,
                       int &disp12MaxDiff, int &preFilterCap, int &uniquenessRatio,
                       int &speckleWindowSize, int &speckleRange, double &sigmaColorBLF,
                       double &sigmaSpaceBLF, double &wls_lambda, double &wls_sigma,
                       SM_BLF_FILTER &BLFfiltering, SM_WLS_FILTER &WLSfiltering,
                       SM_MATCHING_ALG &stereo_matching);

    /**
    * This is the GUI main update loop
    *
    */
    void updateGUI();

    /**
    * XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    *
    */
    void getParams(int& minDisparity, int& numberOfDisparities, int& SADWindowSize,
                   int& disp12MaxDiff, int& preFilterCap, int& uniquenessRatio,
                   int& speckleWindowSize, int& speckleRange, double& sigmaColorBLF,
                   double& sigmaSpaceBLF, double& wls_lambda, double& wls_sigma,
                   SM_BLF_FILTER& BLFfiltering, SM_WLS_FILTER& WLSfiltering,
                   SM_MATCHING_ALG& stereo_matching);

    /**
    * Checks whether the GUI has been closed
    * @return True if the GUI has been closed, False otherwise
    *
    */
    bool isDone();

    /**
    * Checks whether the GUI has been updated
    * @return True if the GUI has been updated in the last timestep, False otherwise
    *
    */
    bool isUpdated();

    /**
    * XXXXXXXXXXXXXXX
    * @param v XXXXXXXXXXXXXXX
    *
    */
    void setUpdated(bool v);

    /**
    * XXXXXXXXXXXXXXX
    * @param v1 XXXXXXXXXXXXXXX
    * @param v2 XXXXXXXXXXXXXXX
    *
    */
    void setUpdated(bool v1, bool v2);

    /**
    * Checks whether the user asked the system to carry out the recalibration process
    * @return True if the Recalibrate button has been pressed, False otherwise
    *
    */
    bool toRecalibrate();

    /**
    * Checks whether the user wants to save the current calibration parameters
    * @return True if the Save Calibration button has been pressed, False otherwise
    *
    */
    bool toSaveCalibration();


    /**
    * XXXXXXXXXXXXXXX
    *
    */
    void convertIDToEnum();


    /**
    * XXXXXXXXXXXXXXX
    *
    */
    void convertEnumToID();


    GUI();
    ~GUI();

};

#endif // _GUI_H_
