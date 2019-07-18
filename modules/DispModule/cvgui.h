#ifndef _GUI_H_
#define _GUI_H_

#include "StereoMatcher_enums.h"
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//enum SM_MATCHING_ALG {
//    SGBM_OPENCV = 0,
//    SGBM_CUDA,
//    LIBELAS
//};

//enum SM_BLF_FILTER {
//    BLF_ORIGINAL = 0,
//    BLF_CUDA,
//    BLF_DISABLED
//};

//enum SM_WLS_FILTER {
//    WLS_DISABLED = 0,
//    WLS_ENABLED,
//    WLS_LRCHECK
//};


//enum STEREO_VISION {

//    BLF_DISABLED = 0,
//    BLF_ORIGINAL,
//    BLF_CUDA,
//    WLS_DISABLED,
//    WLS_ENABLED,
//    WLS_LRCHECK,
//    SGBM_OPENCV,
//    SGBM_CUDA,
//    LIBELAS
//};


typedef struct {

    int * uniquenessRatio;
    int * speckleWindowSize;
    int * speckleRange;
    int * numberOfDisparities;
    int * SADWindowSize;
    int * minDisparity;
    int * preFilterCap;
    int * disp12MaxDiff;

    double * sigmaColorBLF;
    double * sigmaSpaceBLF;
    double * wls_lambda;
    double * wls_sigma;

    SM_BLF_FILTER BLFfiltering;
    SM_WLS_FILTER WLSfiltering;
    SM_MATCHING_ALG stereo_matching;

} Params;

class GUI
{

private:

    bool done;
    int val;
    bool updated;
    bool recalibrate;

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


    void killGUI();
    GUI();
    ~GUI();
    int initializeGUI();
    int initializeGUI(int minDisparity, int numberOfDisparities, int SADWindowSize,
                                             int disp12MaxDiff, int preFilterCap, int uniquenessRatio,
                                             int speckleWindowSize, int speckleRange, double sigmaColorBLF,
                                             double sigmaSpaceBLF, double wls_lambda, double wls_sigma,
                                             SM_BLF_FILTER BLFfiltering, SM_WLS_FILTER WLSfiltering,
                                             SM_MATCHING_ALG stereo_matching);
    void updateGUI();
    void setVal(int);
    int getVal();
    void getParams(int& minDisparity, int& numberOfDisparities, int& SADWindowSize,
                   int& disp12MaxDiff, int& preFilterCap, int& uniquenessRatio,
                   int& speckleWindowSize, int& speckleRange, double& sigmaColorBLF,
                   double& sigmaSpaceBLF, double& wls_lambda, double& wls_sigma,
                   SM_BLF_FILTER& BLFfiltering, SM_WLS_FILTER& WLSfiltering,
                   SM_MATCHING_ALG& stereo_matching);
    bool isDone();
    bool isUpdated();
    void setUpdated(bool);
    void setUpdated(bool, bool);

    bool toRecalibrate();

    void convertIDToEnum();
    void convertEnumToID();
};

#endif // _GUI_H_
