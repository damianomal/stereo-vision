
#ifndef _STEREO_MATCHER_NEW_H_
#define _STEREO_MATCHER_NEW_H_

#include <yarp/os/all.h>

#include <iCub/stereoVision/elasWrapper.h>
#include <iCub/stereoVision/stereoCamera.h>

#include "common.h"
#include "sgbm_cuda_header.h"

#include "opencv2/ximgproc/disparity_filter.hpp"

#include "StereoMatcher_enums.h"


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

typedef struct {

    cv::Mat disp;
    cv::Mat disp16;

    cv::Mat disp_blf;
    cv::Mat disp16_blf;

    cv::Mat disp_wls;
    cv::Mat disp16_wls;

} Disparities;


class StereoMatcherNew {

private:

    StereoCamera * stereo;

//    cv::Mat disp, disp16;

    cv::Mat disparity;
    cv::Mat disparity16;

    cv::Mat disparity_blf;
    cv::Mat disparity16_blf;

    cv::Mat disparity_wls;
    cv::Mat disparity16_wls;

//    SGM_PARAMS cuda_params;

    SM_MATCHING_ALG stereo_matching;
    SM_BLF_FILTER blf_filtering;
    SM_WLS_FILTER wls_filtering;

    bool disp_wls_available, disp_blf_available;
    Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

    cv::cuda::GpuMat imageGpu, gpuDisp, filtGpu;
    Ptr<cuda::DisparityBilateralFilter> pCudaBilFilter;
    SGM_PARAMS cuda_params, params_right;

    bool useBestDisp;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int numberOfDisparities;
    int SADWindowSize;
    int minDisparity;
    int preFilterCap;
    int disp12MaxDiff;
    bool doSFM;
    bool calibUpdated;
    bool debugWindow;
    double sigmaColorBLF;
    double sigmaSpaceBLF;
    double wls_lambda;
    double wls_sigma;


//    Disparities disp_sgbm, disp_cuda, disp_libelas, disparities;

    // different matching algorithms
    void matchSGBM();
    void matchLIBELAS();
    void matchSGBMCUDA();


    //
    void setDisparity(cv::Mat disp, string kind);

    elasWrapper* elaswrap;




public:

    void initELAS(yarp::os::ResourceFinder &rf);

    void setParameters(int minDisparity, int numberOfDisparities, int SADWindowSize,
                       int disp12MaxDiff, int preFilterCap, int uniquenessRatio,
                       int speckleWindowSize, int speckleRange, double sigmaColorBLF,
                       double sigmaSpaceBLF, double wls_lambda, double wls_sigma,
                       SM_BLF_FILTER BLFfiltering, SM_WLS_FILTER WLSfiltering,
                       SM_MATCHING_ALG stereo_matching);
    void compute();
    void setAlgorithm(string n);
    void filter();

    cv::Mat getDisparity(string kind);
    cv::Mat getDisparity16(string kind);


    void initCUDAParams();
    void updateCUDAParams();

    void filterBLF(string kind);
    void filterWLS(string kind);



    StereoMatcherNew(yarp::os::ResourceFinder &rf, StereoCamera * stereo);
    ~StereoMatcherNew();



};

#endif // _STEREO_MATCHER_NEW
