
#ifndef _STEREO_MATCHER_NEW_H_
#define _STEREO_MATCHER_NEW_H_

#include <yarp/os/all.h>

#include <iCub/stereoVision/elasWrapper.h>
#include <iCub/stereoVision/stereoCamera.h>

#include "common.h"
#include "sgbm_cuda_header.h"

#include "opencv2/ximgproc/disparity_filter.hpp"

//

enum SM_MATCHING_ALG {
    SGBM_OPENCV = 0,
    SGBM_CUDA,
    LIBELAS
};

enum SM_BLF_FILTER {
    BLF_ORIGINAL = 0,
    BLF_CUDA,
    BLF_DISABLED
};

enum SM_WLS_FILTER {
    WLS_DISABLED = 0,
    WLS_ENABLED,
    WLS_LRCHECK
};

//

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

//

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

    // base disparity

    cv::Mat disparity;
    cv::Mat disparity16;

    // disparity after blf filtering

    cv::Mat disparity_blf;
    cv::Mat disparity16_blf;

    // disparity after wls filtering

    cv::Mat disparity_wls;
    cv::Mat disparity16_wls;

    // selections for the different methods
    // used to carry out the processing

    SM_MATCHING_ALG stereo_matching;
    SM_BLF_FILTER blf_filtering;
    SM_WLS_FILTER wls_filtering;

    //

    bool disp_wls_available, disp_blf_available;
    Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

    //

    cv::cuda::GpuMat imageGpu, gpuDisp, filtGpu;
    Ptr<cuda::DisparityBilateralFilter> pCudaBilFilter;
    SGM_PARAMS cuda_params, params_right;

    // stereo matching parameters

    bool useBestDisp;

//    int uniquenessRatio;
//    int speckleWindowSize;
//    int speckleRange;
//    int numberOfDisparities;
//    int SADWindowSize;
//    int minDisparity;
//    int preFilterCap;
//    int disp12MaxDiff;
//    double sigmaColorBLF;
//    double sigmaSpaceBLF;
//    double wls_lambda;
//    double wls_sigma;

    Params stereo_parameters;

    // different matching algorithms

    void matchSGBM();
    void matchLIBELAS();
    void matchSGBMCUDA();

    // object wrapping the LibElas calls

    elasWrapper* elaswrap;

public:

    /**
    * Initialize the LibElas stereo matching wrapper object
    * @param rf The ResourceFinder object
    *
    */
    void initELAS(yarp::os::ResourceFinder &rf);

    /**
    * Initializes the CUDA bilateral filter object
    *
    */
    void initCUDAbilateralFilter();



    void setParameters(int minDisparity, int numberOfDisparities, int SADWindowSize,
                       int disp12MaxDiff, int preFilterCap, int uniquenessRatio,
                       int speckleWindowSize, int speckleRange, double sigmaColorBLF,
                       double sigmaSpaceBLF, double wls_lambda, double wls_sigma,
                       SM_BLF_FILTER BLFfiltering, SM_WLS_FILTER WLSfiltering,
                       SM_MATCHING_ALG stereo_matching);

    /**
    * Computes the disparity map on the basis of the chosen algorith
    *
    */
    void compute();

    /**
    * Set the stereo matching method to use
    * @param name The name of the algorithm to select
    *
    */
    void setAlgorithm(string name);

    /**
    * Returns the specified disparity map, between the ones available
    * @param kind The type of disparity map to return
    * @return The chosen disparity map
    *
    */
    cv::Mat getDisparity(string kind);

    /**
    * Returns the specified disparity map, between the ones available
    * @param kind The type of disparity map to return
    * @return The chosen disparity map
    *
    */
    cv::Mat getDisparity16(string kind);

    /**
    * Updates the parameters of the CUDA version of the SGBM algorithm
    *
    */
    void updateCUDAParams();

    /**
    * Filters the chosen disparity map by means of the bilateral filter implementation which has been previously selected
    * @param kind The type of disparity map to filter
    *
    */
    void filterBLF(string kind);

    /**
    * Filters the chosen disparity map by means of a Weighted Least Squares filter
    * @param kind The type of disparity map to filter
    *
    */
    void filterWLS(string kind);

    /**
    * Costructor for the StereoMatcherNew object
    * @param rf The ResourceFinder object
    * @param stereo The pointer to the stereo camera object used in the system
    *
    */
    StereoMatcherNew(yarp::os::ResourceFinder &rf, StereoCamera * stereo);
    ~StereoMatcherNew();



};

#endif // _STEREO_MATCHER_NEW
