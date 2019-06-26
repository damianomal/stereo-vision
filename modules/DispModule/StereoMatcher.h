

#include "opencv2/ximgproc/disparity_filter.hpp"
#include "fastBilateral.hpp"
#include "common.h"


class StereoMatcher {

private:

    cv::Mat disp, disp16;
//    cv::Mat left_im, right_im;



public:

    void setParameters();
    void compute(cv::Mat left, cv::Mat right);
    void setAlgorithm(string n);
    void filter();

    cv::Mat getDisparity();
    cv::Mat getDisparity16();



    StereoMatcher();
    ~StereoMatcher();



}
