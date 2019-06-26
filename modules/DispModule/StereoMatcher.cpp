

#include "StereoMatcher.h"

void StereoMatcher::setParameters()
{

}

void StereoMatcher::compute(cv::Mat left, cv::Mat right)
{
    switch(this->algorithm)
}

void StereoMatcher::filter()
{

}

void StereoMatcher::setAlgorithm(string n)
{

    for (auto & c: n)
        c = toupper(c);

    if(n == "SGBM" || n == "SGBM_CUDA" || n == "LIBELAS")
        this->algorithm = n;
//    else
//        error
}

cv::Mat StereoMatcher::getDisparity()
{
    return disp;
}

cv::Mat StereoMatcher::getDisparity16()
{
    return disp16;
}



StereoMatcher();
