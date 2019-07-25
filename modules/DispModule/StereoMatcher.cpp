

#include "fastBilateral.hpp"
#include "StereoMatcher.h"
#include <opencv2/opencv.hpp>

using cv::Ptr;
using cv::StereoSGBM;

Rect computeROI2(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);

/******************************************************************************/
Rect computeROI2(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);

    return r;
}




/******************************************************************************/
void StereoMatcherNew::setParameters(int minDisparity, int numberOfDisparities, int SADWindowSize,
                                  int disp12MaxDiff, int preFilterCap, int uniquenessRatio,
                                  int speckleWindowSize, int speckleRange, double sigmaColorBLF,
                                  double sigmaSpaceBLF, double wls_lambda, double wls_sigma,
                                  SM_BLF_FILTER BLFfiltering, SM_WLS_FILTER WLSfiltering,
                                  SM_MATCHING_ALG stereo_matching)
{
    this->minDisparity = minDisparity;
    this->numberOfDisparities = numberOfDisparities;
    this->SADWindowSize = SADWindowSize;
    this->disp12MaxDiff = disp12MaxDiff;
    this->preFilterCap = preFilterCap;
    this->uniquenessRatio = uniquenessRatio;
    this->speckleWindowSize = speckleWindowSize;
    this->speckleRange = speckleRange;
    this->sigmaColorBLF = sigmaColorBLF;
    this->sigmaSpaceBLF = sigmaSpaceBLF;
    this->wls_lambda = wls_lambda;
    this->wls_sigma = wls_sigma;
    this->blf_filtering = BLFfiltering;
    this->wls_filtering = WLSfiltering;
    this->stereo_matching = stereo_matching;

    std::cout << "AAAA " << this->numberOfDisparities << std::endl;
}



/******************************************************************************/
void StereoMatcherNew::compute()
{

    std::cout << "SM: rectify\n";

    this->stereo->rectifyImages();

    std::cout << "SM: match\n";


    switch(this->stereo_matching)
    {
        case SM_MATCHING_ALG::SGBM_OPENCV:
            this->matchSGBM();
            break;
        case SM_MATCHING_ALG::SGBM_CUDA:
            this->matchSGBMCUDA();
            break;
        case SM_MATCHING_ALG::LIBELAS:
            this->matchLIBELAS();
            break;
    }
}

/******************************************************************************/
void StereoMatcherNew::filterBLF(string kind = "base")
{

    // TODO finire di scrivere]

    std::cout << "SM: getDisparity\n";

    cv::Mat input = this->getDisparity(kind);

    std::cout << "SM: switch\n";

    switch(this->blf_filtering)
    {
        case SM_BLF_FILTER::BLF_ORIGINAL:

            cv_extend::bilateralFilter(input, this->disparity_blf, this->sigmaColorBLF, this->sigmaSpaceBLF);
            break;

        case SM_BLF_FILTER::BLF_CUDA:
        {
            cv::Mat grayL = this->stereo->getLRectified();
            cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

            imageGpu.upload(grayL);
            gpuDisp.upload(input);

            pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

            filtGpu.download(this->disparity_blf);

            break;
        }
        case SM_BLF_FILTER::BLF_DISABLED:
        default:

            this->disparity_blf = input.clone();
            break;
    }


}

/******************************************************************************/
void StereoMatcherNew::filterWLS(string kind = "base")
{

    cv::Mat input = this->getDisparity(kind);

    if(this->wls_filtering != SM_WLS_FILTER::WLS_DISABLED)
    {

        Ptr<StereoSGBM> sgbm =cv::StereoSGBM::create(this->minDisparity,this->numberOfDisparities,this->SADWindowSize,
                                                    8*3*this->SADWindowSize*this->SADWindowSize,
                                                    32*3*this->SADWindowSize*this->SADWindowSize,
                                                    this->disp12MaxDiff,this->preFilterCap,this->uniquenessRatio,
                                                    this->speckleWindowSize, this->speckleRange,
                                                    this->useBestDisp?StereoSGBM::MODE_HH:StereoSGBM::MODE_SGBM);

        cv::Mat left_rect = this->stereo->getLRectified();

        switch(this->wls_filtering)
        {
            case SM_WLS_FILTER::WLS_ENABLED:
            {
                wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);

                wls_filter->setLambda(this->wls_lambda);
                wls_filter->setSigmaColor(this->wls_sigma);
                wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*this->SADWindowSize));
                cv::Rect ROI = computeROI2(left_rect.size(),sgbm);

                wls_filter->filter(input,left_rect,this->disparity_wls,Mat(),ROI);
//                wls_filter->filter(outputDm,left_rect,disp_wls_2,Mat(),ROI);

                break;
            }
            case SM_WLS_FILTER::WLS_LRCHECK:

//                if(this->stereo_matching == SM_MATCHING_ALG::SGBM_OPENCV)
//                {
//                    wls_filter = createDisparityWLSFilter(sgbm);

//                    wls_filter->setLambda(wls_lambda);
//                    wls_filter->setSigmaColor(wls_sigma);

////                    std::cout << this->stereo->getRightDisparity().channels() << std::endl;
////                    std::cout << this->stereo->getRightDisparity().size << std::endl;
////                    std::cout << this->stereo->getRightDisparity().rows << std::endl;
////                    std::cout << this->stereo->getRightDisparity().cols << std::endl;

//                    wls_filter->filter(input,left_rect,this->disparity_wls,this->stereo->getRightDisparity());
//                }
//                else
//                    this->disparity_wls = input;

                break;

            default:

                break;
        }
    }
    else
        this->disparity_wls = input;

//    std::cout << "FilterWLS - End\n";

}

//void StereoMatcher::filter()
//{

//    cv::Mat disp, disp_blf;

//    disp = this->disparity();


//    if(this->blf_filtering != SM_BLF_FILTER::BLF_DISABLED)
//        this->filterBLF();

//    if(this->wls_filtering != SM_WLS_FILTER::WLS_DISABLED)
//        this->filterWLS();

//    switch(this->blf_filtering)
//    {
//        case SM_BLF_FILTER::BLF_ORIGINAL:

//            cv_extend::bilateralFilter(disp, disp_blf, this->sigmaColorBLF, this->sigmaSpaceBLF);
//            break;

//        case SM_BLF_FILTER::BLF_CUDA:

//            cv::Mat grayL = this->stereo->getLRectified();
//            cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

//            cv::cuda::GpuMat imageGpu, gpuDisp, filtGpu;

//            imageGpu.upload(grayL);
//            gpuDisp.upload(disp);

//            pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

//            filtGpu.download(disp_blf);

//            break;

//        case SM_BLF_FILTER::BLF_DISABLED:
//        default:

//            disp_blf = disp;
//            break;
//    }

//    this->setDisparity(disp_blf, "BLF");



//}

/******************************************************************************/
cv::Mat StereoMatcherNew::getDisparity(string kind="base")
{
//    for (auto & c: kind)
//        c = toupper(c);

//    std::cout << "SM: getDisparity inside\n";

    if(kind == "base")
        return this->disparity;
    else if(kind == "blf")
        return this->disparity_blf;
    else if(kind == "wls")
        return this->disparity_wls;
    else
    {
        std::cout << "!! Disparity kind " << kind <<  " not found, returning BASE disparity." << std::endl;
        return this->disparity;
    }

//    std::cout << "SM: getDisparity should not be here\n";
}


/******************************************************************************/
cv::Mat StereoMatcherNew::getDisparity16(string kind="base")
{
//    for (auto & c: kind)
//        c = toupper(c);

    if(kind == "base")
        return this->disparity16;
    else if(kind == "blf")
        return this->disparity16_blf;
    else if(kind == "wls")
        return this->disparity16_wls;
    else
    {
        std::cout << "!! Disparity16 kind " << kind <<  " not found, returning BASE disparity." << std::endl;
        return this->disparity16;
    }
}


/******************************************************************************/
void StereoMatcherNew::setAlgorithm(string name)
{

//    for (auto & c: name)
//        c = toupper(c);

    if(name == "sgbm")
        this->stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
    else if(name == "sgbm_cuda")
        this->stereo_matching = SM_MATCHING_ALG::SGBM_CUDA;
    else if(name == "libelas")
        this->stereo_matching = SM_MATCHING_ALG::LIBELAS;
    else
    {
        std::cout << "!! Stereo Matching algorithm " << name <<  " not found, defaulting to SGBM." << std::endl;
        this->stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
    }

}

//void StereoMatcher::setDisparity(cv::Mat disp)
//{

//    switch(this->algorithm)
//    {
//        case SM_MATCHING_ALG::SGBM_OPENCV:
//            this->disp_sgbm.disp = disp;
//            break;
//        case SM_MATCHING_ALG::SGBM_CUDA:
//            this->disp_cuda.disp = disp;
//            break;
//        case SM_MATCHING_ALG::LIBELAS:
//            this->disp_libelas.disp = disp;
//            break;
//    }

//}


//void StereoMatcherNew::setDisparity(cv::Mat disp, string kind="BASE")
//{

//    for (auto & c: kind)
//        c = toupper(c);

//    if(kind == "BASE")
//        this->disparity = disp;
//    else if(kind == "BLF")
//        this->disparity_blf = disp;
//    else if(kind == "WLS")
//        this->disparity_wls = disp;
//    else
//    {
//        std::cout << "!! Disparity16 kind " << kind <<  " not found, returning BASE disparity." << std::endl;
//        this->disparity = disp;
//    }

//}

//cv::Mat StereoMatcher::getDisparity()
//{
//    cv::Mat disp;

//    switch(this->algorithm)
//    {
//        case SM_MATCHING_ALG::SGBM_OPENCV:
//            return NULL;
//            break;
//        case SM_MATCHING_ALG::SGBM_CUDA:
//            getDisparityVis(this->disp_cuda.disp, disp, 2);
//            return disp;
//            break;
//        case SM_MATCHING_ALG::LIBELAS:
//        return NULL;
//            break;
//    }
//}

/******************************************************************************/
StereoMatcherNew::StereoMatcherNew(yarp::os::ResourceFinder &rf, StereoCamera * stereo)
{

    this->stereo = stereo;
    this->wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    this->initELAS(rf);
    this->initCUDAbilateralFilter();

}

//void StereoMatcherNew::initCUDAParams()
//{

//    this->cuda_params.preFilterCap = this->preFilterCap;
//    this->cuda_params.BlockSize = this->SADWindowSize;
//    this->cuda_params.P1 = 8 * this->cuda_params.BlockSize * this->cuda_params.BlockSize;
//    this->cuda_params.P2 = 32 * this->cuda_params.BlockSize * this->cuda_params.BlockSize;
//    this->cuda_params.uniquenessRatio = this->uniquenessRatio;
//    this->cuda_params.disp12MaxDiff = this->disp12MaxDiff;

//    cuda_init(&this->cuda_params);

//}

/******************************************************************************/
void StereoMatcherNew::updateCUDAParams()
{
    this->cuda_params.preFilterCap = this->preFilterCap;
    this->cuda_params.BlockSize = this->SADWindowSize;
    this->cuda_params.P1 = 8 * this->cuda_params.BlockSize * this->cuda_params.BlockSize;
    this->cuda_params.P2 = 32 * this->cuda_params.BlockSize * this->cuda_params.BlockSize;
    this->cuda_params.uniquenessRatio = this->uniquenessRatio;
    this->cuda_params.disp12MaxDiff = this->disp12MaxDiff;

    cuda_init(&this->cuda_params);

    pCudaBilFilter->setSigmaRange(sigmaColorBLF);
    pCudaBilFilter->setNumDisparities(this->numberOfDisparities);

}

/******************************************************************************/
void StereoMatcherNew::initCUDAbilateralFilter()
{
    int radius = 7;
    int iters = 2;

    this->pCudaBilFilter = cuda::createDisparityBilateralFilter(this->numberOfDisparities, radius, iters);
    this->pCudaBilFilter->setSigmaRange(sigmaColorBLF);

}

/******************************************************************************/
void StereoMatcherNew::initELAS(yarp::os::ResourceFinder &rf)
{

    string elas_string = rf.check("elas_setting",Value("ROBOTICS")).asString();

    double disp_scaling_factor = rf.check("disp_scaling_factor",Value(1.0)).asDouble();

    this->elaswrap = new elasWrapper(disp_scaling_factor, elas_string);


    if (rf.check("elas_subsampling"))
        elaswrap->set_subsampling(true);

    if (rf.check("elas_add_corners"))
        elaswrap->set_add_corners(true);


    elaswrap->set_ipol_gap_width(40);
    if (rf.check("elas_ipol_gap_width"))
        elaswrap->set_ipol_gap_width(rf.find("elas_ipol_gap_width").asInt());


    if (rf.check("elas_support_threshold"))
        elaswrap->set_support_threshold(rf.find("elas_support_threshold").asDouble());

    if(rf.check("elas_gamma"))
        elaswrap->set_gamma(rf.find("elas_gamma").asDouble());

    if (rf.check("elas_sradius"))
        elaswrap->set_sradius(rf.find("elas_sradius").asDouble());

    if (rf.check("elas_match_texture"))
        elaswrap->set_match_texture(rf.find("elas_match_texture").asInt());

    if (rf.check("elas_filter_median"))
        elaswrap->set_filter_median(rf.find("elas_filter_median").asBool());

    if (rf.check("elas_filter_adaptive_mean"))
        elaswrap->set_filter_adaptive_mean(rf.find("elas_filter_adaptive_mean").asBool());

    cout << endl << "ELAS parameters:" << endl << endl;

    cout << "disp_scaling_factor: " << disp_scaling_factor << endl;

    cout << "setting: " << elas_string << endl;

    cout << "postprocess_only_left: " << elaswrap->get_postprocess_only_left() << endl;
    cout << "subsampling: " << elaswrap->get_subsampling() << endl;

    cout << "add_corners: " << elaswrap->get_add_corners() << endl;

    cout << "ipol_gap_width: " << elaswrap->get_ipol_gap_width() << endl;

    cout << "support_threshold: " << elaswrap->get_support_threshold() << endl;
    cout << "gamma: " << elaswrap->get_gamma() << endl;
    cout << "sradius: " << elaswrap->get_sradius() << endl;

    cout << "match_texture: " << elaswrap->get_match_texture() << endl;

    cout << "filter_median: " << elaswrap->get_filter_median() << endl;
    cout << "filter_adaptive_mean: " << elaswrap->get_filter_adaptive_mean() << endl;

    cout << endl;
}


/******************************************************************************/
void StereoMatcherNew::matchLIBELAS()
{

    cv::Mat outputDm;

    bool success = elaswrap->compute_disparity(this->stereo->getLRectified(), this->stereo->getRRectified(), this->disparity, this->numberOfDisparities);

    if (success)
        outputDm = this->disparity * (255.0 / this->numberOfDisparities);

    outputDm.convertTo(outputDm, CV_16SC1, 16.0);

//    this->disp_libelas.disp16 = outputDm;
//    this->disp_sgbm.disp = this->stereo->remapDisparity(map);

    this->disparity16 = outputDm;
    this->disparity = this->stereo->remapDisparity(outputDm);

}

/******************************************************************************/
void StereoMatcherNew::matchSGBM()
{

    cv::Mat map, disp;

    Ptr<StereoSGBM> sgbm=cv::StereoSGBM::create(this->minDisparity,this->numberOfDisparities,this->SADWindowSize,
                                                8*3*this->SADWindowSize*this->SADWindowSize,
                                                32*3*this->SADWindowSize*this->SADWindowSize,
                                                this->disp12MaxDiff,this->preFilterCap,this->uniquenessRatio,
                                                this->speckleWindowSize, this->speckleRange,
                                                this->useBestDisp?StereoSGBM::MODE_HH:StereoSGBM::MODE_SGBM);


    sgbm->compute(this->stereo->getLRectified(), this->stereo->getRRectified(), disp);


//    this->disp_sgbm.disp16 = disp;
    this->disparity16 = disp;
    disp.convertTo(map,CV_32FC1,255/(this->numberOfDisparities*16.));


//    this->disp_sgbm.disp = this->stereo->remapDisparity(map);
//    this->disparity = this->stereo->remapDisparity(map);
    this->disparity = map;

}

/******************************************************************************/
void StereoMatcherNew::matchSGBMCUDA()
{

    cv::Mat outputDm;

    this->stereo->rectifyImages();

    cv::Mat grayL = this->stereo->getLRectified();
    cv::Mat grayR = this->stereo->getRRectified();

    cv::cvtColor(grayL, grayL, CV_BGR2GRAY);
    cv::cvtColor(grayR, grayR, CV_BGR2GRAY);

    outputDm = zy_remap(grayL, grayR);

//    this->disp_cuda.disp = outputDm;
//    this->disp_cuda.disp16 = outputDm;
    this->disparity = outputDm;
    this->disparity16 = outputDm;
}

/******************************************************************************/
StereoMatcherNew::~StereoMatcherNew()
{

}
