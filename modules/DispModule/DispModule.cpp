/* 
 * Copyright (C) 2015 RobotCub Consortium
 * Author: Sean Ryan Fanello, Giulia Pasquale
 * email:   sean.fanello@iit.it giulia.pasquale@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <cmath>
#include <algorithm>
#include <yarp/cv/Cv.h>
#include "DispModule.h"

// --- DEBUG
//#include <chrono>
//#define PROF_S {start = std::chrono::high_resolution_clock::now();}
//#define PROF_E {stop = std::chrono::high_resolution_clock::now();}
//#define PROF_D(N) {duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); this->debug_timings[N] += duration.count();}

#define PROF_S
#define PROF_E
#define PROF_D


char const *debug_strings[] = {
    "updateViaKinematics",
    "updateViaGazeCtrl",
    "getCameraHGazeCtrlx2",
    "gui.recalibrate",
    "doSFM",
    "gui.isUpdated",
    "computeDisparity",
    "bilateralFilter",
    "wlsFilter",
    "ouputDisparity",
    "outputDepth"
};

using namespace yarp::cv;

bool DispModule::configure(ResourceFinder & rf)
{

//    if(rf.check("640"))
//        rf.setDefaultConfigFile("icubEyes_640x480.ini");

    string name=rf.check("name",Value("SFM")).asString();
    string robot=rf.check("robot",Value("icub")).asString();
    string left=rf.check("leftPort",Value("/left:i")).asString();
    string right=rf.check("rightPort",Value("/right:i")).asString();
    string SFMFile=rf.check("SFMFile",Value("SFM.ini")).asString();

    string sname;
    sname="/"+name;
    left=sname+left;
    right=sname+right;

    string outDispName=rf.check("outDispPort",Value("/disp:o")).asString();
    string outDepthName=rf.check("outNamePort",Value("/depth:o")).asString();

//    string outLeftRectImgPortName=rf.check("outLeftRectImgPort",Value("/rect_left:o")).asString();
//    string outRightRectImgPortName=rf.check("outRightRectImgPort",Value("/rect_right:o")).asString();

    ResourceFinder localCalibration;
    localCalibration.setDefaultContext("cameraCalibration");
    localCalibration.setDefaultConfigFile(SFMFile.c_str());
    localCalibration.configure(0,NULL);

    this->camCalibFile=localCalibration.getHomeContextPath();
    this->camCalibFile+="/"+SFMFile;

    outDepthName=sname+outDepthName;
    outDispName=sname+outDispName;

    string rpc_name=sname+"/rpc";
//    string world_name=sname+rf.check("outWorldPort",Value("/world")).asString();

    int calib=rf.check("useCalibrated",Value(1)).asInt();
    bool useCalibrated=(calib!=0);

    leftImgPort.open(left);
    rightImgPort.open(right);
    outDepth.open(outDepthName);
    outDisp.open(outDispName);
    handlerPort.open(rpc_name);
    attach(handlerPort);

    this->stereo = new StereoCamera(true);

    Mat KL, KR, DistL, DistR;

    loadIntrinsics(rf,KL,KR,DistL,DistR);
    loadExtrinsics(localCalibration,R0,T0,eyes0);
    eyes.resize(eyes0.length(),0.0);

    stereo->setIntrinsics(KL,KR,DistL,DistR);

    this->initializeStereoParams();


    if (rf.check("sgbm"))
        this->stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
    else if(rf.check("sgbm_cuda"))
        this->stereo_matching = SM_MATCHING_ALG::SGBM_CUDA;
    else if(rf.check("libelas"))
    {
        this->stereo_matching = SM_MATCHING_ALG::LIBELAS;
        this->stereo->initELAS(rf);
    }
    else
        this->stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;

    if (rf.check("blf"))
        this->BLFfiltering = SM_BLF_FILTER::BLF_ORIGINAL;
    else if(rf.check("blf_cuda"))
        this->BLFfiltering = SM_BLF_FILTER::BLF_CUDA;
    else
        this->BLFfiltering = SM_BLF_FILTER::BLF_DISABLED;

    if (rf.check("wls"))
        this->WLSfiltering = SM_WLS_FILTER::WLS_ENABLED;
    else if(rf.check("wls_lr"))
        this->WLSfiltering = SM_WLS_FILTER::WLS_LRCHECK;
    else
        this->WLSfiltering = SM_WLS_FILTER::WLS_DISABLED;


    this->debugWindow = !rf.check("debug");

    if(this->debugWindow)
    {
        this->gui.initializeGUI(this->minDisparity, this->numberOfDisparities, this->SADWindowSize,
                            this->disp12MaxDiff, this->preFilterCap, this->uniquenessRatio,
                            this->speckleWindowSize, this->speckleRange, this->sigmaColorBLF,
                            this->sigmaSpaceBLF, this->wls_lambda, this->wls_sigma,
                            this->BLFfiltering, this->WLSfiltering, this->stereo_matching);
    }


    this->HL_root=Mat::zeros(4,4,CV_64F);
    this->HR_root=Mat::zeros(4,4,CV_64F);

    if (useCalibrated)
    {
        Mat KL=this->stereo->getKleft();
        Mat KR=this->stereo->getKright();
        Mat zeroDist=Mat::zeros(1,8,CV_64FC1);
        this->stereo->setIntrinsics(KL,KR,zeroDist,zeroDist);
    }

    init=true;
    numberOfTrials=0;

#ifdef USING_GPU
    utils=new Utilities();
    utils->initSIFT_GPU();
#endif

    Property optionHead;
    optionHead.put("device","remote_controlboard");
    optionHead.put("remote","/"+robot+"/head");
    optionHead.put("local",sname+"/headClient");

    if (headCtrl.open(optionHead))
    {
        headCtrl.view(iencs);
        iencs->getAxes(&nHeadAxes);
    }
    else if (this->usePorts)
    {
//        string fakeEyes=rf.check("fakeEyesPort",Value("/fakeEyes:i")).asString();
//        fakeEkesPort.open(fakeEyes);
//        =rf.check("rightPort",Value("/right:i")).asString();
    }
    else
    {
        cout<<"Devices not available"<<endl;
        return false;
    }

    Property optionGaze;
    optionGaze.put("device","gazecontrollerclient");
    optionGaze.put("remote","/iKinGazeCtrl");
    optionGaze.put("local",sname+"/gazeClient");
    if (gazeCtrl.open(optionGaze))
        gazeCtrl.view(igaze);
    else
    {
        cout<<"Devices not available"<<endl;
        headCtrl.close();
        return false;
    }

    if (!R0.empty() && !T0.empty())
    {
        stereo->setRotation(R0,0);
        stereo->setTranslation(T0,0);
    }
    else
    {
        cout << "No local calibration file found in " <<  camCalibFile << " ... Using Kinematics and Running SFM once." << endl;
        updateViaGazeCtrl(true);
        R0=this->stereo->getRotation();
        T0=this->stereo->getTranslation();
    }

    doSFM=false;
    updateViaGazeCtrl(false);

    this->debug_count = 0;

    this->debug_num_timings = 11;

    debug_timings = new int[debug_num_timings];

    for(int i = 0; i < debug_num_timings; i++)
        this->debug_timings[i] = 0;


//    this->cuda_params.preFilterCap = this->preFilterCap;
//    this->cuda_params.BlockSize = this->SADWindowSize;
//    this->cuda_params.P1 = 8 * this->cuda_params.BlockSize * this->cuda_params.BlockSize;
//    this->cuda_params.P2 = 32 * this->cuda_params.BlockSize * this->cuda_params.BlockSize;
//    this->cuda_params.uniquenessRatio = this->uniquenessRatio;
//    this->cuda_params.disp12MaxDiff = this->disp12MaxDiff;

//    cuda_init(&this->cuda_params);

//    this->params_right.preFilterCap = this->preFilterCap;
//    this->params_right.BlockSize = this->SADWindowSize;
//    this->params_right.P1 = 8 * this->params_right.BlockSize * this->params_right.BlockSize;
//    this->params_right.P2 = 32 * this->params_right.BlockSize * this->params_right.BlockSize;
//    this->params_right.uniquenessRatio = 0;
//    this->params_right.disp12MaxDiff = 1000000;

//    int radius = 7;
//    int iters = 2;

//    pCudaBilFilter = cuda::createDisparityBilateralFilter(this->numberOfDisparities, radius, iters);
//    pCudaBilFilter->setSigmaRange(sigmaColorBLF);

    this->matcher = new StereoMatcherNew(rf, this->stereo);
    this->matcher->initCUDAParams();

    return true;

}

cv::Mat DispModule::depthFromDisparity(Mat disparity, Mat Q, Mat R)
{

    cv::Mat repr;

    repr = disparity.clone();
    repr.convertTo(repr, CV_32FC1, 1./16);

    reprojectImageTo3D(repr, repr, Q, true);

    // TODO: missing rotation given RLRect

    extractChannel(repr, repr, 2);

    return repr;

}


cv::Mat DispModule::depthFromDisparity_alt(Mat disparity, Mat Q, Mat R)
{

    cv::Mat depth;

    depth = disparity.clone();

    depth.convertTo(depth, CV_32FC1, 1./16.);

    float r02 = float(R.at<double>(0,2));
    float r12 = float(R.at<double>(1,2));
    float r22 = float(R.at<double>(2,2));

    float q00 = float(Q.at<double>(0,0));
    float q03 = float(Q.at<double>(0,3));
    float q11 = float(Q.at<double>(1,1));
    float q13 = float(Q.at<double>(1,3));
    float q23 = float(Q.at<double>(2,3));
    float q32 = float(Q.at<double>(3,2));
    float q33 = float(Q.at<double>(3,3));

    for (int u = 0; u < depth.rows; u++)
        for (int v = 0; v < depth.cols; v++)
            depth.at<float>(u,v) = (r02*(float(u)*q00+q03)+r12*(float(v)*q11+q13)+r22*q23)/(depth.at<float>(u,v)*q32+q33);

    return depth;

}



void DispModule::initializeStereoParams()
{
    this->useBestDisp=true;
    this->uniquenessRatio=15;
    this->speckleWindowSize=50;
    this->speckleRange=16;
    this->SADWindowSize=7;
    this->minDisparity=0;
    this->preFilterCap=63;
    this->disp12MaxDiff=0;

    this->numberOfDisparities = 96;

    this->sigmaColorBLF = 10.0;
    this->sigmaSpaceBLF = 10.0;

    this->wls_lambda = 8000.;
    this->wls_sigma = 1.5;

}

/******************************************************************************/
void DispModule::updateViaKinematics(const yarp::sig::Vector& deyes)
{
    double dpan=CTRL_DEG2RAD*deyes[1];
    double dver=CTRL_DEG2RAD*deyes[2];

    yarp::sig::Vector rot_l_pan(4,0.0);
    rot_l_pan[1]=1.0;
    rot_l_pan[3]=dpan+dver/2.0;
    Matrix L1=axis2dcm(rot_l_pan);

    yarp::sig::Vector rot_r_pan(4,0.0);
    rot_r_pan[1]=1.0;
    rot_r_pan[3]=dpan-dver/2.0;
    Matrix R1=axis2dcm(rot_r_pan);

    Mat RT0=buildRotTras(R0,T0);
    Matrix H0; convert(RT0,H0);
    Matrix H=SE3inv(R1)*H0*L1;

    Mat R=Mat::zeros(3,3,CV_64F);
    Mat T=Mat::zeros(3,1,CV_64F);

    for (int i=0; i<R.rows; i++)
        for(int j=0; j<R.cols; j++)
            R.at<double>(i,j)=H(i,j);

    for (int i=0; i<T.rows; i++)
        T.at<double>(i,0)=H(i,3);

    this->stereo->setRotation(R,0);
    this->stereo->setTranslation(T,0);
}


/******************************************************************************/
void DispModule::updateViaGazeCtrl(const bool update)
{
    Matrix L1=getCameraHGazeCtrl(LEFT);
    Matrix R1=getCameraHGazeCtrl(RIGHT);

    Matrix RT=SE3inv(R1)*L1;

    Mat R=Mat::zeros(3,3,CV_64F);
    Mat T=Mat::zeros(3,1,CV_64F);

    for (int i=0; i<R.rows; i++)
        for(int j=0; j<R.cols; j++)
            R.at<double>(i,j)=RT(i,j);

    for (int i=0; i<T.rows; i++)
        T.at<double>(i,0)=RT(i,3);

    if (update)
    {
        stereo->setRotation(R,0);
        stereo->setTranslation(T,0);
    }
    else
        stereo->setExpectedPosition(R,T);
}


/******************************************************************************/
bool DispModule::interruptModule()
{
    leftImgPort.interrupt();
    rightImgPort.interrupt();
    outDisp.interrupt();
    outDepth.interrupt();
    handlerPort.interrupt();

    return true;
}


/******************************************************************************/
bool DispModule::close()
{
    leftImgPort.close();
    rightImgPort.close();
    outDisp.close();
    outDepth.close();
    handlerPort.close();

    headCtrl.close();
    gazeCtrl.close();

#ifdef USING_GPU
    delete utils;
#endif

    delete stereo;
    delete &gui;

    return true;
}

//Rect computeROI2(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);

//Rect computeROI2(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
//{
//    int min_disparity = matcher_instance->getMinDisparity();
//    int num_disparities = matcher_instance->getNumDisparities();
//    int block_size = matcher_instance->getBlockSize();

//    int bs2 = block_size/2;
//    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

//    int xmin = maxD + bs2;
//    int xmax = src_sz.width + minD - bs2;
//    int ymin = bs2;
//    int ymax = src_sz.height - bs2;

//    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);

//    return r;
//}


/******************************************************************************/


void DispModule::recalibrate()
{

#ifdef USING_GPU
        utils->extractMatch_GPU(leftMat,rightMat);
        vector<Point2f> leftM,rightM;
        utils->getMatches(leftM,rightM);
        mutexDisp.lock();
        this->stereo->setMatches(leftM,rightM);
#else

        mutexDisp.lock();
        stereo->findMatch(false);
#endif
        stereo->estimateEssential();
        bool ok=stereo->essentialDecomposition();
        mutexDisp.unlock();

        if (ok)
        {
            calibUpdated=true;
            doSFM=false;
            calibEndEvent.signal();

            R0=stereo->getRotation();
            T0=stereo->getTranslation();
            eyes0=eyes;

            std::cout << "Calibration Successful!" << std::endl;

        }
        else
        {
            if (++numberOfTrials>5)
            {
                calibUpdated=false;
                doSFM=false;
                calibEndEvent.signal();

                std::cout << "Calibration failed after 5 trials.. Please show a non planar scene." << std::endl;

            }
        }

}

void DispModule::handleGuiUpdate()
{

    if(this->gui.toRecalibrate())
    {
        std::cout << "Updating..." << std::endl;

        mutexRecalibration.lock();
        numberOfTrials=0;
        doSFM=true;
        mutexRecalibration.unlock();

        this->gui.setUpdated(false, false);
    }
    else
    {
//        this->cuda_params.preFilterCap = this->preFilterCap;
//        this->cuda_params.BlockSize = this->SADWindowSize;
//        this->cuda_params.uniquenessRatio = this->uniquenessRatio;
//        this->cuda_params.disp12MaxDiff = this->disp12MaxDiff;

//        cuda_init(&this->cuda_params);
//        pCudaBilFilter->setSigmaRange(sigmaColorBLF);
//        pCudaBilFilter->setNumDisparities(this->numberOfDisparities);

        this->matcher->updateCUDAParams();
        this->gui.setUpdated(false);

    }

}

bool DispModule::updateModule()
{
    ImageOf<PixelRgb> *yarp_imgL=leftImgPort.read(true);
    ImageOf<PixelRgb> *yarp_imgR=rightImgPort.read(true);

//    auto start = std::chrono::high_resolution_clock::now();
//    auto stop = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << std::setprecision(2) << std::fixed;

    Stamp stamp_left, stamp_right;
    leftImgPort.getEnvelope(stamp_left);
    rightImgPort.getEnvelope(stamp_right);

    if ((yarp_imgL==NULL) || (yarp_imgR==NULL))
        return true;

    // read encoders
    iencs->getEncoder(nHeadAxes-3,&eyes[0]);
    iencs->getEncoder(nHeadAxes-2,&eyes[1]);
    iencs->getEncoder(nHeadAxes-1,&eyes[2]);

//    PROF_S

    updateViaKinematics(eyes-eyes0);

//    PROF_E
//    PROF_D(0)

//    PROF_S

    updateViaGazeCtrl(false);

//    PROF_E
//    PROF_D(1)

    leftMat=toCvMat(*yarp_imgL);
    rightMat=toCvMat(*yarp_imgR);

    //DEBUG

    getCameraHGazeCtrl(LEFT);
    getCameraHGazeCtrl(RIGHT);

//    PROF_S

    this->stereo->setImages(leftMat,rightMat);

//    PROF_E
//    PROF_D(2)
//    PROF_S

    if(this->gui.isUpdated())
        this->handleGuiUpdate();

//    if(this->gui.isUpdated() && this->gui.toRecalibrate())
//    {
//        std::cout << "Updating..." << std::endl;

//        mutexRecalibration.lock();
//        numberOfTrials=0;
//        doSFM=true;
//        mutexRecalibration.unlock();

//        this->gui.setUpdated(false, false);
//    }

//    PROF_E
//    PROF_D(3)
//    PROF_S

    mutexRecalibration.lock();



//    if (doSFM)
//    {
//#ifdef USING_GPU
//        utils->extractMatch_GPU(leftMat,rightMat);
//        vector<Point2f> leftM,rightM;
//        utils->getMatches(leftM,rightM);
//        mutexDisp.lock();
//        this->stereo->setMatches(leftM,rightM);
//#else

//        mutexDisp.lock();
//        this->stereo->findMatch(false);
//#endif
//        this->stereo->estimateEssential();
//        bool ok=this->stereo->essentialDecomposition();
//        mutexDisp.unlock();

//        if (ok)
//        {
//            calibUpdated=true;
//            doSFM=false;
//            calibEndEvent.signal();

//            R0=this->stereo->getRotation();
//            T0=this->stereo->getTranslation();
//            eyes0=eyes;

//            std::cout << "Calibration Successful!" << std::endl;

//        }
//        else
//        {
//            if (++numberOfTrials>5)
//            {
//                calibUpdated=false;
//                doSFM=false;
//                calibEndEvent.signal();

//                std::cout << "Calibration failed after 5 trials.. Please show a non planar scene." << std::endl;

//            }
//        }
//    }

    if(doSFM)
        this->recalibrate();

    mutexRecalibration.unlock();

//    PROF_E
//    PROF_D(4)

//    PROF_S

//    if(this->gui.isUpdated())
//    {

//        this->gui.getParams(this->minDisparity, this->numberOfDisparities, this->SADWindowSize,
//                            this->disp12MaxDiff, this->preFilterCap, this->uniquenessRatio,
//                            this->speckleWindowSize, this->speckleRange, this->sigmaColorBLF, this->sigmaSpaceBLF,
//                            this->wls_lambda, this->wls_sigma, this->BLFfiltering, this->WLSfiltering, this->stereo_matching);

//        this->cuda_params.preFilterCap = this->preFilterCap;
//        this->cuda_params.BlockSize = this->SADWindowSize;
//        this->cuda_params.uniquenessRatio = this->uniquenessRatio;
//        this->cuda_params.disp12MaxDiff = this->disp12MaxDiff;

//        cuda_init(&this->cuda_params);
//        pCudaBilFilter->setSigmaRange(sigmaColorBLF);
//        pCudaBilFilter->setNumDisparities(this->numberOfDisparities);

//        this->gui.setUpdated(false);
//    }

//    PROF_E
//    PROF_D(5)

//    cv::Mat grayL, grayR;

//    PROF_S

    mutexDisp.lock();

//    switch(this->stereo_matching)
//    {
//        case STEREO_VISION::SGBM_OPENCV:
//        case STEREO_VISION::LIBELAS:

//            this->stereo->computeDisparity(this->useBestDisp,this->uniquenessRatio,this->speckleWindowSize,
//                        this->speckleRange,this->numberOfDisparities,this->SADWindowSize,
//                        this->minDisparity,this->preFilterCap,this->disp12MaxDiff);

//            outputDm = stereo->getDisparity();

//            if(this->WLSfiltering == STEREO_VISION::WLS_LRCHECK)
//                this->stereo->computeRightDisparity(this->useBestDisp,this->uniquenessRatio,this->speckleWindowSize,
//                                                    this->speckleRange,this->numberOfDisparities,this->SADWindowSize,
//                                                    this->minDisparity,this->preFilterCap,this->disp12MaxDiff);

//            break;

//        case STEREO_VISION::SGBM_CUDA:

//            this->stereo->rectifyImages();

//            grayL = this->stereo->getLRectified();
//            grayR = this->stereo->getRRectified();

//            cv::cvtColor(grayL, grayL, CV_BGR2GRAY);
//            cv::cvtColor(grayR, grayR, CV_BGR2GRAY);

//            outputDm = zy_remap(grayL, grayR);

//            // DEBUG: NOT WORKING

//            if(this->WLSfiltering == STEREO_VISION::WLS_LRCHECK)
//            {

//                cuda_init(&this->params_right);

//                cv::Mat outputDm_r = zy_remap(grayR, grayL);

//                this->stereo->setRightDisparity(outputDm_r);

//                cuda_init(&cuda_params);
//            }

//            break;

//        default:

//            throw std::runtime_error(std::string("Wrong stereo_matching value."));
//    }

    matcher->compute();

    mutexDisp.unlock();

//    PROF_E
//    PROF_D(6)

//    cv::Mat disp_blf;
//    cv::cuda::GpuMat imageGpu, gpuDisp, filtGpu;

//    PROF_S

    matcher->filterBLF("base");

//    switch(this->BLFfiltering)
//    {
//        case STEREO_VISION::BLF_ORIGINAL:

//            cv_extend::bilateralFilter(outputDm, disp_blf, sigmaColorBLF, sigmaSpaceBLF);

//            break;

//        case STEREO_VISION::BLF_CUDA:

//            grayL = this->stereo->getLRectified();
//            cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

//            imageGpu.upload(grayL);
//            gpuDisp.upload(outputDm);

//            pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

//            filtGpu.download(disp_blf);

//            break;

//        case STEREO_VISION::BLF_DISABLED:
//        default:

//            disp_blf = outputDm;

//            break;
//    }

//    PROF_E
//    PROF_D(7)


//    cv::Mat disp_wls, disp_wls_2;
//    Rect ROI;
//    Ptr<DisparityWLSFilter> wls_filter;

//    PROF_S

    matcher->filterWLS("blf");

//    if(this->WLSfiltering != STEREO_VISION::WLS_DISABLED)
//    {

//        Ptr<StereoSGBM> sgbm =cv::StereoSGBM::create(this->minDisparity,this->numberOfDisparities,this->SADWindowSize,
//                                                    8*3*this->SADWindowSize*this->SADWindowSize,
//                                                    32*3*this->SADWindowSize*this->SADWindowSize,
//                                                    this->disp12MaxDiff,this->preFilterCap,this->uniquenessRatio,
//                                                    this->speckleWindowSize, this->speckleRange,
//                                                    this->useBestDisp?StereoSGBM::MODE_HH:StereoSGBM::MODE_SGBM);

//        cv::Mat left_rect = this->stereo->getLRectified();

//        switch(this->WLSfiltering)
//        {
//            case STEREO_VISION::WLS_ENABLED:

//                wls_filter = createDisparityWLSFilterGeneric(false);

//                wls_filter->setLambda(this->wls_lambda);
//                wls_filter->setSigmaColor(this->wls_sigma);
//                wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*this->SADWindowSize));
//                ROI = computeROI2(left_rect.size(),sgbm);

//                wls_filter->filter(disp_blf,left_rect,disp_wls,Mat(),ROI);
//                wls_filter->filter(outputDm,left_rect,disp_wls_2,Mat(),ROI);

//                break;

//            case STEREO_VISION::WLS_LRCHECK:

//                if(this->stereo_matching == STEREO_VISION::SGBM_OPENCV)
//                {
//                    wls_filter = createDisparityWLSFilter(sgbm);

//                    wls_filter->setLambda(wls_lambda);
//                    wls_filter->setSigmaColor(wls_sigma);

////                    std::cout << this->stereo->getRightDisparity().channels() << std::endl;
////                    std::cout << this->stereo->getRightDisparity().size << std::endl;
////                    std::cout << this->stereo->getRightDisparity().rows << std::endl;
////                    std::cout << this->stereo->getRightDisparity().cols << std::endl;

//                    wls_filter->filter(disp_blf,left_rect,disp_wls,this->stereo->getRightDisparity());
//                }
//                else
//                    disp_wls = disp_blf;

//                break;

//            default:

//                break;
//        }
//    }
//    else
//        disp_wls = disp_blf;


//    PROF_E
//    PROF_D(8)


//    PROF_S

    cv::Mat disp_vis = matcher->getDisparity("base");

    if (outDisp.getOutputCount()>0 && !disp_vis.empty())
    {
        ImageOf<PixelMono> &outim = outDisp.prepare();

        if(this->stereo_matching == SM_MATCHING_ALG::SGBM_CUDA)
            getDisparityVis(disp_vis, disp_vis, 3);

        outim = fromCvMat<PixelMono>(disp_vis);
        outDisp.write();
    }

//    PROF_E
//    PROF_D(9)


//    PROF_S


    if (outDepth.getOutputCount()>0)
    {
        cv::Mat disp_depth = matcher->getDisparity("base");

        if (disp_depth.empty())
        {
            std::cout << "!!! Impossible to compute the depth map: disparity is not available" << std::endl;
        }
        else
        {

            outputDepth = this->depthFromDisparity_alt(disp_depth, this->stereo->getQ(), this->stereo->getRLrect());

            ImageOf<PixelFloat> &outim = outDepth.prepare();

//            if(this->WLSfiltering != SM_WLS_FILTER::WLS_DISABLED)
//            {
//                outputDepth = this->depthFromDisparity_alt(disp_wls_2, this->stereo->getQ(), this->stereo->getRLrect());
//            }
//            else
//            {
//                if(this->stereo_matching == SM_MATCHING_ALG::SGBM_CUDA)
//                {

//                    if(this->BLFfiltering == SM_BLF_FILTER::BLF_ORIGINAL)
//                        cv_extend::bilateralFilter(outputDm, outputDm, sigmaColorBLF, sigmaSpaceBLF);

//                    if(this->BLFfiltering == SM_BLF_FILTER::BLF_CUDA)
//                    {
//                        grayL = this->stereo->getLRectified();
//                        cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

//                        imageGpu.upload(grayL);
//                        gpuDisp.upload(outputDm);

//                        pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

//                        filtGpu.download(outputDm);
//                    }

//                    outputDepth = this->depthFromDisparity_alt(outputDm, this->stereo->getQ(), this->stereo->getRLrect());
//                }
//                else
//                {

//                    cv::Mat d = this->stereo->getDisparity16();

//                    if(this->BLFfiltering == SM_BLF_FILTER::BLF_ORIGINAL)
//                        cv_extend::bilateralFilter(d, d, sigmaColorBLF, sigmaSpaceBLF);

//                    if(this->BLFfiltering == SM_BLF_FILTER::BLF_CUDA)
//                    {
//                        grayL = this->stereo->getLRectified();
//                        cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

//                        imageGpu.upload(grayL);
//                        gpuDisp.upload(d);

//                        pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

//                        filtGpu.download(d);
//                    }


//                    outputDepth = this->depthFromDisparity_alt(d, this->stereo->getQ(), this->stereo->getRLrect());
//                }
//            }


//            threshold(outputDepth, outputDepth, 10., 10., CV_THRESH_TRUNC);

//                threshold(outputDepth, outputDepth, 0., 0., CV_THRESH_TOZERO);
//                cv::cvtColor(outputDepth, outputDepth,CV_32F);

//                getDisparityVis(outputDepth, outputDepth, 25);

//                cv::Mat cmm, cm2;

//                cv::cvtColor(outputDepth, cm2, CV_8UC1);

//                applyColorMap(outputDepth, cmm, COLORMAP_AUTUMN);


//                cv::namedWindow("AA");
//                cv::imshow("AA", cmm);
//                cv::waitKey(1);

//                outputDepth *= 10;

//            std::cout << "FFF" << std::endl;

            outim = fromCvMat<PixelFloat>(outputDepth);

//            std::cout << "GGG" << std::endl;

            outDepth.write();

        }

    }

//    PROF_E
//    PROF_D(10)


    // DEBUG: printing values from the disparity and depth maps

//    if(!outputDm.empty())
//    {
//        std::cout << "------------------" << std::endl;

//        double min, max;
//        cv::minMaxLoc(disp_wls, &min, &max);
//        std::cout << "DEBUG: min, max for disparity map: " << min << ", " << max << std::endl;
//    }

//    if(!outputDepth.empty())
//    {
//        double min, max;
//        cv::minMaxLoc(outputDepth, &min, &max);
//        std::cout << "DEBUG: min, max for depth map: " << min << ", " << max << std::endl;
//    }

    if(this->debugWindow)
        this->gui.updateGUI();

    if(this->gui.isDone())
        this->gui.killGUI();

//    if(++debug_count == 100)
//    {

//        std::cout << std::endl << "---- DISPMODULE: AVERAGE TIMING OVER " << debug_count << " FRAMES ----" << std::endl;

//        for(int i = 0; i < this->debug_num_timings; i++)
//        {
//            std::cout << debug_strings[i] << ": " << (this->debug_timings[i] / debug_count) / 1000. << " ms" << std::endl;
//            this->debug_timings[i] = 0;
//        }

//        debug_count = 0;
//    }

    return true;
}

/*
bool DispModule::updateModule()
{
    ImageOf<PixelRgb> *yarp_imgL=leftImgPort.read(true);
    ImageOf<PixelRgb> *yarp_imgR=rightImgPort.read(true);

    auto start = std::chrono::high_resolution_clock::now();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);


    std::cout << std::setprecision(2) << std::fixed;

    if(this->usePorts)
    {
        // code to read from fakeEyes port
    }


    Stamp stamp_left, stamp_right;
    leftImgPort.getEnvelope(stamp_left);
    rightImgPort.getEnvelope(stamp_right);

    if ((yarp_imgL==NULL) || (yarp_imgR==NULL))
        return true;

    // read encoders
    iencs->getEncoder(nHeadAxes-3,&eyes[0]);
    iencs->getEncoder(nHeadAxes-2,&eyes[1]);
    iencs->getEncoder(nHeadAxes-1,&eyes[2]);

    PROF_S

    updateViaKinematics(eyes-eyes0);

    PROF_E
    PROF_D(0)

    PROF_S

    updateViaGazeCtrl(false);

    PROF_E
    PROF_D(1)

    leftMat=toCvMat(*yarp_imgL);
    rightMat=toCvMat(*yarp_imgR);

    //DEBUG

    getCameraHGazeCtrl(LEFT);
    getCameraHGazeCtrl(RIGHT);

    PROF_S

    this->stereo->setImages(leftMat,rightMat);

    PROF_E
    PROF_D(2)
    PROF_S

    if(this->gui.isUpdated() && this->gui.toRecalibrate())
    {
        std::cout << "Updating..." << std::endl;

        mutexRecalibration.lock();
        numberOfTrials=0;
        doSFM=true;
        mutexRecalibration.unlock();

        this->gui.setUpdated(false, false);
    }

    PROF_E
    PROF_D(3)
    PROF_S

    mutexRecalibration.lock();

    if (doSFM)
    {
#ifdef USING_GPU
        utils->extractMatch_GPU(leftMat,rightMat);
        vector<Point2f> leftM,rightM;
        utils->getMatches(leftM,rightM);
        mutexDisp.lock();
        this->stereo->setMatches(leftM,rightM);
#else

        mutexDisp.lock();
        this->stereo->findMatch(false);
#endif
        this->stereo->estimateEssential();
        bool ok=this->stereo->essentialDecomposition();
        mutexDisp.unlock();

        if (ok)
        {
            calibUpdated=true;
            doSFM=false;
            calibEndEvent.signal();

            R0=this->stereo->getRotation();
            T0=this->stereo->getTranslation();
            eyes0=eyes;

            std::cout << "Calibration Successful!" << std::endl;

        }
        else
        {
            if (++numberOfTrials>5)
            {
                calibUpdated=false;
                doSFM=false;
                calibEndEvent.signal();

                std::cout << "Calibration failed after 5 trials.. Please show a non planar scene." << std::endl;

            }
        }
    }

    mutexRecalibration.unlock();

    PROF_E
    PROF_D(4)

    PROF_S

    if(this->gui.isUpdated())
    {

        this->gui.getParams(this->minDisparity, this->numberOfDisparities, this->SADWindowSize,
                            this->disp12MaxDiff, this->preFilterCap, this->uniquenessRatio,
                            this->speckleWindowSize, this->speckleRange, this->sigmaColorBLF, this->sigmaSpaceBLF,
                            this->wls_lambda, this->wls_sigma, this->BLFfiltering, this->WLSfiltering, this->stereo_matching);

        this->cuda_params.preFilterCap = this->preFilterCap;
        this->cuda_params.BlockSize = this->SADWindowSize;
        this->cuda_params.uniquenessRatio = this->uniquenessRatio;
        this->cuda_params.disp12MaxDiff = this->disp12MaxDiff;

        cuda_init(&this->cuda_params);
        pCudaBilFilter->setSigmaRange(sigmaColorBLF);
        pCudaBilFilter->setNumDisparities(this->numberOfDisparities);

        this->gui.setUpdated(false);
    }

    PROF_E
    PROF_D(5)

    cv::Mat grayL, grayR;

    PROF_S

    mutexDisp.lock();

    switch(this->stereo_matching)
    {
        case STEREO_VISION::SGBM_OPENCV:
        case STEREO_VISION::LIBELAS:

            this->stereo->computeDisparity(this->useBestDisp,this->uniquenessRatio,this->speckleWindowSize,
                        this->speckleRange,this->numberOfDisparities,this->SADWindowSize,
                        this->minDisparity,this->preFilterCap,this->disp12MaxDiff);

            outputDm = stereo->getDisparity();

            if(this->WLSfiltering == STEREO_VISION::WLS_LRCHECK)
                this->stereo->computeRightDisparity(this->useBestDisp,this->uniquenessRatio,this->speckleWindowSize,
                                                    this->speckleRange,this->numberOfDisparities,this->SADWindowSize,
                                                    this->minDisparity,this->preFilterCap,this->disp12MaxDiff);

            break;

        case STEREO_VISION::SGBM_CUDA:

            this->stereo->rectifyImages();

            grayL = this->stereo->getLRectified();
            grayR = this->stereo->getRRectified();

            cv::cvtColor(grayL, grayL, CV_BGR2GRAY);
            cv::cvtColor(grayR, grayR, CV_BGR2GRAY);

            outputDm = zy_remap(grayL, grayR);

            // DEBUG: NOT WORKING

            if(this->WLSfiltering == STEREO_VISION::WLS_LRCHECK)
            {

                cuda_init(&this->params_right);

                cv::Mat outputDm_r = zy_remap(grayR, grayL);

                this->stereo->setRightDisparity(outputDm_r);

                cuda_init(&cuda_params);
            }

            break;

        default:

            throw std::runtime_error(std::string("Wrong stereo_matching value."));
    }

    mutexDisp.unlock();

    PROF_E
    PROF_D(6)

    cv::Mat disp_blf;
    cv::cuda::GpuMat imageGpu, gpuDisp, filtGpu;

    PROF_S

    switch(this->BLFfiltering)
    {
        case STEREO_VISION::BLF_ORIGINAL:

            cv_extend::bilateralFilter(outputDm, disp_blf, sigmaColorBLF, sigmaSpaceBLF);

            break;

        case STEREO_VISION::BLF_CUDA:

            grayL = this->stereo->getLRectified();
            cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

            imageGpu.upload(grayL);
            gpuDisp.upload(outputDm);

            pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

            filtGpu.download(disp_blf);

            break;

        case STEREO_VISION::BLF_DISABLED:
        default:

            disp_blf = outputDm;

            break;
    }

    PROF_E
    PROF_D(7)


    cv::Mat disp_wls, disp_wls_2;
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;

    PROF_S

    if(this->WLSfiltering != STEREO_VISION::WLS_DISABLED)
    {

        Ptr<StereoSGBM> sgbm =cv::StereoSGBM::create(this->minDisparity,this->numberOfDisparities,this->SADWindowSize,
                                                    8*3*this->SADWindowSize*this->SADWindowSize,
                                                    32*3*this->SADWindowSize*this->SADWindowSize,
                                                    this->disp12MaxDiff,this->preFilterCap,this->uniquenessRatio,
                                                    this->speckleWindowSize, this->speckleRange,
                                                    this->useBestDisp?StereoSGBM::MODE_HH:StereoSGBM::MODE_SGBM);

        cv::Mat left_rect = this->stereo->getLRectified();

        switch(this->WLSfiltering)
        {
            case STEREO_VISION::WLS_ENABLED:

                wls_filter = createDisparityWLSFilterGeneric(false);

                wls_filter->setLambda(this->wls_lambda);
                wls_filter->setSigmaColor(this->wls_sigma);
                wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*this->SADWindowSize));
                ROI = computeROI2(left_rect.size(),sgbm);

                wls_filter->filter(disp_blf,left_rect,disp_wls,Mat(),ROI);
                wls_filter->filter(outputDm,left_rect,disp_wls_2,Mat(),ROI);

                break;

            case STEREO_VISION::WLS_LRCHECK:

                if(this->stereo_matching == STEREO_VISION::SGBM_OPENCV)
                {
                    wls_filter = createDisparityWLSFilter(sgbm);

                    wls_filter->setLambda(wls_lambda);
                    wls_filter->setSigmaColor(wls_sigma);

//                    std::cout << this->stereo->getRightDisparity().channels() << std::endl;
//                    std::cout << this->stereo->getRightDisparity().size << std::endl;
//                    std::cout << this->stereo->getRightDisparity().rows << std::endl;
//                    std::cout << this->stereo->getRightDisparity().cols << std::endl;

                    wls_filter->filter(disp_blf,left_rect,disp_wls,this->stereo->getRightDisparity());
                }
                else
                    disp_wls = disp_blf;

                break;

            default:

                break;
        }
    }
    else
        disp_wls = disp_blf;


    PROF_E
    PROF_D(8)


    PROF_S

    if (outDisp.getOutputCount()>0 && !disp_wls.empty())
    {
        ImageOf<PixelMono> &outim = outDisp.prepare();

        if(this->stereo_matching == STEREO_VISION::SGBM_CUDA)
            getDisparityVis(disp_wls, disp_wls, 3);

        outim = fromCvMat<PixelMono>(disp_wls);
        outDisp.write();
    }

    PROF_E
    PROF_D(9)


    PROF_S

    if (outDepth.getOutputCount()>0)
    {

        if (disp_wls.empty())
        {
            std::cout << "!!! Impossible to compute the depth map: disparity is not available" << std::endl;
        }
        else
        {

            ImageOf<PixelFloat> &outim = outDepth.prepare();

            if(this->WLSfiltering != STEREO_VISION::WLS_DISABLED)
            {
                outputDepth = this->depthFromDisparity_alt(disp_wls_2, this->stereo->getQ(), this->stereo->getRLrect());
            }
            else
            {
                if(this->stereo_matching == STEREO_VISION::SGBM_CUDA)
                {

                    if(this->BLFfiltering == STEREO_VISION::BLF_ORIGINAL)
                        cv_extend::bilateralFilter(outputDm, outputDm, sigmaColorBLF, sigmaSpaceBLF);

                    if(this->BLFfiltering == STEREO_VISION::BLF_CUDA)
                    {
                        grayL = this->stereo->getLRectified();
                        cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

                        imageGpu.upload(grayL);
                        gpuDisp.upload(outputDm);

                        pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

                        filtGpu.download(outputDm);
                    }

                    outputDepth = this->depthFromDisparity_alt(outputDm, this->stereo->getQ(), this->stereo->getRLrect());
                }
                else
                {

                    cv::Mat d = this->stereo->getDisparity16();

                    if(this->BLFfiltering == STEREO_VISION::BLF_ORIGINAL)
                        cv_extend::bilateralFilter(d, d, sigmaColorBLF, sigmaSpaceBLF);

                    if(this->BLFfiltering == STEREO_VISION::BLF_CUDA)
                    {
                        grayL = this->stereo->getLRectified();
                        cv::cvtColor(grayL, grayL, CV_BGR2GRAY);

                        imageGpu.upload(grayL);
                        gpuDisp.upload(d);

                        pCudaBilFilter->apply(gpuDisp, imageGpu, filtGpu);

                        filtGpu.download(d);
                    }


                    outputDepth = this->depthFromDisparity_alt(d, this->stereo->getQ(), this->stereo->getRLrect());
                }
            }


//            threshold(outputDepth, outputDepth, 10., 10., CV_THRESH_TRUNC);

//                threshold(outputDepth, outputDepth, 0., 0., CV_THRESH_TOZERO);
//                cv::cvtColor(outputDepth, outputDepth,CV_32F);

//                getDisparityVis(outputDepth, outputDepth, 25);

//                cv::Mat cmm, cm2;

//                cv::cvtColor(outputDepth, cm2, CV_8UC1);

//                applyColorMap(outputDepth, cmm, COLORMAP_AUTUMN);


//                cv::namedWindow("AA");
//                cv::imshow("AA", cmm);
//                cv::waitKey(1);

//                outputDepth *= 10;

//            std::cout << "FFF" << std::endl;


            outim = fromCvMat<PixelFloat>(outputDepth);

//            std::cout << "GGG" << std::endl;

            outDepth.write();

        }

    }

    PROF_E
    PROF_D(10)


    // DEBUG: printing values from the disparity and depth maps

//    if(!outputDm.empty())
//    {
//        std::cout << "------------------" << std::endl;

//        double min, max;
//        cv::minMaxLoc(disp_wls, &min, &max);
//        std::cout << "DEBUG: min, max for disparity map: " << min << ", " << max << std::endl;
//    }

//    if(!outputDepth.empty())
//    {
//        double min, max;
//        cv::minMaxLoc(outputDepth, &min, &max);
//        std::cout << "DEBUG: min, max for depth map: " << min << ", " << max << std::endl;
//    }

    if(this->debugWindow)
        this->gui.updateGUI();

    if(this->gui.isDone())
        this->gui.killGUI();

    if(++debug_count == 100)
    {

        std::cout << std::endl << "---- DISPMODULE: AVERAGE TIMING OVER " << debug_count << " FRAMES ----" << std::endl;

        for(int i = 0; i < this->debug_num_timings; i++)
        {
            std::cout << debug_strings[i] << ": " << (this->debug_timings[i] / debug_count) / 1000. << " ms" << std::endl;
            this->debug_timings[i] = 0;
        }

        debug_count = 0;
    }

    return true;
}
*/

/******************************************************************************/
double DispModule::getPeriod()
{
    // the updateModule() method gets synchronized
    // with camera input => no need for periodicity
    return 0.0;
}


/******************************************************************************/
bool DispModule::loadExtrinsics(yarp::os::ResourceFinder& rf, Mat& Ro, Mat& To, yarp::sig::Vector& eyes)
{
    Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");

    eyes.resize(3,0.0);
    if (Bottle *bEyes=extrinsics.find("eyes").asList())
    {
        size_t sz=std::min(eyes.length(),(size_t)bEyes->size());
        for (size_t i=0; i<sz; i++)
            eyes[i]=bEyes->get(i).asDouble();
    }

    cout<<"read eyes configuration = ("<<eyes.toString(3,3)<<")"<<endl;

    if (Bottle *pXo=extrinsics.find("HN").asList())
    {
        Ro=Mat::zeros(3,3,CV_64FC1);
        To=Mat::zeros(3,1,CV_64FC1);
        for (int i=0; i<(pXo->size()-4); i+=4)
        {
            Ro.at<double>(i/4,0)=pXo->get(i).asDouble();
            Ro.at<double>(i/4,1)=pXo->get(i+1).asDouble();
            Ro.at<double>(i/4,2)=pXo->get(i+2).asDouble();
            To.at<double>(i/4,0)=pXo->get(i+3).asDouble();
        }
    }
    else
        return false;

    return true;
}


/******************************************************************************/
bool DispModule::loadIntrinsics(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL,
        Mat &DistR)
{
    Bottle left=rf.findGroup("CAMERA_CALIBRATION_LEFT");
    if(!left.check("fx") || !left.check("fy") || !left.check("cx") || !left.check("cy"))
        return false;

    double fx=left.find("fx").asDouble();
    double fy=left.find("fy").asDouble();

    double cx=left.find("cx").asDouble();
    double cy=left.find("cy").asDouble();

    double k1=left.check("k1",Value(0)).asDouble();
    double k2=left.check("k2",Value(0)).asDouble();

    double p1=left.check("p1",Value(0)).asDouble();
    double p2=left.check("p2",Value(0)).asDouble();

    DistL=Mat::zeros(1,8,CV_64FC1);
    DistL.at<double>(0,0)=k1;
    DistL.at<double>(0,1)=k2;
    DistL.at<double>(0,2)=p1;
    DistL.at<double>(0,3)=p2;

    KL=Mat::eye(3,3,CV_64FC1);
    KL.at<double>(0,0)=fx;
    KL.at<double>(0,2)=cx;
    KL.at<double>(1,1)=fy;
    KL.at<double>(1,2)=cy;

    Bottle right=rf.findGroup("CAMERA_CALIBRATION_RIGHT");
    if(!right.check("fx") || !right.check("fy") || !right.check("cx") || !right.check("cy"))
        return false;

    fx=right.find("fx").asDouble();
    fy=right.find("fy").asDouble();

    cx=right.find("cx").asDouble();
    cy=right.find("cy").asDouble();

    k1=right.check("k1",Value(0)).asDouble();
    k2=right.check("k2",Value(0)).asDouble();

    p1=right.check("p1",Value(0)).asDouble();
    p2=right.check("p2",Value(0)).asDouble();

    DistR=Mat::zeros(1,8,CV_64FC1);
    DistR.at<double>(0,0)=k1;
    DistR.at<double>(0,1)=k2;
    DistR.at<double>(0,2)=p1;
    DistR.at<double>(0,3)=p2;

    KR=Mat::eye(3,3,CV_64FC1);
    KR.at<double>(0,0)=fx;
    KR.at<double>(0,2)=cx;
    KR.at<double>(1,1)=fy;
    KR.at<double>(1,2)=cy;

    return true;
}


/******************************************************************************/
bool DispModule::updateExtrinsics(Mat& Rot, Mat& Tr, yarp::sig::Vector& eyes,
        const string& groupname)
{
    ofstream out;
    out.open(camCalibFile.c_str());
    if (out.is_open())
    {
        out << endl;
        out << "["+groupname+"]" << endl;
        out << "eyes (" << eyes.toString() << ")" << endl;
        out << "HN (" << Rot.at<double>(0,0) << " " << Rot.at<double>(0,1) << " " << Rot.at<double>(0,2) << " " << Tr.at<double>(0,0) << " "
                << Rot.at<double>(1,0) << " " << Rot.at<double>(1,1) << " " << Rot.at<double>(1,2) << " " << Tr.at<double>(1,0) << " "
                << Rot.at<double>(2,0) << " " << Rot.at<double>(2,1) << " " << Rot.at<double>(2,2) << " " << Tr.at<double>(2,0) << " "
                << 0.0                 << " " << 0.0                 << " " << 0.0                 << " " << 1.0                << ")"
                << endl;
        out.close();
        return true;
    }
    else
        return false;
}


/******************************************************************************/
void DispModule::setDispParameters(bool _useBestDisp, int _uniquenessRatio,
        int _speckleWindowSize,int _speckleRange,
        int _numberOfDisparities, int _SADWindowSize,
        int _minDisparity, int _preFilterCap, int _disp12MaxDiff)
{
    this->mutexDisp.lock();

    this->useBestDisp=_useBestDisp;
    this->uniquenessRatio=_uniquenessRatio;
    this->speckleWindowSize=_speckleWindowSize;
    this->speckleRange=_speckleRange;
    this->numberOfDisparities=_numberOfDisparities;
    this->SADWindowSize=_SADWindowSize;
    this->minDisparity=_minDisparity;
    this->preFilterCap=_preFilterCap;
    this->disp12MaxDiff=_disp12MaxDiff;

    this->mutexDisp.unlock();

}

/******************************************************************************/
Point3f DispModule::get3DPoints(int u, int v, const string &drive)
{
    Point3f point(0.0f,0.0f,0.0f);
    if ((drive!="RIGHT") && (drive!="LEFT") && (drive!="ROOT"))
        return point;

    LockGuard lg(mutexDisp);

    // Mapping from Rectified Cameras to Original Cameras
    const Mat& Mapper=this->stereo->getMapperL();
    if (Mapper.empty())
        return point;

    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1];

    u=cvRound(usign);
    v=cvRound(vsign);

    const Mat& disp16m=this->stereo->getDisparity16();
    if (disp16m.empty() || (u<0) || (u>=disp16m.cols) || (v<0) || (v>=disp16m.rows))
        return point;

    const Mat& Q=this->stereo->getQ();
    IplImage disp16=disp16m;
    CvScalar scal=cvGet2D(&disp16,v,u);
    double disparity=scal.val[0]/16.0;
    float w=(float)(disparity*Q.at<double>(3,2)+Q.at<double>(3,3));
    point.x=(float)((usign+1)*Q.at<double>(0,0)+Q.at<double>(0,3));
    point.y=(float)((vsign+1)*Q.at<double>(1,1)+Q.at<double>(1,3));
    point.z=(float)Q.at<double>(2,3);

    point.x/=w;
    point.y/=w;
    point.z/=w;

    // discard points far more than 10 meters or with not valid disparity (<0)

    // TODO: "discards" means it returns the point in the same way?

    if ((point.z>10.0f) || (point.z<0.0f))
        return point;

    if (drive=="ROOT")
    {
        const Mat& RLrect=this->stereo->getRLrect().t();
        Mat Tfake=Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1.0;

        Mat Hrect=buildRotTras(RLrect,Tfake);
        P=HL_root*Hrect*P;
        point.x=(float)(P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float)(P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float)(P.at<double>(2,0)/P.at<double>(3,0));
    }
    else if (drive=="LEFT")
    {
        Mat P(3,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;

        P=this->stereo->getRLrect().t()*P;
        point.x=(float)P.at<double>(0,0);
        point.y=(float)P.at<double>(1,0);
        point.z=(float)P.at<double>(2,0);
    }
    else if (drive=="RIGHT")
    {
        const Mat& Rright=this->stereo->getRotation();
        const Mat& Tright=this->stereo->getTranslation();
        const Mat& RRright=this->stereo->getRRrect().t();
        Mat TRright=Mat::zeros(0,3,CV_64F);

        Mat HRL=buildRotTras(Rright,Tright);
        Mat Hrect=buildRotTras(RRright,TRright);

        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1.0;

        P=Hrect*HRL*P;
        point.x=(float)(P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float)(P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float)(P.at<double>(2,0)/P.at<double>(3,0));
    }

    return point;
}

/******************************************************************************/
Mat DispModule::buildRotTras(const Mat& R, const Mat& T)
{     
    Mat A=Mat::eye(4,4,CV_64F);
    for (int i=0; i<R.rows; i++)
    {
        double* Mi=A.ptr<double>(i);
        const double* MRi=R.ptr<double>(i);
        for (int j=0; j<R.cols; j++)
            Mi[j]=MRi[j];
    }

    for (int i=0; i<T.rows; i++)
    {
        double* Mi=A.ptr<double>(i);
        const double* MRi=T.ptr<double>(i);
        Mi[3]=MRi[0];
    }

    return A;
}


/******************************************************************************/
Matrix DispModule::getCameraHGazeCtrl(int camera)
{
    yarp::sig::Vector x_curr;
    yarp::sig::Vector o_curr;
    bool check=false;
    if(camera==LEFT)
        check=igaze->getLeftEyePose(x_curr, o_curr);
    else
        check=igaze->getRightEyePose(x_curr, o_curr);

    if(!check)
    {
        Matrix H_curr(4, 4);
        return H_curr;
    }

    Matrix R_curr=axis2dcm(o_curr);
    Matrix H_curr=R_curr;
    H_curr.setSubcol(x_curr,0,3);

    if(camera==LEFT)
    {
        mutexDisp.lock();
        convert(H_curr,HL_root);
        mutexDisp.unlock();
    }
    else if(camera==RIGHT)
    {
        mutexDisp.lock();
        convert(H_curr,HR_root);
        mutexDisp.unlock();
    }

    return H_curr;
}


/******************************************************************************/
void DispModule::convert(Matrix& matrix, Mat& mat)
{
    mat=cv::Mat(matrix.rows(),matrix.cols(),CV_64FC1);
    for(int i=0; i<matrix.rows(); i++)
        for(int j=0; j<matrix.cols(); j++)
            mat.at<double>(i,j)=matrix(i,j);
}


/******************************************************************************/
void DispModule::convert(Mat& mat, Matrix& matrix)
{
    matrix.resize(mat.rows,mat.cols);
    for(int i=0; i<mat.rows; i++)
        for(int j=0; j<mat.cols; j++)
            matrix(i,j)=mat.at<double>(i,j);
}


/******************************************************************************/
bool DispModule::respond(const Bottle& command, Bottle& reply)
{
    if(command.size()==0)
        return false;

    if (command.get(0).asString()=="quit") {
        cout << "closing..." << endl;
        return false;
    }

    if (command.get(0).asString()=="help") {
//        reply.addVocab(Vocab::encode("many"));
//        reply.addString("Available commands are:");
//        reply.addString("- [calibrate]: It recomputes the camera positions once.");
//        reply.addString("- [Rect tlx tly w h step]: Given the pixels in the rectangle defined by {(tlx,tly) (tlx+w,tly+h)} (parsed by columns), the response contains the corresponding 3D points in the ROOT frame. The optional parameter step defines the sampling quantum; by default step=1.");
//        reply.addString("- [Points u_1 v_1 ... u_n v_n]: Given a list of n pixels, the response contains the corresponding 3D points in the ROOT frame.");
//        reply.addString("- [cart2stereo X Y Z]: Given a world point X Y Z wrt to ROOT reference frame the response is the projection (uL vL uR vR) in the RGB image (in Left and Right images, which are the same here, for compatibility with SFM module).");
//        reply.addString("For more details on the commands, check the module's documentation");
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("- [calibrate]: It recomputes the camera positions once.");
        reply.addString("- [save]: It saves the current camera positions and uses it when the module starts.");
        reply.addString("- [getH]: It returns the calibrated stereo matrix.");
        reply.addString("- [setNumDisp NumOfDisparities]: It sets the expected number of disparity (in pixel). Values must be divisible by 32. ");
        reply.addString("- [Point x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye.");
        reply.addString("- [x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z ur vr computed using the depth map wrt the the ROOT reference system.(ur vr) is the corresponding pixel in the Right image. ");
        reply.addString("- [Left x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the LEFT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0). ");
        reply.addString("- [Right x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the RIGHT eye. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).");
        reply.addString("- [Root x y]: Given the pixel coordinate x,y in the Left image the response is the 3D Point: X Y Z computed using the depth map wrt the ROOT reference system. Points with non valid disparity (i.e. occlusions) are handled with the value (0.0,0.0,0.0).");
        reply.addString("- [Rect tlx tly w h step]: Given the pixels in the rectangle defined by {(tlx,tly) (tlx+w,tly+h)} (parsed by columns), the response contains the corresponding 3D points in the ROOT frame. The optional parameter step defines the sampling quantum; by default step=1.");
        reply.addString("- [Points u_1 v_1 ... u_n v_n]: Given a list of n pixels, the response contains the corresponding 3D points in the ROOT frame.");
        reply.addString("- [Flood3D x y dist]: Perform 3D flood-fill on the seed point (x,y), returning the following info: [u_1 v_1 x_1 y_1 z_1 ...]. The optional parameter dist expressed in meters regulates the fill (by default = 0.004).");
        reply.addString("- [uL_1 vL_1 uR_1 vR_1 ... uL_n vL_n uR_n vR_n]: Given n quadruples uL_i vL_i uR_i vR_i, where uL_i vL_i are the pixel coordinates in the Left image and uR_i vR_i are the coordinates of the matched pixel in the Right image, the response is a set of 3D points (X1 Y1 Z1 ... Xn Yn Zn) wrt the ROOT reference system.");
        reply.addString("- [cart2stereo X Y Z]: Given a world point X Y Z wrt to ROOT reference frame the response is the projection (uL vL uR vR) in the Left and Right images.");
        reply.addString("- [doBLF flag]: activate Bilateral filter for flag = true, and skip it for flag = false.");
        reply.addString("- [bilatfilt sigmaColor sigmaSpace]: Set the parameters for the bilateral filer (default sigmaColor = 10.0, sigmaSpace = 10.0).");
        reply.addString("- [algorithm name]: Set the stereo matching algorithm used (default = SGMB(cpu)).");
        reply.addString("For more details on the commands, check the module's documentation");
        return true;
    }

    if (command.get(0).asString()=="calibrate")
    {
        mutexRecalibration.lock();
        numberOfTrials=0;
        doSFM=true;
        mutexRecalibration.unlock();

        calibEndEvent.reset();
        calibEndEvent.wait();

        if (calibUpdated)
        {
            R0=this->stereo->getRotation();
            T0=this->stereo->getTranslation();
            eyes0=eyes;

            reply.addString("ACK");
        }
        else
            reply.addString("Calibration failed after 5 trials.. Please show a non planar scene.");

        return true;
    }

    if (command.get(0).asString()=="save")
    {
        updateExtrinsics(R0,T0,eyes0,"STEREO_DISPARITY");
        reply.addString("ACK");
        return true;
    }

    if (command.get(0).asString()=="getH")
    {
        Mat RT0=buildRotTras(R0,T0);
        Matrix H0; convert(RT0,H0);

        reply.read(H0);
        return true;
    }

    if (command.get(0).asString()=="setNumDisp")
    {
        int dispNum=command.get(1).asInt();
        if(dispNum%32==0)
        {
            this->numberOfDisparities=dispNum;
            this->setDispParameters(useBestDisp,uniquenessRatio,speckleWindowSize,speckleRange,numberOfDisparities,SADWindowSize,minDisparity,preFilterCap,disp12MaxDiff);
            reply.addString("ACK");
            return true;
        }
        else
        {
            reply.addString("Num Disparity must be divisible by 32");
            return true;
        }
    }

    if (command.get(0).asString()=="setMinDisp")
    {
        int dispNum=command.get(1).asInt();
        this->minDisparity=dispNum;
        this->setDispParameters(useBestDisp,uniquenessRatio,speckleWindowSize,
                speckleRange,numberOfDisparities,SADWindowSize,
                minDisparity,preFilterCap,disp12MaxDiff);
        reply.addString("ACK");
        return true;
    }

    if (command.get(0).asString()=="set" && command.size()==10)
    {
        bool bestDisp=command.get(1).asInt() ? true : false;
        int uniquenessRatio=command.get(2).asInt();
        int speckleWindowSize=command.get(3).asInt();
        int speckleRange=command.get(4).asInt();
        int numberOfDisparities=command.get(5).asInt();
        int SADWindowSize=command.get(6).asInt();
        int minDisparity=command.get(7).asInt();
        int preFilterCap=command.get(8).asInt();
        int disp12MaxDiff=command.get(9).asInt();

        this->setDispParameters(bestDisp,uniquenessRatio,speckleWindowSize,
                                speckleRange,numberOfDisparities,SADWindowSize,
                                minDisparity,preFilterCap,disp12MaxDiff);
        reply.addString("ACK");
    }
    else if (command.get(0).asString()=="Rect")
    {
        int tl_u = command.get(1).asInt();
        int tl_v = command.get(2).asInt();
        int br_u = tl_u+command.get(3).asInt();
        int br_v = tl_v+command.get(4).asInt();

        int step = 1;
        if (command.size()>=6)
            step=command.get(5).asInt();

        for (int u=tl_u; u<br_u; u+=step)
        {
            for (int v=tl_v; v<br_v; v+=step)
            {
                Point3f point=this->get3DPoints(u,v,"ROOT");
                reply.addDouble(point.x);
                reply.addDouble(point.y);
                reply.addDouble(point.z);
            }
        }
    }
    else if (command.get(0).asString()=="Points")
    {
        for (int cnt=1; cnt<command.size()-1; cnt+=2)
        {
            int u=command.get(cnt).asInt();
            int v=command.get(cnt+1).asInt();
            Point3f point=this->get3DPoints(u,v,"ROOT");
            reply.addDouble(point.x);
            reply.addDouble(point.y);
            reply.addDouble(point.z);
        }
    }
    else if (command.get(0).asString()=="cart2stereo")
    {
        double x = command.get(1).asDouble();
        double y = command.get(2).asDouble();
        double z = command.get(3).asDouble();

        Point2f pointL = this->projectPoint("left",x,y,z);
        Point2f pointR = this->projectPoint("right",x,y,z);

        reply.addDouble(pointL.x);
        reply.addDouble(pointL.y);
        reply.addDouble(pointR.x);
        reply.addDouble(pointR.y);
    }
    else if (command.get(0).asString()=="bilatfilt" && command.size()==3)
    {
        if (!doBLF){
            doBLF = true;
            reply.addString("Bilateral filter activated.");
        }
        sigmaColorBLF = command.get(1).asDouble();
        sigmaSpaceBLF = command.get(2).asDouble();
        reply.addString("BLF sigmaColor ");
        reply.addDouble(sigmaColorBLF);
        reply.addString("BLF sigmaSpace ");
        reply.addDouble(sigmaSpaceBLF);
    }
    else if (command.get(0).asString()=="doBLF")
    {
        bool onoffBLF = command.get(1).asBool();
        if (onoffBLF == false ){     // turn OFF Bilateral Filtering
            if (doBLF == true){
                doBLF = false;
                reply.addString("Bilateral Filter OFF");
            } else {
                reply.addString("Bilateral Filter already OFF");
            }

        } else {                    // turn ON Bilateral Filtering
            if (doBLF == true){
                reply.addString("Bilateral Filter Already Running");
            } else {                                     // Set any different from 0 to activate bilateral filter.
                doBLF = true;
                reply.addString("Bilateral Filter ON");
            }
        }
        reply.addDouble(sigmaColorBLF);
        reply.addDouble(sigmaSpaceBLF);
    }
    else
        reply.addString("NACK");

    return true;
}


/******************************************************************************/
Point2f DispModule::projectPoint(const string &camera, double x, double y, double z)
{
    Point3f point3D;
    point3D.x=(float)x;
    point3D.y=(float)y;
    point3D.z=(float)z;

    vector<Point3f> points3D;

    points3D.push_back(point3D);

    vector<Point2f> response;

    mutexDisp.lock();

    if(camera=="left")
        response=this->stereo->projectPoints3D("left",points3D,HL_root);
    else
        response=this->stereo->projectPoints3D("right",points3D,HL_root);

    mutexDisp.unlock();

    return response[0];
}

DispModule::~DispModule()
{
    if(this->debugWindow)
        this->gui.killGUI();
}

DispModule::DispModule()
{

}

/******************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return 1;

    ResourceFinder rf;
    rf.setVerbose(true);

    rf.setDefaultConfigFile("icubEyes.ini");
//    rf.setDefaultConfigFile("icubEyes_640x480.ini");

    rf.setDefaultContext("cameraCalibration");
    rf.configure(argc,argv);

    DispModule mod;

    return mod.runModule(rf);
}

