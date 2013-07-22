#include "SFM_module.h"



bool SFM::configure(ResourceFinder &rf)
{
    string name=rf.check("name",Value("/SFM")).asString().c_str();
    string left=rf.check("leftPort",Value("/left:i")).asString().c_str();
    left=name+left;
    string right=rf.check("rightPort",Value("/right:i")).asString().c_str();

    string outDispName=rf.check("outDispPort",Value("/disp:o")).asString().c_str();
    string outMatchName=rf.check("outMatchPort",Value("/match:o")).asString().c_str();

    right=name+right;
    outMatchName=name+outMatchName;
    outDispName=name+outDispName;

    int calib= rf.check("useCalibrated",Value(1)).asInt();
    bool useCalibrated= calib ? true : false;

    leftImgPort.open(left.c_str());
    rightImgPort.open(right.c_str());
    outMatch.open(outMatchName.c_str());
    outDisp.open(outDispName.c_str());

    this->stereo=new StereoCamera(true);
    Mat KL, KR, DistL, DistR, R, T;
    loadStereoParameters(rf,KL,KR,DistL,DistR,R,T);
    
    this->mutexDisp = new Semaphore(1);

    stereo->setIntrinsics(KL,KR,DistL,DistR);
    stereo->setRotation(R,0);
    stereo->setTranslation(T,0);

    this->useBestDisp=true;
    this->uniquenessRatio=15;
    this->speckleWindowSize=50;
    this->speckleRange=16;
    this->numberOfDisparities=48;
    this->SADWindowSize=7;
    this->minDisparity=0;
    this->preFilterCap=63;
    this->disp12MaxDiff=0;
    
    this->HL_root= Mat::zeros(4,4,CV_64F);
    this->HR_root= Mat::zeros(4,4,CV_64F);

    if(useCalibrated)
    {
        Mat KL=this->stereo->getKleft();
        Mat KR=this->stereo->getKright();
        Mat zeroDist=Mat::zeros(1,8,CV_64FC1);
        this->stereo->setIntrinsics(KL,KR,zeroDist,zeroDist);
    }

    output_match=NULL;
    outputD=NULL;
    init=true;

    #ifdef USING_GPU
        utils = new Utilities();
        utils->initSIFT_GPU();
    #endif
    
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local","/client/disparityClient");
    gazeCtrl=new PolyDriver(option);
    if (gazeCtrl->isValid()) {
        gazeCtrl->view(igaze);
    }
    else {
        cout<<"Devices not available"<<endl;
        return false;
    }
        
    return true;
    
    
}

bool SFM::close()
{
    
    leftImgPort.interrupt();
    leftImgPort.close();

    rightImgPort.interrupt();
    rightImgPort.close();

    outDisp.close();
    outDisp.interrupt();

    outMatch.close();
    outMatch.interrupt();

    if(output_match!=NULL)
        cvReleaseImage(&output_match);

    if(outputD!=NULL)
        cvReleaseImage(&outputD);
   
    delete gazeCtrl;
    #ifdef USING_GPU
        delete utils;
    #endif

    delete mutexDisp;
    return true;
}

bool SFM::interruptModule()
{
    leftImgPort.interrupt();
    leftImgPort.close();
    rightImgPort.interrupt();
    rightImgPort.close();
    outDisp.close();
    outDisp.interrupt();

    outMatch.close();
    outMatch.interrupt();

    return true;
}
bool SFM::updateModule()
{
    ImageOf<PixelRgb> *yarp_imgL=NULL;
    ImageOf<PixelRgb> *yarp_imgR=NULL;

    
    yarp_imgL=leftImgPort.read(true);
    yarp_imgR=rightImgPort.read(true);

    if(yarp_imgL==NULL || yarp_imgR==NULL)
        return true;

    left=(IplImage*) yarp_imgL->getIplImage(); 
    //left= cvLoadImage("/usr/local/src/robot/iCub/app/cameraCalibration/conf/L.ppm");
    right=(IplImage*) yarp_imgR->getIplImage(); 
    //right=cvLoadImage("/usr/local/src/robot/iCub/app/cameraCalibration/conf/R.ppm");

    if(init)
    {
        output_match=cvCreateImage(cvSize(left->width*2,left->height),8,3);
        outputD=cvCreateImage(cvSize(left->width,left->height),8,3);
        // find matches
        init=false;
    }
    Matrix yarp_Left=getCameraHGazeCtrl(LEFT);
    Matrix yarp_Right=getCameraHGazeCtrl(RIGHT);     

    #ifdef USING_GPU
        Mat leftMat(left); 
        Mat rightMat(right);
        this->stereo->setImages(left,right);
        matMatches = Mat(rightMat.rows, 2*rightMat.cols, CV_8UC3);
        matMatches.adjustROI(0, 0, 0, -leftMat.cols);
        leftMat.copyTo(matMatches);
        matMatches.adjustROI(0, 0, -leftMat.cols, leftMat.cols);
        rightMat.copyTo(matMatches);
        matMatches.adjustROI(0, 0, leftMat.cols, 0);

        utils->extractMatch_GPU( leftMat, rightMat, matMatches );
        vector<Point2f> leftM;
        vector<Point2f> rightM;
        
        utils->getMatches(leftM,rightM);
        mutexDisp->wait();
        this->stereo->setMatches(leftM,rightM);
        this->stereo->estimateEssential();           

        

        

        this->stereo->essentialDecomposition();
      
        this->stereo->computeDisparity(this->useBestDisp, this->uniquenessRatio, this->speckleWindowSize, this->speckleRange, this->numberOfDisparities, this->SADWindowSize, this->minDisparity, this->preFilterCap, this->disp12MaxDiff);
        mutexDisp->post();
        
        
        Mat matches=this->stereo->drawMatches();
        vector<Point2f> matchtmp=this->stereo->getMatchRight();
        if(outMatch.getOutputCount()>0)
        {
            Mat F= this->stereo->getFundamental();
            
            if(matchtmp.size()>0)
            {
                Mat m(matchtmp);
                vector<Vec3f> lines;
                cv::computeCorrespondEpilines(m,2,F,lines);
                for (cv::vector<cv::Vec3f>::const_iterator it = lines.begin(); it!=lines.end(); ++it)
                {
                    cv::line(matMatches, cv::Point(0,-(*it)[2]/(*it)[1]), cv::Point(left->width,-((*it)[2] + (*it)[0]*left->width)/(*it)[1]),cv::Scalar(0,0,255));
                }        
            }
            cvtColor( matMatches, matMatches, CV_BGR2RGB);
            ImageOf<PixelBgr>& imgMatch= outMatch.prepare();
            imgMatch.resize(matMatches.cols, matMatches.rows);
            IplImage tmpR = matMatches;
            cvCopyImage( &tmpR, (IplImage *) imgMatch.getIplImage());        
            outMatch.write();
        }
                
        if(outDisp.getOutputCount()>0 && matchtmp.size()>0)
        {
            IplImage disp=stereo->getDisparity();
            cvCvtColor(&disp,outputD,CV_GRAY2RGB);
            ImageOf<PixelBgr>& outim=outDisp.prepare();
            outim.wrapIplImage(outputD);
            outDisp.write();
        }        
        
    #else
        // setting undistorted images
        this->stereo->setImages(left,right);

        // find matches
        this->stereo->findMatch(false,15,10.0);

        //Estimating fundamentalMatrix
        this->stereo->estimateEssential();
        Mat F= this->stereo->getFundamental();

        Mat matches=this->stereo->drawMatches();
        vector<Point2f> rightM=this->stereo->getMatchRight();

        this->stereo->essentialDecomposition();
        this->stereo->hornRelativeOrientations();

        this->stereo->computeDisparity(true,15,50,16,64,7,-32,32,0);

        if(outMatch.getOutputCount()>0 && rightM.size()>0)
        {
            Mat m(rightM);
            vector<Vec3f> lines;
            cv::computeCorrespondEpilines(m,2,F,lines);
            for (cv::vector<cv::Vec3f>::const_iterator it = lines.begin(); it!=lines.end(); ++it)
            {
                cv::line(matches,
                cv::Point(0,-(*it)[2]/(*it)[1]),
                cv::Point(left->width,-((*it)[2] + (*it)[0]*left->width)/(*it)[1]),
                cv::Scalar(255,255,255));
            }


            IplImage ipl_matches=matches;
            cvCvtColor(&ipl_matches,output_match,CV_BGR2RGB);

            ImageOf<PixelBgr>& outim=outMatch.prepare();
            outim.wrapIplImage(output_match);
            outMatch.write();
        }
        

        if(outDisp.getOutputCount()>0 && rightM.size()>0)
        {
            IplImage disp=stereo->getDisparity();
            cvCvtColor(&disp,outputD,CV_GRAY2RGB);
            ImageOf<PixelBgr>& outim=outDisp.prepare();
            outim.wrapIplImage(outputD);
            outDisp.write();
        }
    #endif

    return true;
}

double SFM::getPeriod()
{
    return 0.1;
}



bool SFM::loadStereoParameters(yarp::os::ResourceFinder &rf, Mat &KL, Mat &KR, Mat &DistL, Mat &DistR, Mat &Ro, Mat &T)
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

    Ro=Mat::zeros(3,3,CV_64FC1);
    T=Mat::zeros(3,1,CV_64FC1);

    Bottle extrinsics=rf.findGroup("STEREO_DISPARITY");
    if (Bottle *pXo=extrinsics.find("HN").asList()) {
        for (int i=0; i<(pXo->size()-4); i+=4) {
            Ro.at<double>(i/4,0)=pXo->get(i).asDouble();
            Ro.at<double>(i/4,1)=pXo->get(i+1).asDouble();
            Ro.at<double>(i/4,2)=pXo->get(i+2).asDouble();
            T.at<double>(i/4,0)=pXo->get(i+3).asDouble();
        }
    }
    else
        return false;

    return true;
}

void SFM::setDispParameters(bool _useBestDisp, int _uniquenessRatio, int _speckleWindowSize,int _speckleRange, int _numberOfDisparities, int _SADWindowSize, int _minDisparity, int _preFilterCap, int _disp12MaxDiff)
{
    this->mutexDisp->wait();
    this->useBestDisp=_useBestDisp;
    this->uniquenessRatio=_uniquenessRatio;
    this->speckleWindowSize=_speckleWindowSize;
    this->speckleRange=_speckleRange;
    this->numberOfDisparities=_numberOfDisparities;
    this->SADWindowSize=_SADWindowSize;
    this->minDisparity=_minDisparity;
    this->preFilterCap=_preFilterCap;
    this->disp12MaxDiff=_disp12MaxDiff;
    this->mutexDisp->post();

}


Point3f SFM::get3DPoints(int u, int v, string drive) {
    u=u; // matrix starts from (0,0), pixels from (0,0)
    v=v;
    Point3f point;

    if(drive!="RIGHT" && drive !="LEFT" && drive!="ROOT") {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return point;
    }

    this->mutexDisp->wait();


    // Mapping from Rectified Cameras to Original Cameras
    Mat Mapper=this->stereo->getMapperL();

    if(Mapper.empty()) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }


    float usign=Mapper.ptr<float>(v)[2*u];
    float vsign=Mapper.ptr<float>(v)[2*u+1];

    u=cvRound(usign);
    v=cvRound(vsign);


    IplImage disp16=this->stereo->getDisparity16();


    if(u<0 || u>=disp16.width || v<0 || v>=disp16.height) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }

    Mat Q=this->stereo->getQ();
    CvScalar scal= cvGet2D(&disp16,v,u);
    double disparity=-scal.val[0]/16.0;
    float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
    point.x= (float)((float) (usign+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
    point.y=(float)((float) (vsign+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
    point.z=(float) Q.at<double>(2,3);

    point.x=point.x/w;
    point.y=point.y/w;
    point.z=point.z/w;

    // discard points far more than 2.5 meters or with not valid disparity (<0)
    if(point.z>2.5 || point.z<0) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }

   if(drive=="LEFT") {
        Mat P(3,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;

        P=this->stereo->getRLrect().t()*P;

        point.x=(float) P.at<double>(0,0);
        point.y=(float) P.at<double>(1,0);
        point.z=(float) P.at<double>(2,0);
   }
   if(drive=="RIGHT") {
        Mat Rright = this->stereo->getRotation();
        Mat Tright = this->stereo->getTranslation();
        Mat RRright = this->stereo->getRRrect().t();
        Mat TRright = Mat::zeros(0,3,CV_64F);

        Mat HRL=buildRotTras(Rright,Tright);
        Mat Hrect=buildRotTras(RRright,TRright);

        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;
       
        P=Hrect*HRL*P;

        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));

    }
    if(drive=="ROOT") {
        Mat RLrect=this->stereo->getRLrect().t();
        Mat Tfake = Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;

        Mat Hrect=buildRotTras(RLrect,Tfake);
        P=HL_root*Hrect*P;
        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
   }

    this->mutexDisp->post();
    return point;

}


Point3f SFM::get3DPointMatch(double u1, double v1, double u2, double v2, string drive)
{
    Point3f point;
    if(drive!="RIGHT" && drive !="LEFT" && drive!="ROOT") {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        return point;
    }

    this->mutexDisp->wait();
    // Mapping from Rectified Cameras to Original Cameras
    Mat MapperL=this->stereo->getMapperL();
    Mat MapperR=this->stereo->getMapperR();

    if(MapperL.empty() || MapperR.empty()) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;

        this->mutexDisp->post();
        return point;
    }


    if(cvRound(u1)<0 || cvRound(u1)>=MapperL.cols || cvRound(v1)<0 || cvRound(v1)>=MapperL.rows) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }
    
        if(cvRound(u2)<0 || cvRound(u2)>=MapperL.cols || cvRound(v2)<0 || cvRound(v2)>=MapperL.rows) {
        point.x=0.0;
        point.y=0.0;
        point.z=0.0;
        this->mutexDisp->post();
        return point;
    }

    float urect1=MapperL.ptr<float>(cvRound(v1))[2*cvRound(u1)];
    float vrect1=MapperL.ptr<float>(cvRound(v1))[2*cvRound(u1)+1]; 

    float urect2=MapperR.ptr<float>(cvRound(v2))[2*cvRound(u2)];
    float vrect2=MapperR.ptr<float>(cvRound(v2))[2*cvRound(u2)+1]; 


    Mat Q=this->stereo->getQ();
    double disparity=urect2-urect1;
    float w= (float) ((float) disparity*Q.at<double>(3,2)) + ((float)Q.at<double>(3,3));
    point.x= (float)((float) (urect1+1)*Q.at<double>(0,0)) + ((float) Q.at<double>(0,3));
    point.y=(float)((float) (vrect1+1)*Q.at<double>(1,1)) + ((float) Q.at<double>(1,3));
    point.z=(float) Q.at<double>(2,3);

    point.x=point.x/w;
    point.y=point.y/w;
    point.z=point.z/w;

   if(drive=="LEFT") {
        Mat P(3,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;

        P=this->stereo->getRLrect().t()*P;

        point.x=(float) P.at<double>(0,0);
        point.y=(float) P.at<double>(1,0);
        point.z=(float) P.at<double>(2,0);
    }
    if(drive=="RIGHT") {
        Mat Rright = this->stereo->getRotation();
        Mat Tright = this->stereo->getTranslation();
        Mat RRright = this->stereo->getRRrect().t();
        Mat TRright = Mat::zeros(0,3,CV_64F);

        Mat HRL=buildRotTras(Rright,Tright);
        Mat Hrect=buildRotTras(RRright,TRright);

        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;
       
        P=Hrect*HRL*P;
        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));

    }

    if(drive=="ROOT") {
        Mat RLrect=this->stereo->getRLrect().t();
        Mat Tfake = Mat::zeros(0,3,CV_64F);
        Mat P(4,1,CV_64FC1);
        P.at<double>(0,0)=point.x;
        P.at<double>(1,0)=point.y;
        P.at<double>(2,0)=point.z;
        P.at<double>(3,0)=1;

        Mat Hrect=buildRotTras(RLrect,Tfake);
        P=HL_root*Hrect*P;
        point.x=(float) ((float) P.at<double>(0,0)/P.at<double>(3,0));
        point.y=(float) ((float) P.at<double>(1,0)/P.at<double>(3,0));
        point.z=(float) ((float) P.at<double>(2,0)/P.at<double>(3,0));
    }
    this->mutexDisp->post();
    return point;
}

Mat SFM::buildRotTras(Mat & R, Mat & T) {
        
        Mat A = Mat::eye(4, 4, CV_64F);
        for(int i = 0; i < R.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = R.ptr<double>(i);
            for(int j = 0; j < R.cols; j++)
                 Mi[j]=MRi[j];
         }
        for(int i = 0; i < T.rows; i++)
         {
         double* Mi = A.ptr<double>(i);
         double* MRi = T.ptr<double>(i);
                 Mi[3]=MRi[0];
         }
        return A;
}

Matrix SFM::getCameraHGazeCtrl(int camera) {

    yarp::sig::Vector x_curr;
    yarp::sig::Vector o_curr;

    if(camera==LEFT)
        igaze->getLeftEyePose(x_curr, o_curr);
    else
        igaze->getRightEyePose(x_curr, o_curr);

    Matrix R_curr=axis2dcm(o_curr);

    Matrix H_curr(4, 4);
    H_curr=R_curr;
    H_curr(0,3)=x_curr[0];
    H_curr(1,3)=x_curr[1];
    H_curr(2,3)=x_curr[2];

    if(camera==LEFT)
    {
        this->mutexDisp->wait();
        convert(H_curr,HL_root);
        this->mutexDisp->post();
    }
    else if(camera==RIGHT)
    {
        this->mutexDisp->wait();
        convert(H_curr,HR_root);
        this->mutexDisp->post();
    }


    return H_curr;
}

void SFM::convert(Matrix& matrix, Mat& mat) {
    mat=cv::Mat(matrix.rows(),matrix.cols(),CV_64FC1);
    for(int i=0; i<matrix.rows(); i++)
        for(int j=0; j<matrix.cols(); j++)
            mat.at<double>(i,j)=matrix(i,j);
}

void SFM::convert(Mat& mat, Matrix& matrix) {
    matrix.resize(mat.rows,mat.cols);
    for(int i=0; i<mat.rows; i++)
        for(int j=0; j<mat.cols; j++)
            matrix(i,j)=mat.at<double>(i,j);
}

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("icubEyes.ini"); 
    rf.setDefaultContext("cameraCalibration/conf");   
    rf.configure("ICUB_ROOT",argc,argv);

    SFM mod;

    return mod.runModule(rf);
}

