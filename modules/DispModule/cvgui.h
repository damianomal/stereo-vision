#ifndef _GUI_H_
#define _GUI_H_

#include "StereoMatcher.h"
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// the struct containing the parameters handled by the GUI



class GUI
{

private:

    // flag set to True if the interface has been quitted
    bool done;

    // flag set to True if the interface has been used/updated in the last timeframe
    bool updated;

    // flag set to True if the user asked the system to recalibrate the stereo system
    bool recalibrate;

    // flag set to True if the user clicked the button to save the current calibration parameters
    bool save_calibration;

    // flag set to True if the user chose to revert the matching/filtering parameters to their default values
    bool load_parameters;

    // flag set to True if the user selected
    bool save_parameters;

    // the parameters handled by the GUI
    Params params;

    // the OpenCV Mat object where to draw the interface
    cv::Mat frame;

    // stereo matching parameters and auxiliary variables
    SM_BLF_FILTER BLFfiltering;
    SM_WLS_FILTER WLSfiltering;
    SM_MATCHING_ALG stereo_matching;

    int BLFfiltering_id;
    int WLSfiltering_id;
    int stereo_matching_id;
    int num_disparities_id;

    int refine_th;

public:

    /**
    * Kills the GUI and deletes the no more needed objects
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
    * Runs the GUI main update loop
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
    * XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    * @param XXX XXXXXXXXXXXXXXX
    *
    */
    void setParams(int& minDisparity, int& numberOfDisparities, int& SADWindowSize,
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
     * Resets the GUI internal update flags
     *
     */
    void resetState();

    /**
    * Sets the state of the interface to the one specified
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
    * Checks whether the user wants to reload the stereo parameters from file
    * @return True if the Default Param. button has been pressed, False otherwise
    *
    */
    bool toLoadParameters();

    /**
    * Checks whether the user wants to save the stereo parameters to file
    * @return True if the Ssve Parameters button has been pressed, False otherwise
    *
    */
    bool toSaveParameters();

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


    bool toRefine();

    int getRefineTh();

    GUI();
    ~GUI();


};

#endif // _GUI_H_
