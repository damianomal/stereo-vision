# **Last updates to the DisparityModule**

The following is a list of the last updates to the `DisparityModule` code, which has been refactored a little with respect to its original state under the name of `SFM`. The GUI proposed in the previous iterations has changed, so to avoid additional dependencies to be added to the `stereo-vision` repository.

These items are not the full list, but give a general overview of the last changes made to the code.

---

## **Code Refactoring**

In the previous versions of the SFM module, a few functionalities were spread around in the code. The `StereoCamera` also carried out the stereo matching phase, making the code more bloated than needed.

Things were moved a little, with the following organization:

- `StereoCamera`: handles everything related to the geometry of the stereo system (as before), has been cleaned of everything related to the computation of the disparity maps
- `DisparityModule`: this module implements the main logic driving the computation of the disparity map, exposing the ports through which an user can obtain both the disparity and depth maps. This is the only object which interfaces itself with the (optional) graphical user interface.
- `StereoMatcherNew`: is the newly included element which job is to carry out the computation and post-processing operations of the disparity map, by getting the rectified images directly from the `StereoCamera` being used.

---

## **Handling of the Stereo Parameters**

The stereo parameters were [directly enforced](https://github.com/robotology/stereo-vision/blob/9cbc03ae5d7983bc908d2089abfb5cfae5e55f42/modules/SFM/SFM.cpp#L28) within the code at startup. This choice didn't allow flexibility, for an user which wanted to change the default values had to recompile the whole module.

The stereo parameters are now saved within the configuration file `SFM.ini`, and are loaded and saved at runtime. In case the parameters are not available in the configuration file (as in the case you run this new module on a PC for the first time), the usual default values are loaded.

(in addition, the name of the `SFM.ini` file should be updated to match the new name of the module)

---

## **GUI**

The former graphical interface based on `imgui` has been replaced by [another one](https://github.com/Dovyski/cvui) which only exploits OpenCV's draw primitives to draw the GUI and to handle the interaction with the user.

The choice to maintain a GUI is due to the non-trivial amount of fine-tuning which might be needed to carry out every time the DisparityModule is run.

The components (and the code related to) the GUI can be disabled by means of a toggle in the CMake configuration, named `USE_DISPMODULE_GUI`. Disabling it, takes away the single header file representing the GUI base code, together with all the other bits of source related to it. Thus, compiling the module without the GUI included actually removes everything about it from the final result.

<p align="center">
  <img src="gui.png">
</p>

The GUI makes it now easier to recalibrate the stereo system, to save the calibration, to load default stereo parameters and to save the current ones. Once it is not useful anymore, it can be quitted, closing it for the rest of the execution.

---

## **Support connection script**

The code is now shipped with a `connect.sh` bash script, which simply connects the output camCalib ports to the ones used by the `DisparityModule` as input for the left and right images. 

In case the user invokes the `connect.sh` script with an additional parameter `port_name`, the script also connects the disparity output port of the module to the port `port_name`.

---

## **Inserting a novel stereo matching algorithm**

It is possible to add a new stereo matching method by editing the content of just a few files. These are the steps to follow:

1. file `StereoMatcher.h`:

   - add a new function definition to be associated with the new matching method, i.e.:
   ```cpp
   void matchMethodX();

   ```

   - add a new item to the enumeration used to represent the different matching algorithms available:

   ```cpp
   enum SM_MATCHING_ALG {
       SGBM_OPENCV = 0,
       SGBM_CUDA,
       LIBELAS,
       METHODX
   };
   ```

2. file `StereoMatcher.cpp`:

   - implement the corresponding matching method, which might also rely on external libraries, or have additional dependencies (which handling is not described here, since it's specific to the new algorithm included):

   ```cpp
   void StereoMatcherNew::matchMethodX()
   {
     [...]
     // code executing the matching
     [...]
     this->disparity16 = resulting_disparity;
   }

   ```

3. file `DisparityModule.cpp`:

   - enrich the content of [line 246](https://github.com/damianomal/stereo-vision/blob/01ba763f6220d9bdcea31aaa50fd88538154a455/modules/DispModule/DispModule.cpp#L246), with the new algorithm available, adding a new **keyword** to select it at configuration time:

   ```cpp
   if (rf.check("sgbm"))
      this->stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
   [...]
   else if(rf.check("methodX"))
      this->stereo_matching = SM_MATCHING_ALG::METHODX;
   ```

4. file `StereoMatcher.cpp`:

   - add the handling of the new matching algorithm in the `switch` within the `StereoMatcherNew::compute()` function:

   ```cpp
   case SM_MATCHING_ALG::METHODX:
      this->matchMethodX();
      break;
   ```

The following additional step is optional, and meant to be considered only if you plan to use this module with also the GUI functionalities:

1. (optional) file `cvgui.cpp`:

   - add a new checkbox to the radioButtons call at [line 114](https://github.com/damianomal/stereo-vision/blob/01ba763f6220d9bdcea31aaa50fd88538154a455/modules/DispModule/cvgui.cpp#L114):

   ```cpp
   this->updated |= cvuimine::radioButtons(frame, "Stereo Matching Alg.:", {"SGBM", "SGBM_CUDA", "LibElas", "MethodX"}, {20, 90, 190, 260}, 0);
   ```

   **NOTE**: you might want to enlarge the GUI window, since now there are more options in the radio buttons group associated with the choice of the stereo matching algorithm, this can be done by changing [line 60](https://github.com/damianomal/stereo-vision/blob/01ba763f6220d9bdcea31aaa50fd88538154a455/modules/DispModule/cvgui.cpp#L60) of the file:

   ```cpp
   int gui_width = 450;   
   ```

   the value 450 is the one specified for the width, when initially designed. Increasing this value will make the window a little bigger, correctly fitting your needs.

   - adding the new algorithm to the `switch` construct in `GUI::convertIDTOEnum()`:

   ```cpp
   switch(this->stereo_matching_id)
   {
       [...]
       case 3:
           this->params.stereo_matching = SM_MATCHING_ALG::METHODX;
           break;
   }
   ```

   - adding the new algorithm to the `switch` construct in `GUI::convertEnumToID()`:

   ```cpp
   switch(this->params.stereo_matching)
   {
       [...]
       case SM_MATCHING_ALG::METHODX:
           this->stereo_matching_id = 3;
           break;
   }
   ```
---

## **Known (negligible) issues**

A few things are not complete (however, they don't compromise the functionality of the module), and still require a quick correction each one:

   1. the correction of the implementation of the left-right consistency for the WLS filtering
   2. saving and loading the disparity map refinement threshold from the configuration file
   3. the original bilateral filtering is not working as supposed to, since when the code has been moved on the iCub PC, the filtering is run but it looks like that the sigmaColor and sigmaSpace parameters are ignored. This is not critical at all, since that filtering process is very slow, and it can be substituted with the CUDA implementation of the bilateral filtering, which is now available. however this issue should be fixed at some point

---
# OLD README.MD SECTION

Stereo Vision
============

A C++ [YARP](https://github.com/robotology/yarp) library that provides the **iCub** robot with stereo vision capabilities.

[![ZenHub](https://img.shields.io/badge/Shipping_faster_with-ZenHub-435198.svg)](https://zenhub.com)

## Installation

##### Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [OpenCV](http://opencv.org/downloads.html) (**>=2.4** and with **SIFT** features available)
- [SiftGPU](http://cs.unc.edu/~ccwu/siftgpu) (optional but suggested for better performance)

We also make use of [libelas](http://www.cvlibs.net/software/libelas/) for computing
disparity maps, but there is no need to download it, since it is embedded inside the code.

## Documentation

Online documentation is available here: [http://robotology.github.com/stereo-vision](http://robotology.github.com/stereo-vision).

## License

Material included here is Copyright of _iCub Facility - Istituto Italiano di Tecnologia_ and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.
