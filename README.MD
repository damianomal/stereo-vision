
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

## **GUI**

The former graphical interface based on `imgui` has been replaced by [another one](https://github.com/Dovyski/cvui) which only exploits OpenCV's draw primitives to draw the GUI and to handle the interaction with the user.

The choice to maintain a GUI is due to the non-trivial amount of fine-tuning which might be needed to carry out every time the DisparityModule is run.

The components (and the code related to) the GUI can be disabled by means of a toggle in the CMake configuration, named `USE_DISPMODULE_GUI`. Disabling it, takes away the single header file representing the GUI base code, together with all the other bits of source related to it. Thus, compiling the module without the GUI included actually removes everything about it from the final result.

[insert missing GUI sample image here]

The GUI makes it now easier to recalibrate the stereo system, to save the calibration, to load default stereo parameters and to save the current ones. Once it is not useful anymore, it can be quitted, closing it for the rest of the execution.

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
     this->disparity = resulting_disparity;
   }

   ```

3. file `DisparityModule.cpp`:

   - enrich the content of line 203 [TODO: add hyperlink here], with the new algorithm available, adding a new **keyword** to select it at configuration time:

   ```cpp
   if (rf.check("sgbm"))
      this->stereo_matching = SM_MATCHING_ALG::SGBM_OPENCV;
   [...]
   else if(rf.check("methodX"))
      this->stereo_matching = SM_MATCHING_ALG::METHODX;
   ```

The following additional step is optional, and meant to be considered only if you plan to use this module with also the GUI functionalities:

4. (optional) file `cvgui.cpp`:

   - add a new checkbox to the radioButtons call at line 119 [TODO add hyperlink here]:

   ```cpp
   this->updated |= cvuimine::radioButtons(frame, "Stereo Matching Alg.:", {"SGBM", "SGBM_CUDA", "LibElas", "MethodX"}, {20, 90, 190, 260}, 0);
   ```

   **NOTE**: you might want to enlarge the GUI window, since now there are more options in the radio buttons group associated with the choice of the stereo matching algorithm, this can be done by changing line 65 [TODO hyperlink here] of the file:

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