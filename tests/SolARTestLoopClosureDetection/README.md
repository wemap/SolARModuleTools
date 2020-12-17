SolAR Test Loop closure detection
=============

This test uses the prebuilt map to try to detect a loop closure for the last keyframe. 
If a loop closure is detected, it will transform local point cloud of the last keyframe to the coordinate system of the loop detected keyframe.

## How to run

* :warning: Don't forget to download the [fbow vocabulary](https://github.com/SolarFramework/binaries/releases/download/fbow%2F0.0.1%2Fwin/fbow_voc.zip) unzip this archive and put the `akaze.fbow` in ``**./data**`` directory.

* Open a terminal and execute: `SolARTestLoopClosureDetection.exe`

## Contact 

*   Website https://solarframework.github.io/

*   Contact framework.solar@b-com.com
