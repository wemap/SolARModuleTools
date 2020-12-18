# SolARModuleTools

[![License](https://img.shields.io/github/license/SolARFramework/SolARModuleTools?style=flat-square&label=License)](https://www.apache.org/licenses/LICENSE-2.0)

**SolARModuleTools** is a module that implements some generic features used in vision processing, such as 2D or 3D transformations, matches between keypoints, homography checking, keyframe searching, 3D points filtering, etc.

SolARModuleTools is open-source, designed by [b<>com](https://b-com.com/en), under [Apache v2 licence](https://www.apache.org/licenses/LICENSE-2.0).

# Before running the tests

Some tests require external data which has to be download before executing them.

First, download the vocabularies required for the bag of words available on the GitHub, and extract the akaze.fbow file and copy it in the data folder.


# The tests

## SolAR Test 3D Transform Estimation using RANSAC

Given two sets of 3D-3D point correspondences with noisy data, this test aims at estimating the 3D transformation (7-DoF) between them and defining inlier correspondences.