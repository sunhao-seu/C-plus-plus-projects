Version1.0 of HW3_SIMPLE_BA:
1: Read files to get the data of rgb, depth and groundtruth.
2: Compare consequent images , obtain the ORB keypoints and matched it.
3: Using 7-Point RANSAC to remove the outliers.
4: By getting the 3D points, use opencv finction PnPsolve to get the R,T
5:  Store the  feature 3D points, camera pose, correspondance pixel keypoints as a vector of struct, Prepare for g2o
6: After obtainong ten poses, construct the G2O vertexs and edges, optimize the trajactory by BA;

Problem:
1: need to store 2D keyypoints corres to camera pose to Struct;
2: cannot GET TRUE BA result.
3: Cannot visualize my G2O file.


This project is a cross-platform project under Visual Studio 2017. And compiled on Ubuntu16.04;
This project mainly used OPENCV library and G2O library.
Again, I put many props files in the peoject for a guide on library include and link.