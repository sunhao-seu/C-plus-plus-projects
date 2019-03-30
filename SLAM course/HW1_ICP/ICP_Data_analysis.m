clc;
clear;

% ICP_P2P_transformation_store_PCL_LIBRARY.txt
% ICP_P2P_transformation_store
% ICP_P2L_transformation_store
Ground_truth_transformation_store = load("E:\Code\VS code\SLAM_HW1_ICP\SLAM_HW1_ICP\Ground_truth_transformation_store.txt");
ICP_P2P_transformation_store = load("E:\Code\VS code\SLAM_HW1_ICP\SLAM_HW1_ICP\ICP_P2L_transformation_store.txt");
[G_row,G_column] = size(Ground_truth_transformation_store);

% change Relative tran\sform to absolute psoes;
% R_(w,n) = R_(w,n-1)*R_(n-1,n)
% t_(n) = t_(n-1)+R_(w,n))t_(n)
% 
ICP_P2P_poses = zeros(G_row,G_column);
ICP_P2P_last_pose= Ground_truth_transformation_store(1:4,1:4);
for i = 1:4:(G_row-4)
    ICP_P2P_poses(i:i+3,1:4) = ICP_P2P_last_pose;
    Transform_Matrix = ICP_P2P_transformation_store(i:i+3,1:4);
    % ICP_P2P_last_pose = ICP_P2P_last_pose*Transform_Matrix;
    ICP_P2P_last_pose = Ground_truth_transformation_store(i:i+3,1:4)*Transform_Matrix';
end
ICP_P2P_poses(G_row-3:G_row,1:4) = ICP_P2P_last_pose;

count = 1;
Ground_truth_X = zeros(G_row/4,1);
Ground_truth_Y = Ground_truth_X;
Ground_truth_Z = Ground_truth_X;
ICP_Calculate_X = Ground_truth_X;
ICP_Calculate_Y = Ground_truth_X;
ICP_Calculate_Z = Ground_truth_X;
for i=1:4:G_row
Ground_truth_X(count) = Ground_truth_transformation_store(i,4);
Ground_truth_Y(count) = Ground_truth_transformation_store(i+1,4);
Ground_truth_Z(count) = Ground_truth_transformation_store(i+2,4);
ICP_Calculate_X(count) = ICP_P2P_poses(i,4);
ICP_Calculate_Y(count) = ICP_P2P_poses(i+1,4);
ICP_Calculate_Z(count) = ICP_P2P_poses(i+2,4);
count= count+1;
end

figure(1);
%scatter3(Ground_truth_X,Ground_truth_Y,Ground_truth_Z,'o');
plot3(Ground_truth_X,Ground_truth_Y,Ground_truth_Z,'b*');
hold on;
%scatter3(ICP_Calculate_X,ICP_Calculate_Y,ICP_Calculate_Z,'*');
plot3(ICP_Calculate_X,ICP_Calculate_Y,ICP_Calculate_Z,'r-');
legend("GroundTruth","MyICPCalculate");
title("ICP\_P2L result with GroundTruth");
hold off;


Add functions:
1: Read ground _truth file and return the time_stamp and absolute poses[ transform quaternion to homogenous coordinate
2: Get related groundtruth from indice[timestamp] to verify the ICP result.
3:Change some code to output Absolute pose and ICP transform pose.
4: .m is the MATLAB verify file.