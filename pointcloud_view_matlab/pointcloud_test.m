% Simple Matlab script for viewing 3D pointcloud files in .PLY format
clear;
ptCloudColor = pcread('exportColor.ply');
pcshow(ptCloudColor);
figure();
ptCloudDepth = pcread('exportDepth.ply');
pcshow(ptCloudDepth);