%{ 
Camera project for ROB 514
Team: Hannah Kolano, Aiden Shaevitz, Natasha Troxler

Code purpose: use two images from two cameras to reconstruct a 3D sphere.

Dependencies:
Matlab Computer Vision Toolbox

Last modified by Hannah Kolano 11/11/2020

%}

% Load the camera parameters from the calibration
load('CameraCalibration/Nikon/NikonParams.mat')
nikonCamParams = cameraParams;

load('CameraCalibration/Canon/CanonParams.mat')
canonCamParams = cameraParams;

% Load the two actuator images from the different cameras
nikon_img = imread('Pictures/FrontCam-Nikon/DSC_0370.JPG');
canon_img = imread('Pictures/SideCam-Canon/IMG_0418.JPG');

% Show undistorted images
% figure
% imshowpair(nikon_img, canon_img, 'montage'); 
% title('Original Images');

% nikon_img = undistortImage(nikon_img, nikonCamParams);
% canon_img = undistortImage(canon_img, canonCamParams);
% figure 
% imshowpair(nikon_img, canon_img, 'montage');
% title('Undistorted Images');

% Detect feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(nikon_img), 'MinQuality', 0.1);

% Visualize detected points
figure
imshow(nikon_img, 'InitialMagnification', 50);
title('150 Strongest Corners from the First Image');
hold on
plot(selectStrongest(imagePoints1, 150));
