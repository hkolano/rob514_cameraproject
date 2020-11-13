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
left_img = imread('Pictures/FrontCam-Nikon/Right1.JPG');
right_img = imread('Pictures/FrontCam-Nikon/Left1.JPG');

% Show undistorted images
% figure
% imshowpair(nikon_img, canon_img, 'montage'); 
% title('Original Images');

left_img = undistortImage(left_img, nikonCamParams);
right_img = undistortImage(right_img, nikonCamParams);
figure 
imshowpair(left_img, right_img, 'montage');
title('Undistorted Images');

% Detect feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(left_img), 'MinQuality', 0.1);
imagePoints2 = detectMinEigenFeatures(rgb2gray(right_img), 'MinQuality', 0.1);

% Visualize detected points
figure
subplot(1,2,1)
imshow(nikon_img, 'InitialMagnification', 50);
title('150 Strongest Corners from the First Image');
hold on
plot(selectStrongest(imagePoints1, 150));

% Visualize detected points
subplot(1,2,2)
imshow(canon_img, 'InitialMagnification', 50);
title('150 Strongest Corners from the Second Image');
hold on
plot(selectStrongest(imagePoints2, 150));

%Track points
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, left_img);

% Track the points
[imagePoints2, validIdx] = step(tracker, right_img);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Visualize correspondences
figure
showMatchedFeatures(left_img, right_img, matchedPoints1, matchedPoints2);
title('Tracked Features');