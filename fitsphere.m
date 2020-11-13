%{ 
Camera project for ROB 514
Team: Hannah Kolano, Aiden Shaevitz, Natasha Troxler

Code purpose: use two images from two cameras to reconstruct a 3D sphere.

Dependencies:
Matlab Computer Vision Toolbox

Last modified by Aiden Shaevitz 11/12/2020

%}

%% Load camera parameters from calibration files
load('CameraCalibration/Nikon/NikonParams.mat')
nikonCamParams = cameraParams;

load('CameraCalibration/Canon/CanonParams.mat')
canonCamParams = cameraParams;

%% Load images from both cameras
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

%% Feature point detection
% Detect feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(left_img), 'MinQuality', 0.1);
imagePoints2 = detectMinEigenFeatures(rgb2gray(right_img), 'MinQuality', 0.1);


% Visualize detected points for nikon and canon images
figure
subplot(1,2,1)
imshow(right_img, 'InitialMagnification', 50);
title('150 Strongest Corners from the First Image');
hold on
plot(selectStrongest(imagePoints1, 150));


% Visualize detected points
subplot(1,2,2)
imshow(left_img, 'InitialMagnification', 50);
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

% Estimate the fundamental matrix
[fMatrix, epipolarInliers] = estimateFundamentalMatrix(...
  matchedPoints1, matchedPoints2, 'Method', 'MSAC', 'NumTrials', 10000);

% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);

% Display inlier matches
figure
showMatchedFeatures(left_img, right_img, inlierPoints1, inlierPoints2);
title('Epipolar Inliers');
