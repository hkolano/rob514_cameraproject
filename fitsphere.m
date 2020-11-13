%{
Camera project for ROB 514
Team: Hannah Kolano, Aiden Shaevitz, Natasha Troxler

Code purpose: use two images from two cameras to reconstruct a 3D sphere.

Dependencies:
Matlab Computer Vision Toolbox


Last modified by Natasha Troxler 11/12/2019

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

[R, t] = cameraPose(fMatrix, nikonCamParams, inlierPoints1, inlierPoints2)

% Detect dense feature points
imagePoints1 = detectMinEigenFeatures(rgb2gray(left_img), 'MinQuality', 0.001);

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, left_img);

% Track the points
[imagePoints2, validIdx] = step(tracker, right_img);
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Compute the camera matrices for each position of the camera
% The first camera is at the origin looking along the X-axis. Thus, its
% rotation matrix is identity, and its translation vector is 0.
camMatrix1 = cameraMatrix(nikonCamParams, eye(3), [0 0 0]);
camMatrix2 = cameraMatrix(nikonCamParams, R', -t*R');

% Compute the 3-D points
points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

% Get the color of each reconstructed point
numPixels = size(left_img, 1) * size(left_img, 2);
allColors = reshape(left_img, [numPixels, 3]);
colorIdx = sub2ind([size(left_img, 1), size(left_img, 2)], round(matchedPoints1(:,2)), ...
    round(matchedPoints1(:, 1)));
color = allColors(colorIdx, :);

% Create the point cloud
ptCloud = pointCloud(points3D, 'Color', color);

% Visualize the camera locations and orientations
cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', t, 'Orientation', R, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);

% Visualize the point cloud
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);

% Rotate and zoom the plot
camorbit(0, -30);
camzoom(1.5);

% Detect the globe
globe = pcfitsphere(ptCloud, 0.1);

% Display the surface of the globe
plot(globe);
title('Estimated Location and Size of the Globe');
hold off

% Label the axes
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis')

title('Up to Scale Reconstruction of the Scene');
