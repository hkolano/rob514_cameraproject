% Auto-generated by cameraCalibrator app on 10-Nov-2020
%-------------------------------------------------------


% Define images to process
imageFileNames = {'DSC_0344.JPG',...
    'DSC_0345.JPG',...
    'DSC_0346.JPG',...
    'DSC_0347.JPG',...
    'DSC_0348.JPG',...
    'DSC_0349.JPG',...
    'DSC_0350.JPG',...
    'DSC_0351.JPG',...
    'DSC_0354.JPG',...
    'DSC_0355.JPG',...
    'DSC_0356.JPG',...
    'DSC_0357.JPG',...
    'DSC_0358.JPG',...
    'DSC_0360.JPG',...
    'DSC_0361.JPG',...
    'DSC_0362.JPG',...
    };
% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 10;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

save 'NikonParams.mat' cameraParams
save 'NikonEstimationErrors.mat' estimationErrors

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
