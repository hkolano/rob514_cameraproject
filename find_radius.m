%{ 
Object ID project for ROB 514
Team: Hannah Kolano, Aiden Shaevitz, Natasha Troxler

Code purpose: use image processing toolbox to determine radius of soft
actuator from a picture.

Dependencies:
Matlab Image Processing Toolbox

Last modified by Aiden Shaevitz 11/03/2020
%}
clc
clearvars

%% Setup: Read in images
% read in the image with the ruler
% ruler_pic = imread('Pictures/ScalingRuler.JPG');
% imshow(ruler_pic)

% pixels on ruler separated by 15cm
pixel1 = [868 2540];
pixel2 = [3630 2545];
% relate cm / pixel
ratio = 15 / (3630-868);

% read in the image of the actuator
actuator_pic_1 = imread('Pictures/DSC_0276.JPG');
% imshow(actuator_pic_1) % shows image in window
% whos actuator_pic_1 % prints info about image

%% Pre-process images
% Turn the image to grayscale
gray_image = rgb2gray(actuator_pic_1);
imwrite(gray_image, 'Progression/GrayImage.png')
% figure
% imshow(gray_image)
% title('Grayscale version of image')

% Make the image binary
threshold = 0.65; % set threshold for creating binary image
binary_image = imbinarize(gray_image, threshold);
imwrite(binary_image, 'Progression/BinaryImage.png')
% figure
% imshow(binary_image)
% title('Binary version of image')

% Fill in holes
binary_filled = imfill(binary_image,'holes');
imwrite(binary_filled, 'Progression/BinaryFilled.png')
% figure
% imshow(binary_filled)
% title('Binary image filled with imfill')

%% Find objects 
% labels all the individual objects (regions of filled vs not filled)
[L, n] = bwlabel(binary_filled);
% get the object properties
stats = regionprops('table', L, 'Area', 'FilledImage');
% Find largest object, which is the actuator, and the index it exists at
% TODO: maybe also check object circularity to make sure it's the actuator
[MaxArea, MaxAreaIndex] = max(stats.Area);
% Get just the object in an image (needs to get out of cell struct)
isolated_actuator_img = cell2mat(stats.FilledImage(MaxAreaIndex));
imwrite(isolated_actuator_img, 'Progression/IsolatedActuator.png')

%% Analyze the isolated object
objstats = regionprops('table', isolated_actuator_img, 'MajoraxisLength', 'MinoraxisLength', 'Orientation', 'Centroid', 'Extrema');
majAxisLength = objstats.MajorAxisLength;
minAxisLength = objstats.MinorAxisLength;
center = objstats.Centroid;
orientation = objstats.Orientation;
extrema = cell2mat(objstats.Extrema);
% Extrema pixels
[minPixel_x, minPixelIndex] = min(extrema(:,1));
[maxPixel_x, maxPixelIndex] = max(extrema(:,1));
minPixel_y = extrema(minPixelIndex, 2);
maxPixel_y = extrema(maxPixelIndex, 2);

% Width in pixels of object
pixelWidth_r = sqrt((maxPixel_x - minPixel_x)^2 + (maxPixel_y - minPixel_y)^2);
% Diameter (cm)
physicalWidth = pixelWidth_r * ratio;
% Radius (cm)
radius = physicalWidth / 2;

%% Plot object (unrotated)
% Display the actuator
figure; subplot(2, 4, [1 2 5 6]); hold on
imshow(isolated_actuator_img)
plot([minPixel_x maxPixel_x], [minPixel_y maxPixel_y], 'r', 'LineWidth', 5)
title('Unrotated Actuator')
xlabel(['Radius of inflated actuator = ', num2str(round(radius, 3)), 'cm'])

%%  Construct ellipse from object analysis
theta = linspace(0, 2*pi);
col = (majAxisLength/2) * cos(theta);
row = (minAxisLength/2) * sin(theta);
transformMatrix = makehgtform('translate',[center, 0],'zrotate',deg2rad(-1*orientation));
ellipse = transformMatrix * [col; row; zeros(1, numel(row)); ones(1, numel(row))];
% plot(ellipse(1,:), ellipse(2,:),'r','LineWidth',3)


%% Rotate, repeat and plot again
% Determine rotation direction
if orientation > 0
    rotated = imrotate(isolated_actuator_img, (90-orientation));
else
    rotated = imrotate(isolated_actuator_img, (-90-orientation));
end
% Object analysis
objstats_r = regionprops('table', rotated, 'Extrema', 'Orientation');
extrema_r = cell2mat(objstats_r.Extrema);
[minPixel_xr, minPixelIndex_r] = min(extrema_r(:,1));
[maxPixel_xr, maxPixelIndex_r] = max(extrema_r(:,1));
minPixel_yr = extrema_r(minPixelIndex_r, 2);
maxPixel_yr = extrema_r(maxPixelIndex_r, 2);

%Object width in pixels
pixelWidth_r = sqrt((maxPixel_xr - minPixel_xr)^2 + (maxPixel_yr - minPixel_yr)^2);
% Diameter (cm)
physicalWidth_r = pixelWidth_r * ratio;
% Radius (cm)
radius_r = physicalWidth_r / 2;
% Plot rotated object
subplot(2,4,[3 4 7 8]); hold on
imshow(rotated)
plot([minPixel_xr maxPixel_xr], [minPixel_yr maxPixel_yr], 'r', 'LineWidth', 5)
title('Rotated Actuator')
xlabel(['Radius of inflated actuator = ', num2str(round(radius_r, 3)), 'cm'])
saveas(gcf, 'Progression/Radius.png')

