%{ 
Object ID project for ROB 514
Team: Hannah Kolano, Aiden Shaevitz, Natasha Troxler

Code purpose: use image processing toolbox to determine radius of soft
actuator from a picture.

Dependencies:
Matlab Image Processing Toolbox

Last modified by Hannah Kolano 10/31/2020
%}
clc
clearvars

%% Setup: Read in images
% read in the image with the ruler
ruler_pic = imread('Pictures/ScalingRuler.JPG');

% read in the image of the actuator
actuator_pic_1 = imread('Pictures/DSC_0275.JPG');
% imshow(actuator_pic_1) % shows image in window
% whos actuator_pic_1 % prints info about image

%% Pre-process images
% Turn the image to grayscale
gray_image = rgb2gray(actuator_pic_1);
% figure
% imshow(gray_image)
% title('Grayscale version of image')

% Make the image binary
threshold = 0.65; % set threshold for creating binary image
binary_image = imbinarize(gray_image, threshold);
% figure
% imshow(binary_image)
% title('Binary version of image')

% Fill in holes
binary_filled = imfill(binary_image,'holes');
% figure
% imshow(binary_filled)
% title('Binary image filled with imfill')

%% Find objects 
% labels all the individual objects (regions of filled vs not filled)
[L, n] = bwlabel(binary_filled);
% get the object properties
stats = regionprops('table', L, 'Area', 'FilledImage');
% Find the largest object, which is the actuator, and the index it exists
% at
% TODO: maybe also check object circularity to make sure it's the actuator
[MaxArea, MaxAreaIndex] = max(stats.Area);
% Get just the object in an image (needs to get out of cell struct)
isolated_actuator_img = cell2mat(stats.FilledImage(MaxAreaIndex));
figure
imshow(isolated_actuator_img)
title('Largest object image')

%% Analyze the isolated object
objstats = regionprops('table', isolated_actuator_img, 'MajoraxisLength', 'MinoraxisLength', 'Orientation', 'Centroid');
majAxisLength = objstats.MajorAxisLength;
minAxisLength = objstats.MinorAxisLength;
center = objstats.Centroid;

% TODO: use the centroid and axes to construct the ellipse




