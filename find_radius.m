%{ 
Object ID project for ROB 514
Team: Hannah Kolano, Aiden Shaevitz, Natasha Troxler

Code purpose: use image processing toolbox to determine radius of soft
actuator from a picture.

Last modified by Hannah Kolano 10/31/2020
%}

ruler_pic = imread('Pictures/ScalingRuler.JPG');
% imshow(ruler_pic)
whos ruler_pic

actuator_pic_1 = imread('Pictures/DSC_0275.JPG');
imshow(actuator_pic_1)