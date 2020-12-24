%- World Frame to Camere Frame - Extrinsic
clear all; clc; close all;
%position in world coordinates
fprintf('Position with respect to world frame: ');
P_w = [0.05; 0.03; 0.3]

% world frame center position 
% with respect to camera frame
Wo_cf = [0.1; 0.2; 1]; %in meters

theta = 12*pi/180;
% calculation of R|T matrix
R = [cos(theta) -sin(theta) 0;
     sin(theta)  cos(theta) 0;
     0              0       1];
R = [R Wo_cf];
R_T = [R; 0 0 0 1];

% add 1 to world cordinates for homogeneousity
P_w = [P_w; 1];
% Position with respect to camera frame
P_cf = R_T*P_w;
P_cf(4) = [];
fprintf('Position with respect to camera frame: ');
P_cf

%- Camera Frame to Image Plane - Intrinsic
plane_size = 0.016; %in meters
pixel_num  = 500;
pixel_size = plane_size / pixel_num; %number of pixels
focal_length = 0.024; %in meters

% central pixel -> (u0, v0)
u0 = 250; % in pixels
v0 = 250; % in pixels

% calculate scale factor for discretization
k  = 1/pixel_size;
alpha = k*focal_length;
K = [alpha      0       u0;
     0          alpha   v0;
     0          0       1];
% We obtained coordinates of the position wrt camera frame in part 1
% now calculate the pixel points
Pixel_xy = K*P_cf;
lambda = Pixel_xy(3);
u = round(Pixel_xy(1)/lambda);
v = round(Pixel_xy(2)/lambda);
fprintf('Position in image plane (in pixels): (%d,%d) \n', u,v);



