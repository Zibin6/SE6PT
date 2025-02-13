
%% Code demo for pose tracking of uncooperative spacecraft
% Zibin Liu, Banglei Guan, Yang Shang, Yifei Bian, Pengju Sun, and Qifeng Yu
% "Stereo Event-Based, 6-DOF Pose Tracking for Uncooperative Spacecraft"
% In: IEEE TRANSACTIONS ON GEOSCIENCE AND REMOTE SENSING

close all; clc; clear;
addpath('func\');

%% Load data

% Load the event stream from the left event camera
load("l.mat")
% Load the event stream from the right event camera
load("r.mat")

% Load left camera intrinsic parameters
K1=[320,0,320;0,320,240;0,0,1];

% Load right camera intrinsic parameters
K2=[320,0,320;0,320,240;0,0,1];

% The initial pose of the left camera
pose_cur_l=[0.3980,-0.9174,0,-2.7549;-0.9174,-0.3980,0,-1.3076;0,0,-1.0000,5.0000];

% Extrinsic parameters between left and right cameras
R2L=[1,0,0,-0.3;0,1.0000,0,0;0,0,1,0;0,0,0,1];

% The initial pose of the right camera
pose_cur_r=R2L(1:3,:)*[pose_cur_l;0 0 0 1];

% Load a wireframe line model represented by two endpoints
load("Line_model.mat")
%% Load parameter

% Time interval within each window
event_t=0.01;

% Number of events within each window
event_n=4000;

% The distance threshold of Event-Line Matching
event_d=3;

%% Pose tracking
for t=L(1,1):event_t:L(end,1)

    if t<(L(end,1)-event_t)

        % Event - line matching
        [event_cluster_l event_cur_l]=el_match(K1,pose_cur_l,t,L,P_3d,event_d,event_n);
        [event_cluster_r event_cur_r]=el_match(K2,pose_cur_r,t,R,P_3d,event_d,event_n);

        % Pose optimization
        fprintf('\nThe optimized pose is:');

        pose_cur_l=pose_optim_stereo(event_cluster_l,event_cluster_r,pose_cur_l(1:3,1:3),pose_cur_l(1:3,4),K1,K2,P_3d,R2L)
        pose_cur_r=R2L(1:3,:)*[pose_cur_l;0 0 0 1];

        % Pose reprojection visualization
        % Figure 1 shows events of the left camera and Figure 2 showes events of the right camera
        repro_Image=reproje_show(K1,K2,pose_cur_l,pose_cur_r,event_cur_l,event_cur_r,P_3d);

    else

        fprintf('\nThe demo ends here.');

        return;

    end
end