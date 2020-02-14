% EKF-SLAM with known data association.
clear; clc; clf; close all;

% map_o3.txt    +   so_o3 ie.txt
% map_pent_big_40.txt   +   so_pb_40_no.txt
% map_sym2.txt  +   so_sym2_nk.txt
% map_sym3.txt  +   so_sym3_nk.txt

%%%%%%%%%% Load Data %%%%%%%%%%
% Map
landmarks = load('map_sym3.txt'); % Load landmark data (only used to plot map and get the number of landmarks)
N = size(landmarks,1); % Get the number of landmarks in the map

% Sensor
sensordata = 'so_sym3_nk.txt';
fid = fopen(['data/' sensordata],'r');
if fid <= 0
    fprintf('Failed to open simoutput file "%s"\n\n',sensordata);
    return
end
flines = {};
while 1
    line = fgetl(fid);
    if ~ischar(line)
        break
    end
    flines = {flines{:} line};
end
fclose(fid);

%%%%%%%%%% Parameter Initialization %%%%%%%%%%
INF = 1000;
mu = zeros(2*N+3, 1); % [x,y,theta, x1,y1, x2,y2 ....... xN,yN]'
% mu = [(Pose of robot) (Coordinates of landmark 1) ...]'
robSigma = zeros(3);
robMapSigma = zeros(3,2*N); % Or robMapSigma = INF * ones(3,2*N); 
mapSigma = INF * eye(2*N);
sigma = [[robSigma robMapSigma];[robMapSigma.' mapSigma]];
% The covariance matrix is of size (2N +3)*(2N +3).
% 3*3 zero matrix for the robot pose variables.
% All other covariance values are 0 infinite (1000).

error = [];
odom = zeros(3,1);
count = 0;
gth = [];
sigma_save = sigma(:);
t = 0;
enc = [0;0];
landmark_obs_flag = false(1,N);
% landmark_obs_flag is a vector that keeps track of which landmarks have been observed so far.

%%%%%%%%%% main loop %%%%%%%%%%
% for i = 1 : 30 % Test
while 1
    count = count + 1;
    if count > length(flines)
        break;
    end
    
    %%%%%%%%%% Get sensor data  %%%%%%%%%%
    % Data structure in lab simulation data:
    % 1   2 3 4    5 6      7 8 9     10       11 12 13       ... ... ...
    % t    odm   encoder  true pose  flag   ID bearing range
    % (Vx, Vy, Omega)   (x, y, theta)
    line = flines{count};
    values = sscanf(line, '%f');
    pt = t;
    t = values(1);
    delta_t = t - pt; % delta_t in iteration
    odom = values(2:4); % True odomentry of the car (not used)
    penc = enc;
    enc = values(5:6);
    denc = enc - penc; % encoder in robot (left and right)
    truepose = values(7:9); % true postion of the robot
    gth = [gth truepose];
    if (values(10) > 0) % flag > 0
        ids = values(11:3:end); % ID
        bearings = values(12:3:end); % Bearing
        ranges = values(13:3:end); % Ranges
        sensor = [ids'; ranges'; bearings'];
    else
        sensor = [];
    end
    
    % Perform the prediction step of the EKF
    [mu_bar, sigma_bar] = predict(mu, sigma, denc(1), denc(2), delta_t, N);
    
    % Perform the correction step of the EKF
    [mu, sigma, landmark_obs_flag] = correct(mu_bar, sigma_bar, sensor, landmark_obs_flag, landmarks, N);
    sigma_save = [sigma_save sigma(:)];
    
    % Generate visualization plots of the current state of the filter
    % Also Error analysing
    [error_R] = plot_state(mu, sigma, landmarks, count, landmark_obs_flag, sensor, truepose);
    % error_R = [x, y, theta, map, map_abs]
    error = [error error_R];
end

%%%%%%%%%% Plot error %%%%%%%%%%
mean_err_x_abs = mean(abs(error(1,:)));
mean_err_x = mean(error(1,:));
mean_err_y_abs = mean(abs(error(2,:)));
mean_err_y = mean(error(2,:));
mean_err_theta_abs = mean(abs(error(3,:)));
mean_err_theta = mean(error(3,:));
mean_err_map = mean(error(4,:));
figure(2);
clf;
subplot(4,1,1);
plot(error(1,:));
title(sprintf('Error on x, mean error=%f, mean absolute err=%f',mean_err_x,mean_err_x_abs));
subplot(4,1,2);
plot(error(2,:));
title(sprintf('Error on y, mean error=%f, mean absolute err=%f',mean_err_y,mean_err_y_abs));
subplot(4,1,3);
plot(error(3,:));
title(sprintf('Error on theta, mean error=%f, mean absolute err=%f',mean_err_map,mean_err_theta_abs));
subplot(4,1,4);
plot(error(4,:));
title(sprintf('Error on map, mean error=%f',mean_err_map));

%%%%%%%%%% Plot covariance %%%%%%%%%%
figure(3);
clf;
subplot(3,1,1);
plot(sigma_save(1,:));
title('\Sigma(1,1)');
subplot(3,1,2);
plot(sigma_save(5,:));
title('\Sigma(2,2)');
subplot(3,1,3);
plot(sigma_save(9,:));
title('\Sigma(3,3)');