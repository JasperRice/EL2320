% EKF SLAM with unknown data association
% Run this script to do the SLAM

clear; clc;
close all;

%%%%%%% Load Data %%%%%%%
landmarks = load('map_sym3.txt'); % Load landmark data (only used to plot map and get the number of landmarks)

sensordata = 'so_sym3_nk.txt'; % Load sensor data
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

%%%%%%%% Parameter Initialization %%%%%%%%%%%
N = 0; % The initial size of map is set as 0
mu = zeros(3, 1); % [x,y,theta, x1,y1, x2,y2 ....... xN,yN].'
% (pose of robot) (coordinates of landmark)
sigma = zeros(3);
% The covariance matrix is of size (2N +3)*(2N +3).
% It is composed of a small 3*3 matrix of zeros for the robot pose variables.

enc = [0;0];
error = [];
sigma_save = sigma(:);
count = 0;
t = 0;

%%%%%%% main loop %%%%%%%%
while 1
    count = count + 1;
    if count > length(flines)
        break;
    end
    
    %%%%%%%%%%%% Get sensor data %%%%%%%%%%%%%%%
    % @1@
    % data structure in lab simulation data
    % 1   2 3 4    5 6      7 8 9     10       11 12 13       ... ... ...
    % t    odm   encoder  true pose  flag   ID bearing range
    % (Vx, Vy, Omega)   (x, y, theta)
    line = flines{count};
    values = sscanf(line, '%f');
    pt = t;
    t = values(1);
    delta_t = t - pt; % delta_t in iteration
    odom = values(2:4); %true odomentry of the car (seems not useful)
    penc = enc;
    enc = values(5:6);
    denc = enc - penc; %encoder in rob(left and right)
    truepose = values(7:9);%true postion of the robot
    if (values(10) > 0) % flag > 0
        ids = values(11:3:end);%ID
        ranges = values(13:3:end);% Ranges
        bearings = values(12:3:end);% Bearing
        sensor = [ids'; ranges'; bearings']; % the association is loaded but not used
        
    else
        sensor = [];
    end
    
    % Perform the prediction step of the EKF
    [mu_bar, sigma_bar] = predict(mu, sigma, denc(1), denc(2), delta_t, N);
    
    % Perform the correction step of the EKF
    [mu, sigma, N, Id] = correct(mu_bar, sigma_bar, sensor, N);
    sigma_pose = sigma(1:3,1:3); % only save sigma of pose for covariance analysis.
    sigma_save = [sigma_save sigma_pose(:)];
    
    % Generate visualization plots of the current state of the filter
    % Also Error analysing
    [error_R] = plot_state(mu, sigma, landmarks, count, Id, truepose, N);
    error = [error error_R]; % error_R [x, y, theta, map, map_abs ]
end

% plot error
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
title(sprintf('error on x, mean error=%f, mean absolute err=%f',mean_err_x,mean_err_x_abs));
subplot(4,1,2);
plot(error(2,:));
title(sprintf('error on y, mean error=%f, mean absolute err=%f',mean_err_y,mean_err_y_abs));
subplot(4,1,3);
plot(error(3,:));
title(sprintf('error on theta, mean error=%f, mean absolute err=%f',mean_err_map,mean_err_theta_abs));
subplot(4,1,4);
plot(error(4,:));
title(sprintf('error on map, mean error=%f',mean_err_map));

% plot covariance
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








