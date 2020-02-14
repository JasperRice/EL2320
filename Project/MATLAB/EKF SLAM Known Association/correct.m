function [mu, sigma, observedLandmarks] = correct(mu_bar, sigma_bar, z, observedLandmarks, landmarks, N)
% Input
%   mu_bar(t)         3+2N x 1: state vector from prediction
%   sigma_bar(t)      3+2N x 3+2N: covariance matrix
%   z                 3 x i: observed i landmarks [ID Range Bearing]^T in this
%                            iteration
%   observedLandmarks 1 x N: which landmarks have been observed at some point by the robot.
%                            observedLandmarks(j) is false if the landmark
%                            with id = j has never been observed before
%   N                 1 x 1: number of all landmarks
% Output
%   mu                3+2N x 1: state vector after correction
%   sigma             3+2N x 3+2N: covariance matrix
%   observedLandmarks 1 x N: same as below

%%%%%%%%%% Paramaters initialize %%%%%%%%%%
m = size(z, 2); % Number of measurements in this time step
H = zeros(2,5); % Jacobian blocks of the measurement function
z_hat = zeros(2, 1);
z_measure = zeros(2, 1); % Vectorized form of all measurements
% [range_j bearing_j]

%%%%%%%%%% Get landmarks index %%%%%%%%%%
landmark_all_index = landmarks(:,1);

%%%%%%%%%% Construct the sensor noise matrix Q %%%%%%%%%%
% sen_noise = 0.01;
% Q = sen_noise * eye(2);

Q = [0.01^2 0; 0 (2*pi/360)^2]; % map_o3.txt + so_o3 ie.txt
% Q = [0.1^2 0; 0 0.1^2]; % map_pent_big_40.txt + so_pb_40_no.txt
% Q = [0.01 0; 0 (2*pi/360)]; % map_sym + so_sym

%%%%%%%%%% Iteration for all observed landmarks %%%%%%%%%%
for i = 1:m
    landmarkId = z(1,i); % Get the id of the landmark corresponding to the i-th observation
    if isempty(landmarkId)
        continue
    end
    landmark_index = find(landmark_all_index == landmarkId);
    
    z_measure(1) = z(2, i); % range
    z_measure(2) = z(3, i); % bearing
    
    if observedLandmarks(landmark_index) == false % If the landmark is obeserved for the first time:
        % Initialize its pose in mu based on the measurement and the current robot pose:
        mu_bar(2+landmark_index*2) = mu_bar(1) + z(2,i) * cos(mu_bar(3)+z(3,i));
        mu_bar(3+landmark_index*2) = mu_bar(2) + z(2,i) * sin(mu_bar(3)+z(3,i));
        % Indicate in the observedLandmarks vector that this landmark has been observed
        observedLandmarks(landmark_index) = true;
    end
    
    % Compute the Jacobian H and the measurement function h
    delta_x = mu_bar(2+landmark_index*2)-mu_bar(1);
    delta_y = mu_bar(3+landmark_index*2)-mu_bar(2);
    delta = [delta_x; delta_y];
    q = delta.'*delta;
    z_hat(1) = sqrt(q);
    z_hat(2) = atan2(delta_y, delta_x) - mu_bar(3);
    
    F = [eye(3) zeros(3, 2*N); zeros(2, 2*landmark_index+1) eye(2) zeros(2,2*N-2*landmark_index)];
    H = zeros(2,5);
    H(1,1) = -sqrt(q)/q*delta_x;
    H(1,2) = -sqrt(q)/q*delta_y;
    H(1,4) = sqrt(q)/q*delta_x;
    H(1,5) = sqrt(q)/q*delta_y;
    H(2,1) = delta_y/q;
    H(2,2) = -delta_x/q;
    H(2,3) = -1;
    H(2,4) = -delta_y/q;
    H(2,5) = delta_x/q;
    
    H = H * F;
    %%%%%%%%%% Compute the Kalman gain %%%%%%%%%%
    K = sigma_bar * H.' / (H * sigma_bar * H.' + Q);
    
    % Compute the difference between the expected and recorded measurements, subZ.
    % Use the normalize_all_bearings function to normalize the bearings after subtracting
    subZ = normalize_all_bearings(z_measure - z_hat);
    
    %%%%%%%%%% Computing new mu and sigma %%%%%%%%%%
    mu_bar = mu_bar + K * (subZ());
    mu_bar(3) = mod(mu_bar(3) + pi, 2*pi) - pi;
    sigma_bar = (eye(size(K*H)) - K * H) * sigma_bar;
    
end
mu = mu_bar;
sigma = sigma_bar;

end

