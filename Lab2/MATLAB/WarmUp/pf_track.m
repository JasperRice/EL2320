% function pf_track(Z, X, VERBOSE, Dimension, Motion, Sigma_Q, Sigma_R)
% Performs the particle filter tracking
% Inputs:   Z:          2xK
%           X:          2xK
%           VERBOSE:    1x1
%           Dimension:  1x1
%           Motion:     1x1
%           Sigma_Q:    2x2
%           Sigma_R:    2x2 or 3x3
% VERBOSE   0: no visual output
%           1: information about end resutls
%           2: shows particles
% Motion    0: fixed
%           1: linear
%           2: circular
% Motion determine the motion type the prediction would take.

function pf_track(Z, X, VERBOSE, Dimension, Motion, Sigma_Q, Sigma_R)
%% Parameter Initialization
if nargin < 3
    VERBOSE = 2;
    Dimension = 2;
    Motion = 0;
    Sigma_Q = diag([100 100]);
    Sigma_R = diag([2 2]);
end
params.state_space_dimension = Dimension;   % Either 3 or 2
params.Sigma_Q = Sigma_Q;   % Measurement noise covariance matrix
                            % [100 100] by default:
params.M = 1000;	% Number of particles
params.motion_type = Motion;
params.v_0 = 2 * pi * 200 / 688;
params.theta_0 = 0;
params.state_space_bound = [640; 480]; % [640; 480] by default
params.thresh_avg_likelihood = 0.0001;
RESAMPLE_MODE = 1;  % 0 = no resampling
                    % 1 = vanilla resampling
                    % 2 = systematic resampling
switch params.state_space_dimension
    case 2
        % params.Sigma_R = diag([0 0]); % Question 6
        params.Sigma_R = diag([2 2]);   % process noise covariance matrix
                                        % [2 2] by default
        % if params.omega_0
        % error('2D state space can not use omega_0');
        % end
    case 3
        params.omega_0 = 2 * pi / 688;
        params.Sigma_R = Sigma_R;   % process noise covariance matrix
                                    % [2 2 0.01] by default
end
dt = 1;
%% Variable Initialization
close all;
S.X = [rand(1, params.M)*params.state_space_bound(1);
    rand(1, params.M)*params.state_space_bound(2)];
% sampling uniformly from the state space
if params.state_space_dimension > 2
    S.X = [S.X; rand(1, params.M) * 2 * pi - pi];
end
S.W = 1/params.M * ones(1,params.M); % initialize equal weights
K = size(Z,2); % number of observations
mean_S = zeros(2,K);
figure(1);
clf;
%% Particle Filter Algorithm
for i = 1 : K
    S_bar = pf_predict(S, params, dt);
    z = Z(:,i); % ith observation: (x, y)
    S_bar = pf_weight(S_bar, z, params, 0); % 0 = not print outlier warning
                                            % 1 = print outlier warning
    switch RESAMPLE_MODE
        case 0
            S = S_bar;
        case 1
            S = multinomial_resample(S_bar);
        case 2
            S = systematic_resample(S_bar);
    end
    mean_S(:,i) = mean(S.X(1:2,:),2); % compute the center of the distribution
    if VERBOSE > 1
        plot(Z(1,i),Z(2,i),'rx','MarkerSize',10);
        hold on;
        plot(X(1,i),X(2,i),'go','MarkerSize',20);
        plot(S.X(1,:),S.X(2,:),'b.');
        hold off;
        axis([0 params.state_space_bound(1) 0 params.state_space_bound(2)]);
        title(sprintf('Timestep %d of %d',i,K));
        drawnow;
    end
end
%% Analyzing the estimates
err_z = sqrt(sum((Z - X).^2, 1));
err_xhat = sqrt(sum((X - mean_S).^2,1));
merr_z = mean(err_z);
merr_xhat = mean(err_xhat);
stderr_z = std(err_z);
stderr_xhat = std(err_xhat);
format compact;
fprintf('Absolute error analysis: measurements: %0.1f +- %0.1f, estimates: %0.1f +- %0.1f\n',merr_z,stderr_z, merr_xhat,stderr_xhat);
if VERBOSE
    figure(3);
    clf;
    plot(err_z,'r-');
    hold on;
    plot(err_xhat,'b-');
    title(sprintf('Absolute error analysis: measurements: %0.1f \\pm %0.1f, estimates: %0.1f \\pm %0.1f',merr_z,stderr_z, merr_xhat,stderr_xhat));
    if VERBOSE > 1
        figure(2);
        visualize_vision_data(Z,X,mean_S);
    end
end
end