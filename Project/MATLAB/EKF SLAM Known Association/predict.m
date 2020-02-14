function [mu_bar, sigma_bar] = predict(mu, sigma, e_R, e_L, delta_t, N)
% Input:
%    mu(t-1)         (2N+3) x 1: State vector (see Parameter Initialization)
%    sigma(t-1)      (2N+3) x (2N+3): Covariance matrix
%    e_L(t)          1 x 1: Left encoder
%    e_R(t)          1 x 1: Right encoder
%    delta_t         1 x 1: Delta time
%    N               1 x 1: Nuber of landmarks

% Output:
%    mu_bar(t)       (2N+3) x 1: Predicted state vector in next iteration
%    sigma_bar(t)    (2N+3) x 3+2N(2N+3): Covariance matrix

%%%%%%%%%% Calculate odometry %%%%%%%%%%
E_T = 2048;         % Number of encoder ticks per wheel revolution
B = 0.35;           % Wheel base
R_R = 0.1;          % Radius of the right wheels
R_L = 0.1;          % Radius of the left wheels

if ~delta_t
    u = [0;0;0];
else
w_r = (2*pi*e_R)/(E_T*delta_t);
w_l = (2*pi*e_L)/(E_T*delta_t);
w_t = (w_r*R_R - w_l*R_L)/B;
v_t = (w_r*R_R + w_l*R_L)/2;
mu(3) = pi_2_pi(mu(3));
u = [v_t * delta_t * cos(mu(3));
     v_t * delta_t * sin(mu(3));
     w_t * delta_t];
end

% Motion noise
R = [0.01^2 0 0; 0 0.01^2 0; 0 0 (2 * pi / 360)^2]; % map_o3.txt + so_o3 ie.txt
% R = [1 0 0; 0 1 0; 0 0 1]; % map_pent_big_40.txt + so_pb_40_no.txt
% R = [0.01 0 0; 0 0.01 0; 0 0 2*pi/360]; % map_sym + so_sym

%%%%%%%%%% Prediction process %%%%%%%%%%
F = [eye(3) zeros(3,2*N)];
mu_bar = mu + F' * u;
G = eye(2*N+3) + F' * [ 0 0 -u(2,1); 0 0 u(1,1); 0 0 0] * F;
sigma_bar = G * sigma * G' + F' * R * F;
end

