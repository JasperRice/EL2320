function [mu_bar, sigma_bar] = predict(mu, sigma, e_R, e_L, delta_t, N)
%Updates the belief concerning the robot pose according to the motion model,
%Input
%    mu(t-1)         3+2N x 1: state vector(see Parameter Initialization in main function)
%    sigma(t-1)      3+2N x 3+2N: covariance matrix
%    e_L(t)          1 x 1: left encoder
%    e_R(t)          1 x 1: right encoder
%    delta_t         1 x 1
%    N               1 x 1: nuber of observed landmarks(t-1)

%Output
%    mu_bar(t)       3+2N x 1: state vector in next iteration with prediction
%    sigma_bar(t)    3+2N x 3+2N: covariance matrix

%%%%%%%%% calculate odometry %%%%%%%%%%%%%

E_T = 2048;       %number of encoder ticks per wheel revolution
B = 0.35;         %wheel base(the distance between the contact points of the wheels)
R_R = 0.1;
R_L = 0.1;        %radius of the right and the left wheels
if ~delta_t
    u = [0;0;0];
else
    w_r = (2*pi*e_R)/(E_T*delta_t);
    w_l = (2*pi*e_L)/(E_T*delta_t);
    w_t = (w_r*R_R - w_l*R_L)/B;
    v_t = (w_r*R_R + w_l*R_L)/2;
    u = [ v_t*delta_t*cos(mu(3));v_t*delta_t*sin(mu(3));w_t*delta_t];
end

% Motion noise
% R = [0.01^2 0 0; 0 0.01^2 0; 0 0 (2 * pi / 360)^2]; % map_o3.txt + so_o3 ie.txt
% R = [1 0 0; 0 1 0; 0 0 1]; % map_pent_big_40.txt + so_pb_40_no.txt
R = [0.01 0 0; 0 0.01 0; 0 0 2*pi/360]; % map_sym + so_sym

%%%%%%%%%%%% prediction process %%%%%%%%%%%
F = [eye(3) zeros(3,2*N)];
mu_bar = mu + F.' * u;
G = eye(3+2*N)+  F.' *[ 0 0 -u(2,1); 0 0 u(1,1); 0 0 0] * F;
sigma_bar = G*sigma*G' + F.' * R * F;
end