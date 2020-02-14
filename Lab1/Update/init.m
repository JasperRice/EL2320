% function [mu,sigma,R,Q,Lambda_M] = init()
% This function initializes the parameters of the filter.
% Outputs:
%			mu(0):			3X1
%			sigma(0):		3X3
%			R:				3X3
%			Q:				2X2
function [mu,sigma,R,Q,Lambda_M] = init()
mu = [0;0;0]; % initial estimate of state
sigma = 1e-10*diag([1 1 1]); % initial covariance matrix

% Fill In This Part
case_id = 2;
switch case_id
    case 1 % map_o3.txt + so_o3 ie.txt
        R = [0.01^2 0 0; 0 0.01^2 0; 0 0 (2 * pi / 360)^2];
        Q = [0.01^2 0; 0 (2*pi / 360)^2];
        delta_m = 0.999;
    case 2 % map_pent_big_10.txt + so_pb_10_outlier.txt
        R = [0.01^2 0 0; 0 0.01^2 0; 0 0 (2 * pi / 360)^2];
        Q = [0.2^2 0; 0 0.2^2];
        delta_m = 0.999;
    case 3 % map_pent_big_40.txt + so_pb_40_no.txt
        R = [1 0 0; 0 1 0; 0 0 1];
        Q = [0.1^2 0; 0 0.1^2];
        delta_m = 1;
end
Lambda_M = chi2inv(delta_m,2);
end