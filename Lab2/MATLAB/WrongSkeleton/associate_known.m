% function [outlier,Psi] = associate_known(S_bar,z,W,Lambda_psi,Q,known_associations)
%           S_bar(t)            4XM
%           z(t)                2Xn
%           W                   2XN
%           Lambda_psi          1X1
%           Q                   2X2
%           known_associations  1Xn
% Outputs:
%           outlier             1Xn
%           Psi(t)              1XnXM
function [outlier,Psi] = associate_known(S_bar, z, W, Lambda_psi, Q, known_associations)
% FILL IN HERE
% BE SURE THAT YOUR innovation 'nu' has its second component in [-pi, pi]
% Also notice that you have to do something here even if you do not have to maximize the likelihood.
n = size(z,2);      % n: The number of measurements
M = size(S_bar,2);  % M: The number of particles
N = size(W,2);      % N: The number of landmarks

% 'nu' cannot be transposed
diag_inverse_Q = diag(inv(Q));
Q_new = repmat(diag_inverse_Q, [1 N M]);

% {Predict Measurement}
z_bar = zeros(2, M, N);
for k = 1 : N
    z_bar(:,:,k) = observation_model(S_bar, W, k);
end
% Rearrange to 2xNxM
z_bar = permute(z_bar, [1,3,2]);

% {Calculate Innovation} and {Calculate Likelihood}
% Keep it outside the for loop to save time
norm_param = (2*pi*sqrt(det(Q)))^(-1);
Psi = zeros(1, n, M);
for i = 1 : n
    nu(1,:,:) = z(1,i) - z_bar(1,:,:);
    nu(2,:,:) = z(2,i) - z_bar(2,:,:);
    nu(2,:,:) = mod(nu(2,:,:)+pi, 2*pi) - pi; % [-pi, pi)
    Psi(1,i,:) = max(norm_param * exp(sum(-0.5 .* nu .* Q_new .* nu, 1)));
end

outlier = mean(reshape(Psi, n, M), 2) <= Lambda_psi; % Logical array
end
