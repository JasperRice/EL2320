% function [S_bar] = predict(S,u,R)
% This function should perform the prediction step of MCL
% Inputs:
%           S(t-1)              4XM
%           v(t)                1X1
%           omega(t)            1X1
%           R                   3X3
%           delta_t             1X1
% Outputs:
%           S_bar(t)            4XM
function [S_bar] = predict(S,v,omega,R,delta_t)
% FILL IN HERE
M = size(S, 2);

dx = v * delta_t * cos(S(3, :));
dy = v * delta_t * sin(S(3, :));
dtheta = ones(1, M) * omega * delta_t;
dfix = zeros(1, M);
u = [dx; dy; dtheta; dfix];

noise = [R * randn(3,M); dfix];

S_bar = S + u + noise;
end