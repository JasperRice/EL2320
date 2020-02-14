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
u = [delta_t*v*cos(S(3,:));
     delta_t*v*sin(S(3,:));
     delta_t*omega*ones(1,M);
     zeros(1,M)];
Diffusion = [normrnd(0,R(1,1),[1,M]);
             normrnd(0,R(2,2),[1,M]);
             normrnd(0,R(3,3),[1,M]);
             zeros(1,M)];
S_bar = S + u + Diffusion;
end