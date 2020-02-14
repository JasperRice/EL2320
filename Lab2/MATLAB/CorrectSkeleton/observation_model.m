% function h = observation_model(S,W,j)
% This function is the implementation of the observation model
% The bearing should lie in the interval [-pi,pi)
% Inputs:
%           S           4XM
%           W           2XN
%           j           1X1
% Outputs:  
%           h           2XM
function h = observation_model(S,W,j)
% FILL IN HERE
M = size(S, 2);
h = ones(2, M);

h(1, :) = sqrt((repmat(W(1, j),1, M) - S(1, :)).^2 + (repmat(W(2, j),1, M) - S(2, :)).^2);
h(2, :) = atan2(repmat(W(2, j),1, M) - S(2, :), repmat(W(1, j),1, M) - S(1, :)) - S(3, :);
h(2, :) = mod(h(2, :) + pi, 2 * pi) - pi;
end