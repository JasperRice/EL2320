% function S_bar = weight(S_bar,Psi,outlier)
%           S_bar(t)            4XM
%           outlier             1Xn
%           Psi(t)              1XnXM
% Outputs: 
%           S_bar(t)            4XM
function S_bar = weight(S_bar,Psi,outlier)
% FILL IN HERE
M = size(S_bar, 2);
n = size(outlier, 2);

Psi = reshape(Psi, n, M);
p = prod(Psi(~outlier, :), 1);
p = p / sum(p); %normalize
% save weight to the last row of S_bar
S_bar(4, :) = p';
end
