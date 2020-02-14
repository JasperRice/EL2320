% function S = multinomial_resample(S_bar)
% This function performs systematic re-sampling
% Inputs:   
%           S_bar(t):       4XM
% Outputs:
%           S(t):           4XM
function S = multinomial_resample(S_bar)
% FILL IN HERE
cdf = cumsum(S_bar(4, :));
M = size(S_bar,2);
S = zeros(size(S_bar));
for m = 1 : M
    r_m = rand; % random number within range [0, 1]
    i = find(cdf >= r_m, 1, 'first');
    S(1:3, m) = S_bar(1:3, i);
end
S(4, :) = 1/M *ones(size(1, M));
end
