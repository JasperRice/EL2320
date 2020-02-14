% function S = multinomial_resample(S_bar)
% This function performs systematic re-sampling
% Inputs:
%           S_bar(t):       4XM
% Outputs:
%           S(t):           4XM
function S = multinomial_resample(S_bar)
% FILL IN HERE
cdf = cumsum(S_bar(4,:));
M = size(S_bar, 2);
S = zeros(4,M);
for m = 1 : M
    r_m = rand;
    i = find(cdf >= r_m, 1, 'first');
    if ~isempty(i)
        S(:,m) = S_bar(:,i);
    end
    S(4,m) = 1 / M;
end
end