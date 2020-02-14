% function S = systematic_resample(S_bar)
% This function performs systematic re-sampling
% Inputs:   
%           S_bar(t):       structure
% Outputs:
%           S(t):           structure
function S = systematic_resample(S_bar)
cdf = cumsum(S_bar.W);
M = size(S_bar.X, 2);
S.X = zeros(size(S_bar.X));
r_0 = rand / M;
for m = 1 : M
    i = find(cdf >= r_0,1,'first');
    S.X(:,m) = S_bar.X(:,i);
    r_0 = r_0 + 1/M;
end
S.W = 1 / M * ones(size(S_bar.W));
end