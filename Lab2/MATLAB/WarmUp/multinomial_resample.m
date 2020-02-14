% function S = multinomial_resample(S_bar)
% This function performs multinomial re-sampling
% Inputs:
%           S_bar(t):       structure
% Outputs:
%           S(t):           structure
function S = multinomial_resample(S_bar)
cdf = cumsum(S_bar.W);
M = size(S_bar.X, 2);
S.X = zeros(size(S_bar.X));
for m = 1 : M
    r_m = rand;
    i = find(cdf >= r_m,1,'first');
    S.X(:,m) = S_bar.X(:,i);
end
S.W = 1 / M * ones(size(S_bar.W));
end
