% function S = pf_weight(S, z, params)
% This function performs particle weighting
% Inputs:
%           S_bar(t):       structure
%           z(t):           2x1
%           params:         structure
% Outputs:
%           S_bar(t):       structure
function S_bar = pf_weight(S_bar, z, params, verbose)
S_bar.W = sensor_model(z,params.Sigma_Q,S_bar.X(1:2,:),params.thresh_avg_likelihood, verbose);
end

function p = sensor_model(z, Sigma_Q, X,thresh_avg_likelihood, verbose)
VECTOR_MODE = 1;
if ~ VECTOR_MODE
    M = size(X,2);
    p = zeros(1,M);
    for m = 1 : M
        % p(m) = exp(-0.5* (z - X(:,m))' * inv(Sigma_Q) * (z - X(:,m)));
        p(m) = (exp(-0.5* (z - X(:,m))' / Sigma_Q) * (z - X(:,m)));
    end
else
    p = exp(-0.5 * ((z(1) - X(1,:)).^2/Sigma_Q(1) + (z(2) - X(2,:)).^2 / Sigma_Q(4)));
end
if mean(p) < thresh_avg_likelihood % probably an outlier, or bad parameter settings
    if verbose
        fprintf('Warning: Outlier detected. Ignoring.\n');
    end
    p(:) = 1;
end
p = p / sum(p);
end