% function S_bar = pf_predict(S, params, dt)
% This function performs particle weighting
% Inputs:
%           S(t-1):         structure
%           params:         structure
%           dt:             1x1
% Outputs:
%           S_bar(t):       structure
function S_bar = pf_predict(S, params, dt)
N = size(S.X, 1);
M = size(S.X, 2);
switch params.motion_type
    case 0
        S_bar.X = S.X;
    case 1
        switch params.state_space_dimension
            case 2
                S_bar.X = S.X + repmat(dt * params.v_0 * [cos(params.theta_0); sin(params.theta_0)],1,M);
            case 3
                S_bar.X = S.X + dt * params.v_0 * [cos(S.X(3,:)); sin(S.X(3,:)); zeros(1,M)];
        end
    case 2
        switch params.state_space_dimension
            case 3
                S_bar.X = S.X + dt * [params.v_0*cos(S.X(3,:)); params.v_0*sin(S.X(3,:)); repmat(params.omega_0,1,M)];
        end
end
S_bar.X = S_bar.X + randn(N, params.M) .* repmat(sqrt(diag(params.Sigma_R)),1,M); % Diffusion, assuming an uncorrelated sigma_R
S_bar.W = S.W;
end