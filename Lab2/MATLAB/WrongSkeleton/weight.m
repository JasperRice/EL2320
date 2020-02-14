% function S_bar = weight(S_bar,Psi,outlier)
%           S_bar(t)            4XM
%           outlier             1Xn
%           Psi(t)              1XnXM
% Outputs:
%           S_bar(t)            4XM
function S_bar = weight(S_bar,Psi,outlier)
% FILL IN HERE
% BE CAREFUL to normalize the final weights
Psi_ = reshape(Psi, size(Psi,2), size(Psi,3));
% outlier_id = find(outlier);
% Psi_(outlier_id,:) = 1; % Set to 1 so that would not affect production
w = prod(Psi_(~outlier, :), 1);
S_bar(4,:) = w / sum(w); % Normalization
end
