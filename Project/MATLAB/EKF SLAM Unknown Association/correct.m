function [mu, sigma, N, Id] = correct(mu_bar, sigma_bar, z, N)
% Updates the belief: mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
%Input
%   mu_bar(t)         3+2N x 1: state vector from prediction
%   sigma_bar(t)      3+2N x 3+2N: covariance matrix
%   z                 3 x i: observed i landmarks [ID Range Bearing]^T in this iteration
%                            the ID is not used
%   N                 1 x 1: number of observed landmarks(t-1)
%Output
%   mu                3+2N x 1: state vector after correction
%   sigma             3+2N x 3+2N: covariance matrix
%   N                 1 x 1: number of observed landmarks(t)
%   Id                1 x i: Id of observed landmark

%%%%%%%%%% Paramaters initialize %%%%%%%%%%%
m = size(z, 2); % number of measurements in this time step
H = zeros(2,5); % Jacobian blocks of the measurement function
z_hat = zeros(2, 1);
z_measure = zeros(2, 1); % vectorized form of all measurements
% [range_j bearing_j ]
alpha = 6; % threshold of landmark association
Id = [];

%%%%%%%%%%% Construct the sensor noise matrix Q %%%%%%%%
%Q = [0.01^2 0; 0 (2*pi/360)^2]; % map_o3.txt + so_o3 ie.txt
 Q = [0.1^2 0; 0 0.1^2]; % map_pent_big_40.txt + so_pb_40_no.txt
% Q = [0.01 0; 0 (2*pi/360)]; % map_sym + so_sym

%%%%%%%%%%% iteration for all observed landmarks %%%%%%%%%%%
for i = 1:m
    landmarkId = z(1,i); % Get the id of the landmark corresponding to the i-th observation
    if isempty(landmarkId)
        continue
    end
    
    z_measure(1) = z(2, i); % range
    z_measure(2) = z(3, i); % bearing
    
    mu_bar = [mu_bar;zeros(2,1)]; % open area for new landmarks
    mu_bar(2+(N+1)*2) = mu_bar(1) + z(2,i) * cos(mu_bar(3)+z(3,i)); % add new hypothetical landmark
    mu_bar(3+(N+1)*2) = mu_bar(2) + z(2,i) * sin(mu_bar(3)+z(3,i));
    inf = 1000;
    sigma_bar = [sigma_bar zeros(2*N+3, 2);zeros(2, 2*N+3) inf*eye(2)]; % add infinity covarince of the new landmarks to the sigma
    
    % mu_bar'=[mu_bar 0 0]^T
    % sigma_bar'={[sigma_bar]0   0}
    %                       ... ...
    %             0 ... ... inf  0
    %             0 ... ... 0    inf
    
    H_all = []; % initialize the register
    Phi_all = [];
    Pi_all = [];
    z_hat_all = [];
    
    for k = 1:N+1
        
        Nt = N+1; % temporary number of landmarks
        % Compute the Jacobian H and the measurement function h
        delta_x = mu_bar(2+k*2)-mu_bar(1);
        delta_y = mu_bar(3+k*2)-mu_bar(2);
        delta = [delta_x; delta_y];
        q = delta.'*delta;
        
        z_hat(1) = sqrt(q);
        z_hat(2) = atan2(delta_y, delta_x) - mu_bar(3);
        
        F = [eye(3) zeros(3, 2*Nt); zeros(2, 2*k+1) eye(2) zeros(2,2*Nt-2*k)];
        
        H = zeros(2,5);
        H(1,1) = -sqrt(q)/q*delta_x;
        H(1,2) = -sqrt(q)/q*delta_y;
        H(1,4) = sqrt(q)/q*delta_x;
        H(1,5) = sqrt(q)/q*delta_y;
        H(2,1) = delta_y/q;
        H(2,2) = -delta_x/q;
        H(2,3) = -1;
        H(2,4) = -delta_y/q;
        H(2,5) = delta_x/q;
        H = H * F;
        
        Phi = H * sigma_bar * H.'+ Q;
        % Compute the difference between the expected and recorded measurements, subZ.
        subZ = normalize_all_bearings(z_measure - z_hat);
        Pi = (subZ.'/ Phi)* subZ;
        
        % save H, Phi, Pi z_hat in H_all Phi_all, Pi_all, z_hat_all
        H_all(:,:,k) = H;
        Phi_all(:,:,k) = Phi;
        Pi_all(k) = Pi;
        z_hat_all(:,k) = z_hat;
    end
    Pi_all(Nt) = alpha; % the Pi of new landmarks is set as alpha
    % Get the index for the min value of Pi.
    j = find(Pi_all == min(Pi_all));
    
    Id(i) = j; % save ID of observed landmarks
    N_save = N;
    N = max(N,j);
    
    if N_save == N % if N is not changed (no landmarks is added)
        %            delete the temporary landmarks and its sigma in line 37
        mu_bar = mu_bar(1:3+2*N);
        sigma_bar = sigma_bar(1:3+2*N, 1:3+2*N);
    end
    
    H_K = H_all(:,:,j);
    H_K = H_K(:,1:3+2*N); % choose the suitable size of H from H_all
    %%%%%%%%%%%% Compute the Kalman gain %%%%%%%%%%
    K = sigma_bar * H_K.' / Phi_all(:,:,j);
    
    %%%%%%%%%%%% Computing new mu and sigma %%%%%%%%%
    subZ = normalize_all_bearings(z_measure - z_hat_all(:,j));
    mu_bar = mu_bar + K * (subZ);
    mu_bar(3) = mod(mu_bar(3) + pi, 2*pi) - pi;
    sigma_bar = (eye(size(K*H_K)) - K * H_K) * sigma_bar;
end
mu = mu_bar;
sigma = sigma_bar;

end