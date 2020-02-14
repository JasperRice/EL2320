function [mu, sigma, N, Id] = correct(mu_bar, sigma_bar, z, N)
inf = 1000;
m = size(z, 2);
H = zeros(2,5);
z_hat = zeros(2, 1);
z_measure = zeros(2, 1);
alpha = 3;
Id = [];
sen_noise = 0.01;
Q = sen_noise * eye(2);

for i = 1:m
    landmarkId = z(1,i);
    if isempty(landmarkId)
        continue
    end

    z_measure(1) = z(2, i);
    z_measure(2) = z(3, i);

    mu_bar = [mu_bar;zeros(2,1)];
    mu_bar(2+(N+1)*2) = mu_bar(1) + z(2,i) * cos(mu_bar(3)+z(3,i));
    mu_bar(3+(N+1)*2) = mu_bar(2) + z(2,i) * sin(mu_bar(3)+z(3,i));

    sigma_bar = [sigma_bar zeros(2*N+3, 2);zeros(2, 2*N+3) inf*eye(2)];

    H_all = [];
    Phi_all = [];
    Pi_all = [];
    z_hat_all = [];

    for k = 1:N+1
        Nt = N+1;
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
        subZ = normalize_all_bearings(z_measure - z_hat);
        Pi = (subZ.'/ Phi)* subZ;
        H_all(:,:,k) = H;
        Phi_all(:,:,k) = Phi;
        Pi_all(k) = Pi;
        z_hat_all(:,k) = z_hat;
    end
    Pi_all(Nt) = alpha;
    j = find(Pi_all == min(Pi_all));
    Id(i) = j;
    N_save = N;
    N = max(N,j);

    if N_save == N
        mu_bar = mu_bar(1:3+2*N);
        sigma_bar = sigma_bar(1:3+2*N, 1:3+2*N);
    end

    H_K = H_all(:,:,j);
    H_K = H_K(:,1:3+2*N);
    K = sigma_bar * H_K.' / Phi_all(:,:,j);

    subZ = normalize_all_bearings(z_measure - z_hat_all(:,j));
    mu_bar = mu_bar + K * (subZ);
    mu_bar(3) = mod(mu_bar(3) + pi, 2*pi) - pi;
    sigma_bar = (eye(size(K*H_K)) - K * H_K) * sigma_bar;
end
mu = mu_bar;
sigma = sigma_bar;
end
