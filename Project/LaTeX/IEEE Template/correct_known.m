function [mu, sigma, observedLandmarks] = correct(mu_bar, sigma_bar, z, observedLandmarks, landmarks, N)
m = size(z, 2);
H = zeros(2,5);
z_hat = zeros(2, 1);
z_measure = zeros(2, 1);

landmark_all_index = landmarks(:,1);

sen_noise = 0.01;
Q = sen_noise * eye(2);

for i = 1:m
    landmarkId = z(1,i);
    if isempty(landmarkId)
        continue
    end
    landmark_index = find(landmark_all_index == landmarkId);

    z_measure(1) = z(2, i);
    z_measure(2) = z(3, i);

    if observedLandmarks(landmark_index) == false
        mu_bar(2+landmark_index*2) = mu_bar(1) + z(2,i) * cos(mu_bar(3)+z(3,i));
        mu_bar(3+landmark_index*2) = mu_bar(2) + z(2,i) * sin(mu_bar(3)+z(3,i));
        observedLandmarks(landmark_index) = true;
    end

    delta_x = mu_bar(2+landmark_index*2)-mu_bar(1);
    delta_y = mu_bar(3+landmark_index*2)-mu_bar(2);
    delta = [delta_x; delta_y];
    q = delta.'*delta;
    z_hat(1) = sqrt(q);
    z_hat(2) = atan2(delta_y, delta_x) - mu_bar(3);

    F = [eye(3) zeros(3, 2*N); zeros(2, 2*landmark_index+1) eye(2) zeros(2,2*N-2*landmark_index)];
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
    K = sigma_bar * H.' / (H * sigma_bar * H.' + Q);

    subZ = normalize_all_bearings(z_measure - z_hat);

    mu_bar = mu_bar + K * (subZ());
    mu_bar(3) = mod(mu_bar(3) + pi, 2*pi) - pi;
    sigma_bar = (eye(size(K*H)) - K * H) * sigma_bar;
end
mu = mu_bar;
sigma = sigma_bar;
end
