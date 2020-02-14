function [mu_bar, sigma_bar] = predict(mu, sigma, e_R, e_L, delta_t, N)
E_T = 2048;
B = 0.35;
R_R = 0.1;
R_L = 0.1;

if ~delta_t
    u = [0;0;0];
else
w_r = (2*pi*e_R)/(E_T*delta_t);
w_l = (2*pi*e_L)/(E_T*delta_t);
w_t = (w_r*R_R - w_l*R_L)/B;
v_t = (w_r*R_R + w_l*R_L)/2;
mu(3) = pi_2_pi(mu(3));
u = [v_t * delta_t * cos(mu(3));
     v_t * delta_t * sin(mu(3));
     w_t * delta_t];
end

R = [0.01, 0, 0;
     0, 0.01, 0;
     0, 0, 0.01];

F = [eye(3) zeros(3,2*N)];
mu_bar = mu + F' * u;
G = eye(2*N+3) + F' * [ 0 0 -u(2,1); 0 0 u(1,1); 0 0 0] * F;
sigma_bar = G * sigma * G' + F' * R * F;
end

