% This is an example program to simulate the car example described by the
% equations
% p(k+1) = p(k) + dt * s(k)
% s(k+1) = s(k) + dt * u(k)
% where p(k) and s(k) is the position and speed respectively
%
% This can be re-written in state-space form as
% x(k+1) = A * x(k) + B * u(k)
% where A = [1 dt; 0 1], B = [0; dt], x=[p; s]
%
% We also assume that we measure the position, which gives
% y(k) = [1 0] * x(k) = C * x(k)
%
% We also add noise, i.e.
% x(k+1) = A * x(k) + B * u(k) + G * w(k)
% y(k) = C * x(k) + D * v(k)
% where w(k) is process noise and v(k) is measurement noise.
%
% We assume that G is eye(2) (identity matrix) and that D is 1;
% We assume that w(k) and v(k) are white, zero-mean and Gaussian
%
% For estimation we use the Kalman Filter
% xhat(k+1|k) = A * xhat(k|k) + B * u(k)
% P(k+1|k) = A * P * A' + G * R * G'
%
% K(k+1) = P(k+1|k) * C' * inv(C * P(k+1|k) * C' + D * Q * D')
% xhat(k+1|k+1) = xhat(k+1|k) + K(k+1) * (y(k+1) - C * xhat(k+1|k))
% P(k+1|k+1) = P(k+1|k) - K(k+1) * C * P(k+1|k)

% Try with different values for the noise levels

%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%
% The system model
dt = 0.1;
A = [1 dt; 0 1];
B = [0; dt];
C = [1 0];

x = [1 0.5]';
u = 0;

% The simulated noise
wStdP = 0.01; % Noise on simulated position
wStdV = 0.1; % Noise on simulated velocity
vStdP = 0.1; % Simulated measurement noise on position

%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Kalman Filter modeled uncertainties and initial values
xhat = [-2 0]';
% xhat = [100 50]';
% xhat = [1.5 0.75]';
P = eye(2);
% P = 100 * P;
% P = 0.01 * P;
G = eye(2);
D = 1;
R = diag([wStdP^2 wStdV^2]);
% R = 100 * R; % R = [0.01 0; 0 1]
Q = vStdP^2;
% Q = 100 * Q; % Q = 1

n = 100;

X = zeros(2,n+1);
Xhat = zeros(2,n+1);
PP = zeros(4,n+1);
KK = zeros(2,n);

X(:,1) = x;
Xhat(:,1) = xhat;
PP(:,1) = reshape(P,4,1);
figure(1)
drawnow

for k = 1:n
    x = A * x + B * u + [wStdP*randn(1,1); wStdV*randn(1,1)]; % True
    y = C * x + D * vStdP * randn(1,1); % Measurement
    X(:,k+1) = x;
    
    % Prediction (Time update)
    xhat = A * xhat + B * u;
    P = A * P * A' + G * R * G';
    
    % Measurement update
    K = (P * C') / (C * P * C' + D * Q * D');
    xhat = xhat + K * (y - C * xhat);
    P = P - K * C * P;
    Xhat(:,k+1) = xhat;
    KK(:,k) = K;
    PP(:,k+1) = reshape(P,4,1);
    
    clf, subplot(2,1,1),
    plot(X(1,1:(k+1)),'r')
    hold on,
    plot(Xhat(1,1:(k+1)),'b')
    plot(X(1,1:(k+1))-Xhat(1,1:(k+1)),'g')
    % axis([0 n+5 -2 7])
    title('Position','Interpreter', 'latex')
    legend('True','Estimate','Error','Location','northwest')
    
    subplot(2,1,2),
    plot(X(2,1:(k+1)),'r')
    hold on,
    plot(Xhat(2,1:(k+1)),'b')
    plot(X(2,1:(k+1))-Xhat(2,1:(k+1)),'g')
    % axis([0 n+5 -2 2])
    title('Speed','Interpreter', 'latex')
    legend('True','Estimate','Error','Location','southwest')
    
    drawnow
end

E = X - Xhat;
fprintf('Standard deviation of error in position (second half): %fm\n', std(E(1,round(size(E,2)/2):end)))
fprintf('Standard deviation of error in velocity (second half): %fm/s\n', std(E(2,round(size(E,2)/2):end)))

figure(2)
title('Estimated error covariance')
subplot(2,1,1),
plot(sqrt(PP(1,:)))
title('$\sqrt{P(1,1)}$','Interpreter', 'latex')
subplot(2,1,2),
plot(sqrt(PP(4,:)))
title('$\sqrt{P(2,2)}$','Interpreter', 'latex')

figure(3)
title('Kalman filter gain coefficients')
subplot(2,1,1),
plot(KK(1,:))
title('$K(1)$','Interpreter', 'latex')
subplot(2,1,2),
plot(KK(2,:))
title('$K(2)$','Interpreter', 'latex')