function [error] = plot_state(mu, sigma, landmarks, timestep, Id, truepose, N)
% Visualizes the state of the EKF SLAM algorithm and feedback the error of the result
% The resulting plot displays the following information:
% - map ground truth (black +'s)
% - current robot pose estimate (red)
% - current landmark pose estimates (blue)
% - visualization of the observations made at this time step (line between robot and landmark)
%
%Input
%   mu                   3+2N x 1: state vector [x, y, theta(pose), x1, y1, x2, y2, ...]
%   sigma_bar(t)         3+2N x 3+2N: covariance matrix
%   landmarks            N x 3: landmarks from map
%   timestep             1 x 1: time of iteration
%   Id                   1 x m: Id of landmark observed by sensor
%   truepose             1 x 3: pose of real position
%   N                    1 x 1: number of observed landmarks(t)
%Output
%   error                3+2N x 1: error of state vector

clf;
hold on
grid('on')

xmin = min(landmarks(:,2)) - 5;
xmax = max(landmarks(:,2)) + 5;
ymin = min(landmarks(:,3)) - 5;
ymax = max(landmarks(:,3)) + 5;
plot(landmarks(:,2), landmarks(:,3), 'kx', 'markersize', 5, 'linewidth', 1);% draw map landmarks
hold on;
axis([xmin xmax ymin ymax]);

drawprobellipse(mu(1:3), sigma(1:3,1:3), 0.6, 'r');% draw elliptic probability region of pose
for i=1:N % draw map of landmarks from EKF slam
    plot(mu(2*i+ 2),mu(2*i+ 3), 'bo', 'markersize', 5, 'linewidth', 1)
    drawprobellipse(mu(2*i+ 2:2*i+ 3), sigma(2*i+ 2:2*i+ 3,2*i+ 2:2*i+ 3), 0.6, 'b'); % draw elliptic probability
end

for i=1:size(Id,2)
    mX = mu(2*Id(i)+2);
    mY = mu(2*Id(i)+3);
    line([mu(1), mX],[mu(2), mY], 'color', 'k', 'linewidth', 1);% draw observations
end

drawrobot(mu(1:3), 'r', 3); % draw robot

drawrobot(truepose, 'g', 3); % draw robot
hold off
figure(1);
drawnow;

% Make GIF
frame = getframe(1);
im = frame2im(frame);
[imind, cm] = rgb2ind(im,256);
if timestep == 1
    imwrite(imind,cm,'EKF implementation.gif','gif', 'Loopcount',Inf,'DelayTime',0);
else
    imwrite(imind,cm,'EKF implementation.gif','gif', 'WriteMode','append','DelayTime',0);
end

% Calculate Error
error(1:3,1) = mu(1:3,1) - truepose(1:3,1);% pose(x,y)
error(3,1) = mod(error(3,1) + pi, 2*pi) - pi;% pose(theta)

j = 0;
error_map = 0;
for i=1:N
    j=j+1;
    error_map = error_map + norm([mu(2*i+2) mu(2*i+3)] - [landmarks(i, 2) landmarks(i, 3)]);
end
error(4,1) = error_map / j; % error of map

end
