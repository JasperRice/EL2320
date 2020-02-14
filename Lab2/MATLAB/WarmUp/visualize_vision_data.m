% This function helps you visualize the data from visiondata.mat
% Usage: visualize_vision_data(Z, X, X_hat)
% where Z is a measurement set,
%       X is the ground truth (optional)
%       X_hat is the estimate of X (optional)
function visualize_vision_data(Z, X, X_hat)
for k=1:size(Z,2)
    plot(Z(1,k),Z(2,k),'r.','MarkerSize',20)
    if nargin>1
        hold on
        plot(X(1,k),X(2,k),'go','MarkerSize',20)
    end
    if nargin >2
        plot(X_hat(1,k),X_hat(2,k),'bx','MarkerSize',20)
    end
    hold off;
    axis([0 640 0 480])
    drawnow
end
end