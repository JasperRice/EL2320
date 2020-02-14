% [ix1,ix2,val] = closest_point_2set(X,Y)
% returns the indicies and the distance of two points, one in X and one in Y which are the
% closest points.
%       X   (dim x m)
%       Y   (dim x n)
%       ix1 (1   x 1)
%       ix2 (1   x 1)
%       val (1   x 1)
function [ix1,ix2,val] = closest_point_2set(X,Y)
if nargin==0
    M = 1000;
    N = 1000;
    dim = 3;
    X = rand(dim,M);
    Y = rand(dim,N);   
end
clc;
profile clear; profile on;
[ix1,ix2,val, times] = compute(X,Y);
profile off; profile viewer; 
%% plotting
figure(1); clf; set(gcf, 'Name','Run Times'); 
bar(times);
figure(2); clf; set(gcf, 'Name','Speedup over naive for loop');
bar(max(times)./times);
end

function [ix1,ix2,val, times] = compute(X,Y)
%% naive for loop
tic;
[d m] = size(X);
[d2 n] = size(Y);
if d~=d2, error('vectors must have the same dimension'); end
% D1 = zeros(m,n);
ix1_1 = -1; ix1_2 = -1; val1 = 1e100;
for i = 1 : m
    for j = 1 : n
        td = 0;
        for k = 1 : d
            td = td + (X(k,i) - Y(k,j))^2;
        end
%         D1(i,j) = td;
        if td < val1
            ix1_1 = i; ix1_2 = j; val1 = td;
        end
    end
end
t_naive = toc
%% vectorized
tic;
% X(d,m) and Y(d,n) -> we want to contruct Xr(d,m,n) and Yr(d,m,n)
Xr = reshape(X,[d m 1]);
Xr = repmat(Xr,[1 1 n]);
size(Xr)

Yr = reshape(Y,[d 1 n]);
Yr = repmat(Yr,[1 m 1]);
size(Yr)

% D2 = sum((Xr - Yr).^2,1); % WHAT WILL BE THE DIMENSION OF D2 here?
D2 = squeeze(sum((Xr - Yr).^2,1)); 

[val,ix] = min(D2(:));
ix1 = mod(ix-1,m)+1;
ix2 = floor((ix-1)/m)+1;
t_vec = toc
if abs(val - val1)>1e-10 || ix1~=ix1_1 || ix2 ~= ix1_2
    error('no match');
end
times =[t_naive t_vec];
end