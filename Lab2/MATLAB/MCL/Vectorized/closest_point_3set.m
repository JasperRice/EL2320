% [ix1,ix2,ix3,val] = closest_point_3set(X,Y,Z)
% returns the indicies and the distance of two points, one in X and one in Y which are the
% closest points.
%       X   (dim x m)
%       Y   (dim x n)
%       ix1 (1   x 1)
%       ix2 (1   x 1)
%       val (1   x 1)
function [ix1,ix2,ix3,val] = closest_point_3set(X,Y,Z)
if nargin==0
    M = 25;
    N = 50;
    O = 100;
    dim = 3;
    X = rand(dim,M);
    Y = rand(dim,N);   
    Z = rand(dim,O);
end
clc;
profile clear; profile on;
[ix1,ix2,ix3,val, times] = compute(X,Y,Z);
profile off; profile viewer; 
%% plotting
figure(1); clf; set(gcf, 'Name','Run Times'); 
bar(times);
figure(2); clf; set(gcf, 'Name','Speedup over vectorized for loop');
bar(max(times)./times);
end

function [ix1,ix2,ix3,val, times] = compute(X,Y,Z)
%% vectorized for loop
tic;
[d m] = size(X);
[d2 n] = size(Y);
[d3 o] = size(Z);
if d~=d2 || d ~= d3, error('vectors must have the same dimension'); end
ix1_1 = -1; ix1_2 = -1; ix1_3 = -1; val1 = 1e100;
for i = 1 : m
    x = X(:,i);
    for j = 1 : n
        y = Y(:,j);
        for k = 1 : o
            z = Z(:,k);
            yx = x - y;
            yz = z - y;
	    nyz=norm(yz);
	    yzn = yz ./ nyz;
            yz_dot_yx = sum(yz .* yx); % EFFICIENT INNER PRODUCT
            vec = yx.*nyz - yzn.*yz_dot_yx;
            area = norm(vec);
            if area < val1
                ix1_1 = i; ix1_2 = j; ix1_3 = k; val1 = area;
            end
        end
    end
end
t_naive = toc
%% vectorized
tic;
% X(d,m), Y(d,n), Z(d,o) -> we want to contruct Xr(d,m,n,o) and Yr(d,m,n,o)
% and Zr(d,m,n,o)
Xr = reshape(X,[d m 1 1]);
Xr = repmat(Xr,[1 1 n o]);
size(Xr)

Yr = reshape(Y,[d 1 n 1]);
Yr = repmat(Yr,[1 m 1 o]);
size(Yr)

Zr = reshape(Z,[d 1 1 o]);
Zr = repmat(Zr,[1 m n 1]);
size(Zr)

YX = Xr - Yr;
YZ = Zr - Yr;
NYZ=repmat(sqrt(sum(YZ.^2,1)),[d 1 1 1]);
YZN = YZ ./ NYZ;

YZ_dot_YX = repmat(sum(YZ .* YX,1),[d 1 1 1]);
Vec = YX.*NYZ - YZN .* YZ_dot_YX;

Area = squeeze(sqrt(sum(Vec.^2,1)));

[val,ix] = min(Area(:));

ix3 = floor((ix-1)/m/n)+1;
ix2 = floor((ix-(ix3-1)*m*n)/m)+1;
ix1 = mod(ix-1,m)+1;

t_vec = toc
if abs(val - val1)>1e-10 || ix1~=ix1_1 || ix2 ~= ix1_2 || ix3 ~= ix1_3
    error('no match');
end
times =[t_naive t_vec];
end
