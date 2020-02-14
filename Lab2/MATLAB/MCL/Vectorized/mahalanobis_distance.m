% function D = mahalanobis_distance(x,Y,Sigma)
% computes the mahalanobis distance of x to all points in Y given the
% covariance matrix Sigma
%       x   (dim x 1)
%       Y   (dim x n)
%       D   (1   x n)
function D = mahalanobis_distance(x,Y,Sigma)
if nargin==0
    VEC_LEN = 10000;
    dim = 3;
    x = rand(dim,1);
    Y = rand(dim,VEC_LEN);
    Sigma = eye(dim) + rand(dim);
end

profile clear; profile on;
[D, times] = compute(x,Y,Sigma);
profile off; profile viewer; 
%% plotting
figure(1); clf; set(gcf, 'Name','Run Times'); 
bar(times);
figure(2); clf; set(gcf, 'Name','Speedup over naive for loop');
bar(max(times)./times);
end

function [D, times] = compute(x,Y,Sigma)
%% naive for loop
tic;
[d n] = size(Y);
D1 = zeros(1,n);
nu = zeros(d,1);
Inv_Sigma = inv(Sigma);
for i = 1 : n
    for j = 1 : d
        nu(j) = x(j) - Y(j,i);
    end
    D1(i) = nu'*Inv_Sigma* nu;
end
t_naive = toc
%% vectorized
tic;
nu = repmat(x,1,n) - Y; % what is the dimension of nu? What can be inferred from this?
D2 = sum(nu .* (Sigma\ nu),1); 
t_vec = toc
times =[t_naive t_vec];
if any(abs(D1 - D2)>1e-10)
    error('no match');
end
D= D2;
end