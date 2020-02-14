function motivating_example_1()
clc;

VEC_LEN = 10000;
X = rand(3,VEC_LEN); Sigma = eye(3) + rand(3,3);

%% doing the computations and profiling
profile clear; profile on;
[t_no_alloc,t_alloc,t_alloc_inv_sig,t_vectorized] = compute(X,Sigma,VEC_LEN);
profile off; profile viewer;
%% plotting

times = [t_no_alloc,t_alloc,t_alloc_inv_sig,t_vectorized];
figure(1); clf; set(gcf, 'Name','Run Times'); 
% plot(times,'kx','MarkerSize',5,'LineWidth',5); 
bar(times);
figure(2); clf; set(gcf, 'Name','Speedup over naive for loop');
bar(max(times)./times);

end

function [t_no_alloc,t_alloc,t_alloc_inv_sig,t_vectorized] = compute(X,Sigma,VEC_LEN)
% Trying to compute Y = x'*inv(Sigma)*x for all x in X

%% 1) naive way: naive for-loops without pre allocation
tic;
Y1 = [];
for i = 1 : VEC_LEN
    x = X(:,i);
    y = x'*inv(Sigma)*x;
    Y1 = [Y1 y];
end
t_no_alloc = toc
%% 2) using for loops with pre allocation
tic;
Y2 = zeros(1,VEC_LEN);
for i = 1 : VEC_LEN
    x = X(:,i);
    Y2(:,i) =  x'*inv(Sigma)*x;
end
t_alloc = toc
%% 3) optimization: calculating the inverse once!
tic;
Y3 = zeros(1,VEC_LEN);
Inv_Sig = inv(Sigma);
for i = 1 : VEC_LEN
    x = X(:,i);
    Y3(:,i) = x'*Inv_Sig*x;
end
t_alloc_inv_sig = toc
%% 4) using vectorized processing
tic;

% IS Y = X' * Inv_Sig * X; CORRECT? WHY?

Y4 = sum(X .* (Sigma\ X),1); 
t_vectorized = toc

% IS THIS A CORRECT WAY TO CHECK IF THE COMPUTATIONS ARE CORRECT? WHY?
% if any(Y1(:) ~= Y2(:) | Y2(:) ~= Y3(:) | Y3(:) ~= Y4(:))      
%     error('no match!');
% end


if any(sum(abs(Y1(:) - Y2(:)))>1e-10 || sum(abs(Y1(:) - Y3(:)))>1e-10 ||  sum(abs(Y1(:) - Y4(:)))>1e-10)
    error('no match!');
end
end