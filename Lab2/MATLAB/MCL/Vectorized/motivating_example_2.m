function motivating_example_2()
clc;
VEC_LEN = 10000;
DIM = 128;
X1 = rand(DIM,VEC_LEN);
x2 = rand(DIM,1);
%% doing the computations and profiling

profile clear; profile on;
[t_2_for_loops t_1_for_loop t_vec] = compute(X1,x2,DIM,VEC_LEN);
profile off; profile viewer;

%% plotting
times = [t_2_for_loops t_1_for_loop t_vec];
figure(1); clf; set(gcf, 'Name','Run Times');
bar(times);
figure(2); clf; set(gcf, 'Name','Speedup over naive for loop');
bar(max(times)./times);
end

function [t_2_for_loops t_1_for_loop t_vec] = compute(X1,x2,DIM,VEC_LEN)
%% 1) naive for loops
tic;
Y1 = zeros(1,VEC_LEN); % zeros(1,VEC_LEN) or zeros(VEC_LEN,1)? WHY?
for i = 1 : VEC_LEN
    for k = 1 : DIM
        dist2dim = (x2(k) - X1(k,i))^2;
        Y1(i) = Y1(i) + dist2dim;
    end
    Y1(i) = sqrt(Y1(i));
end
t_2_for_loops = toc
%% 2) vectorizing the loop over the dimension
tic;
Y2 = zeros(1,VEC_LEN);
for i = 1 : VEC_LEN
    dist2 = sum((x2 - X1(:,i)).^2);
    Y2(i) = sqrt(dist2);
end
t_1_for_loop = toc
%% 3) full vectorization
tic;
x2r = repmat(x2,1,VEC_LEN);
dists2 = sum((x2r - X1).^2,1);
Y3 = sqrt(dists2);
t_vec = toc
if any( abs(Y1-Y2)>1e-10 |  abs(Y1-Y3)>1e-10)
    error('no match');
end
end