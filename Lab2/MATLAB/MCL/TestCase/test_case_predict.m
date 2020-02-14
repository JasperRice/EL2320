function valid = test_case_predict()
load test_case_prediction.mat;
errs = ones(1,NUM_TEST);
S_bar = zeros(4,PARTICLE_NUM,NUM_TEST);
for i = 1 : NUM_TEST
    s=S(:,:,i);
    v = V(i);
    r = diag(R(:,i));
    omega = OMEGA(i);
    for t = 1 : RUN_TESTS_AVERAGE
        [s_bar] = predict(s,v,omega,r,delta_t);
        S_bar(:,:,i) = S_bar(:,:,i) + s_bar;
    end
    S_bar(:,:,i) = S_bar(:,:,i)./RUN_TESTS_AVERAGE;
%     S_bar(3,:,i) = mod(S_bar(3,:,i) + pi,2*pi)-pi;
    err = S_BAR(1:3,:,i) -S_bar(1:3,:,i);
    errs(i) = mean(sqrt(sum((err).^2,1)));
end
mse_err = mean(errs .^2);
THRESH_VALID = 1e-3;
fprintf('prediction mse = %f\n',mse_err);
valid = mse_err < THRESH_VALID;
end