function valid = test_case_observation_model()
load test_case_obs.mat;
errs = ones(1,NUM_TEST);
z= zeros(2,PARTICLE_NUM,NUM_TEST);
for i = 1 : NUM_TEST
    s = S_BAR(:,:,i);
    z(:,:,i) = observation_model(s,W,J(i));
    err = z(:,:,i) - Z(:,:,i);
    errs(i) = norm(err);
end   
mse_err = mean(errs.^2);
THRESH_VALID = 1e-20;
valid = mse_err < THRESH_VALID;

end