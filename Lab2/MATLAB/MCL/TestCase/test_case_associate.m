function valid = test_case_associate()
load test_case_associate.mat;
errs_psi = ones(1,NUM_TEST);
errs_outliers = ones(1,NUM_TEST);
psi = zeros(NUM_OBS,PARTICLE_NUM,NUM_TEST);
outliers = zeros(NUM_OBS,NUM_TEST);
for i = 1 : NUM_TEST
    s_bar = S_BAR(:,:,i);
    q = diag(Q(:,i));
    z_i = Z(:,:,i);
    [outliers(:,i),psi(:,:,i)] = associate(s_bar,z_i,W,LAMBDA(i),q);
    err_outlier = OUTLIERS(:,i) - outliers(:,i);
    err_psi = PSI(:,:,i) - psi(:,:,i);
    errs_psi(i) = norm(err_psi);
    errs_outliers(i) = norm(err_outlier);
end
mse_err_outliers = mean(errs_outliers.^2);
mse_err_psi = mean(errs_psi.^2);
THRESH_VALID = 1e-20;
valid_outliers = mse_err_outliers < THRESH_VALID;
if ~ valid_outliers
    fprintf('outliers computed in associate.m seem to be wrong, mse=%f\n',mse_err_outliers);
end
valid_psi = mse_err_psi < THRESH_VALID;
if ~ valid_psi
    fprintf('psi s computed in associate.m seem to be wrong, mse=%f\n',mse_err_psi);
end
valid = valid_outliers && valid_psi;
end