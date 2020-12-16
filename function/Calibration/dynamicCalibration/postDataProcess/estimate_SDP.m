function [para,E] = estimate_SDP(Robot,identificationModel, postData,addConstraints,estILS)

identifyPara.linkModel = 1;
identifyPara.offsetModel = 1;
if strcmp(identificationModel,'External')
    identifyPara.frictionModel = 0;
    identifyPara.roterInertiaModel = 0;
elseif strcmp(identificationModel,'Internal')
    identifyPara.frictionModel = 1;
    identifyPara.roterInertiaModel = 1;
end

%% load data
if strcmp(identificationModel,'External')
    current_all = postData.sensorData;
elseif strcmp(identificationModel,'Internal')
    current_all = postData.currentData;
end
data_num = size(postData.motionTraj.q,2);

%% cal identification matrix and joint torque/wrench from current
regression = cal_identification_matrix(Robot, postData.motionTraj, identificationModel, identifyPara);
tau = [];
for i = 1:data_num
    tau = [tau;current_all(:,i)];
end
%% ************************ estimation ***********************
% 最小二乘增量辨识
if estILS % Incremental least squares
    regression1 = [];
    tau1 = [];
    for i= 1:1:data_num/2
        H1 = regression(6*(i-1)+1:6*i,:);
        H2 = regression(6*(i-1)+1 + data_num/2 * 6 : 6*i + data_num/2 * 6,:);
        regression1 = [regression1;H2 - H1];
        current1 = current_all(:,i);
        current2 = current_all(:,i + data_num/2);
        tau1 = [tau1;current2 - current1 ];
    end
    regression = regression1;
    tau = tau1;
end

%% OLS
[Col,beta] = get_mini_para_set_numerical(regression);

if 1
    W1 = regression(:,Col.i);
    para_OLS = inv(W1'*W1) * W1'*tau;
    para.OLS = para_OLS;
    %% BPFT
    n_b = size(Col.i,2);
    n_d = size(Col.c,2);
    
    P0 = eye(size(regression,2));
    P = [P0(:,Col.i),P0(:,Col.c)];
    
    Kd = beta;
    K = [eye(n_b),-Kd; zeros(n_d,n_b), eye(n_d)];
    
    W = regression(:,Col.i);
    [QQ,RR] = qr(W);
    
    R1 = RR(1:n_b,1:end);
    Q1 = QQ(:,1:n_b);
    Q2 = QQ(:,n_b+1:end);
    
    pp1 = Q1' * tau;
    pp2 = Q2' * tau;
    
    % 可行性测试（BPFT）
    n = n_d;
    cvx_begin sdp
    variable x(n_d)
    minimize(0)
    subject to
    get_Dext(P * K * [para_OLS;vec(x)],6,eps,identifyPara,addConstraints) >= 0
    cvx_end
    
    % %可行性矫正(BPFC)
    cvx_begin sdp
    variable x1(1)
    variable x2(n_b)
    variable x3(n_d)
    minimize(x1)
    subject to
    blkdiag([vec(x1),(para_OLS - vec(x2))';para_OLS - vec(x2), eye(n_b)],  get_Dext(P * K * [vec(x2);vec(x3)],6,eps,identifyPara,addConstraints)) >= 0
    cvx_end
    para.bpfc = x2;
    
    % % FBPE_OLS
    cvx_begin sdp
    variable x1 % mu
    variable x2(n_b) % beta
    variable x3(n_d) % delta_d
    minimize(x1)
    subject to
    blkdiag([x1 - norm(pp2)^2, (pp1 - R1*vec(x2))';pp1 - R1*vec(x2), eye(n_b) ],  get_Dext(P * K * [vec(x2);vec(x3)],6,eps,identifyPara, addConstraints)) >= 0
    cvx_end
    para.fbpe_ols = x2;
end
%% ************************ post process ***********************
%% 最小参数集的符号表达式
if 0
    [X1,expersion] = get_mini_para_set_symbol(identifyPara,Col,beta);
end
%% cal error
% para = [para.OLS, para.bpfc, para.fbpe_ols];
para = [para.OLS, para.fbpe_ols];

for i = 1:1:size(para,2)
    E(i) = norm(tau - W1 * para(:,i))/norm(tau);
end

%% standard deviations of estimate para
q1_square = (norm(tau - W1 * para_OLS)^2)/(size(tau,1) - size(W1,2));
e_beta = sqrt(q1_square .* diag(inv(W1'*W1)));
D_beta = e_beta./abs(para_OLS);

end