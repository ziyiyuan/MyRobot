%% test SDP out put form
clc
clear all
robotType = 'I5';
Robot = get_cad_model_para(robotType);
Traj = set_excitation_traj_feature();
sampleRate = 200;
% identificationModel = 'External';
identificationModel = 'Internal';
addConstraints.M.cons = 1;
addConstraints.M.offset = 1;
addConstraints.M.reff = Robot.Para.DP.Mreff;

estILS = 0; %采用增量最小二乘
% 
% addpath(genpath('C:\Users\Sherry\Desktop\MyRobot\dataLib\dynamicCalibrationData\data_20200908'))
% datafile = 'jointStatusRecord_lizy2.txt';
% motionParaCoeff = 'qq_lizy2.mat';
% postData = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate);
% save postData postData
 load('postData.mat')

 dlmwrite('qi.txt',postData.motionTraj.q','delimiter',',')
 dlmwrite('qdi.txt',postData.motionTraj.qd','delimiter',',')
 dlmwrite('qddi.txt',postData.motionTraj.qdd','delimiter',',')
 dlmwrite('currentData.txt',postData.currentData','delimiter',',')
%% get CADPara
identifyPara.linkModel = 1;
identifyPara.offsetModel = 1;
identifyPara.frictionModel = 1;
identifyPara.roterInertiaModel = 1;

%% load data
current_all = postData.currentData;
data_num = size(postData.motionTraj.q,2);

%% cal identification matrix and joint torque/wrench from current
regression = cal_identification_matrix(Robot, postData.motionTraj, identificationModel, identifyPara);
tau = [];
for i = 1:data_num
    tau = [tau;current_all(:,i)];
end
%% ************************ estimation ***********************
%% OLS
[Col,beta] = get_mini_para_set_numerical(regression);
[q1,r1] = qr(regression);
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
    %     block = [59,6,6,6,6,6,6,ones(1,18)];
    block = [125];
    
    y = zeros(1,85);
    record = [];
    for i = 0:1:85
        y = zeros(1,85);
        if i ~= 0
            y(i) = 1;
        end
        y1 = y(1);
        y2 = y(1+1:1+n_b)';
        y3 = y(2+n_b:end)';
        eqn = blkdiag([y1 - norm(pp2)^2, (pp1 - R1*vec(y2))';pp1 - R1*vec(y2), eye(n_b) ],  get_Dext(P * K * [vec(y2);vec(y3)],6,eps,identifyPara, addConstraints));
        if i == 0
            eqn0 = -eqn;
            eqn = eqn0;
        else
            eqn = eqn + eqn0;
        end
        if(i == 82)
            a = 1;
        end
        se = 0;
        for m = 1:1:size(block,2)
            se = se + block(m);
            si = se - block(m) + 1;
            for j = si :1:se
                for  k = j :1:se
                    if(abs(eqn(j,k)) > 1e-8)
                        record = [record;i,m,j-si+1,k - si+1,eqn(j,k)];%% Fi, block_m  j,k元素  value
                    else
                        eqn(j,k) = 0;
                    end
                end
            end
        end
    end
    dlmwrite('sdp.txt',record,'delimiter','\t')
    
    
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