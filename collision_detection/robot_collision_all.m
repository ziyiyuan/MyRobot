clc
clear all
close all
robotType = 'I5';
Robot = get_cad_model_para(robotType);
Traj = set_excitation_traj_feature();
sampleRate = 200;
% identificationModel = 'External';
identificationModel = 'Internal';
addConstraints.M.cons = 1;
addConstraints.M.offset = 1;
addConstraints.M.reff = Robot.Para.DP.Mreff;
%% load data
load('postData.mat')
dt = 0.005;
%%
L1 = load('L1_c.txt');
q_all = L1(:,1:2:12);
I_all = L1(:,2:2:12)./Robot.Para.TC';
% 一阶差分
qd_all = diff(q_all,1,1);
qdd_all = diff(qd_all,1,1);
% 
index = 1; % index collision

figure(1)
plot(I_all(:,index))

%%
%$ collision point
F_ext = 20;
N = size(postData.motionTraj.q,2);
T = N*dt;
num_p = 10;
t = 0:dt:T;
% y = (-square(2*pi/(T/num_p)*t,50) + 1)/2;
% tau1 = y(1:N)*10; %% 方形波

tau1 = zeros(1,N);
f_index = [30,31,32,33,34,35,36,37,100,101,102,800];
tau1(f_index) = F_ext; % 脉冲波形

tau_f = ones(6,1)*tau1;
postData.currentData = postData.currentData - tau_f;
plot(postData.currentData(1,:)')
%%

K0 = 150*eye(6);
res = [];
load('paraEst.mat')
load('Col.mat')
para = paraEst(:,2);
%% data process 减去摩擦力的影响
fv = zeros(6,4);
para_f_index = [];
for i = 1:1:6
    for j = 11:1:14
        if ismember(14*(i-1)+j, Col.i)
            index = find(Col.i==14*(i-1)+j);
            fv(i,j-10) = para(index);
            para_f_index = [para_f_index,index];
        else
            fv(i,j-10) = 0;
        end
    end
end
a1 = 1:1:size(para,1);
a1(para_f_index) = [];
para_link_index = a1;

L_i = Col.i(para_link_index);
F_i = Col.i(para_f_index);

EYE = eye(6);

% 利用最小参数集计算
for i = 1:1:N
        q = postData.motionTraj.q(:,i);
        qd = postData.motionTraj.qd(:,i);
%     q = q_all(i,:)';
%     qd = qd_all(i,:)';
    
    % 计算摩擦力
        qdd = postData.motionTraj.qdd(:,i);
%     qdd = zeros(6,1);
    for j = 1:1:6
        Ff(j,1) = fv(j,2) .* qd(j) + fv(j,3) .* sign(qd(j)) +  fv(j,4) + fv(j,1) * qdd(j);
    end
    % % 减去摩擦力后的测量力矩
        Tm = postData.currentData(:,i) - Ff;
%     Tm = I_all(i,:)' - Ff;
    
    %力矩误差
    motionPara.q = q;
    motionPara.qd = qd;
    motionPara.qdd = qdd;
    regression = cal_ele_identification_matrix_SDP(Robot, motionPara, identificationModel);
    ET(:,i) = regression(:,Col.i) * paraEst(:,2) - postData.currentData(:,i);
    ET1(:,i) = regression(:,L_i) *  para(para_link_index,1) - Tm;
    
    % 计算重力 G, qd = 0； qdd = 0；
    motionPara.q = q;
    motionPara.qd = zeros(6,1);
    motionPara.qdd = zeros(6,1);
    regression = cal_ele_identification_matrix_SDP(Robot, motionPara, identificationModel);
    G = regression(:,L_i) * para(para_link_index,1);
    
    % 计算M
    motionPara.q = q;
    motionPara.qd = zeros(6,1);
    for j = 1:1:6
        motionPara.qdd = EYE(:,j);
        regression = cal_ele_identification_matrix_SDP(Robot, motionPara, identificationModel);
        M(:,j) = regression(:,L_i) * para(para_link_index,1) - G;
    end
    
    % 计算C*qd + G
    motionPara.q = q;
    motionPara.qd = qd;
    motionPara.qdd = zeros(6,1);
    regression = cal_ele_identification_matrix_SDP(Robot, motionPara, identificationModel);
    CG = regression(:,L_i) * para(para_link_index,1);
    
    ET2(:,i) = M*qdd + CG - Tm;
    
    %%
    % 动量
    Pt = M *qd;
    
    % 计算 beta = G - C' * qd =  G + C*qd - Md*qd
    if i == 1
        P_hat = zeros(6,1);
        Mk_1 = M;
        Rk_1 = zeros(6,1);
        P0 = Pt;
    end
    beta = CG - (M - Mk_1)*qd./dt;
    P_hat = P_hat + (Tm - beta + Rk_1).* dt;
    
    
    R = K0*(Pt - P_hat - P0);
    res = [res,R];
    Mk_1 = M;
    Rk_1 = R;
end

figure
for i = 1:1:6
    subplot(2,3,i);
    plot(res(i,:))

    hold on
    plot([0,N],[F_ext,F_ext],'--')
    
    hold on
    plot(f_index,F_ext,'r-o')

    legend('est_f','F_input')
end






function regression = cal_ele_identification_matrix_SDP(Robot, motionTraj, identificationModel)

identifyPara.linkModel = 1;
identifyPara.roterInertiaModel = 1;
identifyPara.frictionModel = 1;
identifyPara.offsetModel = 1;

regression = [];
for i = 1
    motionPara.q = motionTraj.q(:,i);
    motionPara.qd = motionTraj.qd(:,i);
    motionPara.qdd = motionTraj.qdd(:,i);
    
    HH = cal_ele_identification_matrix(Robot, motionPara,identificationModel);
    HI = inertia_matrix(motionPara);
    HF = friction_matrix(motionPara);
    Hoff = offset_matrix(motionPara);
    % in order [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,Ia, fv,fc,fok
    % ](1:14), change order to compara with python
    HA = [];
    for j = 1:1:6
        H_i = [];
        if identifyPara.linkModel
            H_i = [H_i, HH(:,10*(j-1) + 1 : 10*j)];
        end
        if identifyPara.roterInertiaModel
            H_i = [H_i, HI(:,j)];
        end
        if identifyPara.frictionModel
            H_i = [H_i, HF(:,2*(j-1) + 1 : 2*j)];
        end
        if identifyPara.offsetModel
            H_i = [H_i, Hoff(:,j)];
        end
        HA = [HA,H_i];
    end
    regression = [regression;HA];
end
end









