clc;clear all; close all; format short
%% initiall
global Robot Traj %#ok<NUSED>
robotType = 'I5';
ParaCAD(robotType); % robot para
trajParaOption(); %

%% ====================================data process===========================================
%% load data
data_all = load('jointStatusRecord_yuan1_1.txt');
load('qq_yuan1.mat');
% data_all = load('jointStatusRecord_lizy4.txt');
% load('qq_lizy4.mat');
qc = data_all(:,1:6);
qs = data_all(:,7:12);
Ic = data_all(:,13:18);

sensordata = data_all(:,19:24);
motionTraj = ObtainMotionPara(qq,Traj.sampleRate);
N_data = size(qc,1);

CN = Traj.sampleRate * Traj.TrajectoryPeriod; % 采样频率 * 轨迹周期
CyclePeriod = 40;
if N_data < CyclePeriod * CN
    CyclePeriod = 20;
end
%% plot row data
figure(1)
plot(qc(:,1),'r')
title('row joint data')
%
N = 3000;
figure(2)
plot(qc(1:N,1),'r')
hold on
plot(motionTraj.q(:,1))
legend('qc','q');
%% 对 qc 和 qs 相对于优化的轨迹进行对齐
close all
P = CyclePeriod - 1;
count_N = P * CN;% 对齐参与计算点的个数；
e1 = 10;
for i = 1:1:CN
    q = qc(i : count_N + i - 1,: );
    sum = zeros(CN, 6);
    for j = 1:1:P
        ss = q(CN * (j-1) + 1 : CN * j,:);
        sum = sum + ss;
    end
    q = sum/P;
    e = norm(q - motionTraj.q);
    if e < e1
        e1 = e;
        index = i;
    end
end
start_qc = index;

figure(1)
plot(qc(start_qc:start_qc + 2000 - 1,1))
hold on
plot(motionTraj.q(:,1))
legend('qc','q')
title('joint data after aligned ')
%% average qc data and sensor data and current data
% average q data
close all
q = qc(start_qc: CyclePeriod * CN + start_qc - 1,: );
sum = zeros(CN, 6);
for i = 1:1:CyclePeriod
    st = [CN * (i-1) + 1 ,CN * i];
    ss = q(CN * (i-1) + 1 : CN * i,:);
    sum = sum + ss;
end
q = sum/CyclePeriod;

figure(1)
plot(motionTraj.q(:,1))
hold on
plot(q(:,1))
legend('qs','qc');
title('joint data after averaged ')

% average sensor data
sensor_avg = sensordata(start_qc: CyclePeriod * CN + start_qc - 1,: );
sum = zeros(CN, 6);
for i = 1:1:CyclePeriod
    st = [CN * (i-1) + 1 ,CN * i];
    ss = sensor_avg(CN * (i-1) + 1 : CN * i,:);
    sum = sum + ss;
end
sensor_avg = - sum/CyclePeriod; % 反作用力; 作用在连杆端的力；

figure(2)
plot(sensor_avg)
legend('Fx','Fy','Fz','Tx','Ty','Tz')
title('sensor data after averaged ')

%% 传感器数据滤波，关节角采用同样的滤波方式，保持相位一致
close all
fc = 10 * Traj.OrderNumber * Traj.frequency;  % 截止频率
fs = Traj.sampleRate; % 采样频率
order = 6;
[b,a] = butter(order,fc/(fs/2));
freqz(b,a)
sensor_avg_1 = filter(b,a,sensor_avg);
motionTraj_f.q = filter(b,a,motionTraj.q);
motionTraj_f.qd = filter(b,a,motionTraj.qd);
motionTraj_f.qdd = filter(b,a,motionTraj.qdd);

figure(1)
plot(sensor_avg_1)
legend('Fx','Fy','Fz','Tx','Ty','Tz')
title('filtered sensor data')

figure(2)
title('compara sensor data, filtered before and after')
for i = 1:1:6
    subplot(2,3,i);
    plot(sensor_avg(:,i))
    hold on
    plot(sensor_avg_1(:,i))
    hold off
end


figure(3)
plot(motionTraj.qd(:,1))
hold on
plot(motionTraj_f.qd(:,1))
title('compara joint data, filtered before and after') % 观察相位差；

% 去掉第一个周期失真数据
sensor_avg_s = sensor_avg_1(fs+1:end,:);
motionTraj_s.q = motionTraj_f.q(fs+1:end,:);
motionTraj_s.qd = motionTraj_f.qd(fs+1:end,:);
motionTraj_s.qdd = motionTraj_f.qdd(fs+1:end,:);

% % 传感器偏执 G + T_off = F
% G = CalG([0,0,0,0,0,0]') %%% 奇异时算不了
% motionPara.q = zeros(6,1);
% motionPara.qd = zeros(6,1);
% motionPara.qdd = zeros(6,1);
% G = identificationModel(motionPara,'External',FKLink(motionPara.q,1))
% S_offset = sensordata(1,:) + G';
% for i = 1: size(sensor_avg_s,1)
%     sensor_avg_s(i,:) = sensor_avg_s(i,:) + S_offset;
% end
%% CAD 数据力矩曲线和测量力矩对比
close all
W = [];
[index, MDP] = GetRobotIdyMimParaSet('External');
for i = 1:1:size(sensor_avg_s,1)
    motionPara.q = motionTraj_s.q(i,:);
    motionPara.qd = motionTraj_s.qd(i,:);
    motionPara.qdd = motionTraj_s.qdd(i,:);
    HH = IdentificationMatrix(motionPara,'External');
    wrench(i,:) = (HH(:,index) * MDP)';
end

figure(1)
title_name = {'Fx','Fy','Fz','Tx','Ty','Tz'};
for i = 1:1:6
    subplot(2,3,i);
    plot(wrench(:,i))
    hold on
    plot(sensor_avg_s(:,i))
    title(title_name{i})
    xlabel('Time')
    ylabel('Torque(Nm)')
    hold off
end
legend('CAD','Sensor')
%% *********************************************************************************************************
%%==========================================esimate ==========================================
%%*********************************************************************************************************
close all
identificationModel = 'External';
q_all = motionTraj_s.q;
qd_all = motionTraj_s.qd;
qdd_all = motionTraj_s.qdd;
current_all = sensor_avg_s;
data_num = size(q_all,1);
%% identification matrix % order [[Ixx1, Ixy1, Ixz1, Iyy1, Iyz1, Izz1, Mx1,My1,Mz1,M1...(1:60)],[fok1...(61:66)],]
[index, MDP] = GetRobotIdyMimParaSet('External');
regression = [];
for i = 1:1:data_num
    motionPara.q = q_all(i,:)';
    motionPara.qd = qd_all(i,:)';
    motionPara.qdd = qdd_all(i,:)';
    HH = IdentificationMatrix(motionPara,identificationModel);
    HO = eye(6);
    HA = [];
    for j = 1:1:6
        % in order [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,Ia, fv,fc,fok
        % ](1:14), change order to compara with python
        H_I = [HH(:,10*(j-1) + 1 : 10*j),HO(:,j)];
%         H_I = [HH(:,10*(j-1) + 1 : 10*j)];
        HA = [HA,H_I];
    end
    regression = [regression;HA];
end

%% cal joint torque from current
tau = [];
tau_cad = [];
for i = 1:data_num
    tau = [tau;current_all(i,:)'];
end

%% 增量最小二乘
% H3 = [];T3 = [];
% for i= 1:1:data_num/2
%     H1 = regression(6*(i-1) + 1 : 6*i ,:);
%     H2 = regression(6*(data_num/2 + i-1) + 1:6*(data_num/2 + i),:);
%     H3 = [H3;H2-H1];
%     T1 = tau(6*(i-1) + 1 : 6*i ,:);
%     T2 = tau(6*(data_num/2 + i-1) + 1:6*(data_num/2 + i),:);
%     T3 = [T3;T2 - T1];
% end
% regression = H3;
% tau = T3;
%% ************************ estimation ***********************
%% OLS
[Col,beta] = getMiniPara(regression);
W1 = regression(:,Col.i);

if 1
    para_OLS = inv(W1'*W1) * W1'*tau;
    para.OLS = para_OLS;
    if 1
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
        
        eps = 1e-6;
        % 可行性测试（BPFT）
        n = n_d;
        cvx_begin sdp
        variable x(n_d)
        minimize(0)
        subject to
        getDext(P * K * [para_OLS;vec(x)],6,eps) >= 0
        cvx_end
        %
        % % FBPE_OLS
        cvx_begin sdp
        variable x1 % mu
        variable x2(n_b) % beta
        variable x3(n_d) % delta_d
        minimize(x1)
        subject to
        blkdiag([x1 - norm(pp2)^2, (pp1 - R1*vec(x2))';pp1 - R1*vec(x2), eye(n_b) ],  getDext(P * K * [vec(x2);vec(x3)],6,eps)) >= 0
        cvx_end
        para.fbpe_ols = x2;
    end
    
    para_all = [para.OLS, para.fbpe_ols]
    for i = 1:1:size(para_all,2)
        E(i) = norm(tau - W1 * para_all(:,i))^2/norm(tau);
    end
    E
    
%     save para_all para_all
end

%% standard deviations of estimate para
% set offset 
para_2 = para_all;
para_1 = load('para_all.mat')
para_2([6,7,15,30,38,46],:) = para_1.para_all([6,7,15,30,38,46],:);

for i = 1:1:size(para_2,2)
    e(i) = norm(tau - W1 * para_2(:,i))^2/norm(tau);
end
e















