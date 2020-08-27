clc;clear all; close all; format short
%% initiall
global Robot Traj %#ok<NUSED>
robotType = 'I5';
ParaCAD(robotType); % robot para
trajParaOption(); %
%% ==========================================esimate ==========================================
% load data
% load('sensor_avg_yuan_1_off.mat')
load('sensor_avg_yuan_1.mat')
load('motionTraj_yuan_1.mat')

identificationModel = 'External';
q_all = motionTraj_f.q;
qd_all = motionTraj_f.qd;
qdd_all = motionTraj_f.qdd;
current_all = sensor_avg_2;
data_num = size(q_all,1);
%% identification matrix % order [[Ixx1, Ixy1, Ixz1, Iyy1, Iyz1, Izz1, Mx1,My1,Mz1,M1...(1:60)],[fok1...(61:66)],]
[index, MDP] = GetRobotIdyMimParaSet('External');
regression = [];
for i = 1:1:data_num
    motionPara.q = q_all(i,:)';
    motionPara.qd = qd_all(i,:)';
    motionPara.qdd = qdd_all(i,:)';
    HH = IdentificationMatrix(motionPara,identificationModel);
%     HI = inertiaMatrix(motionPara);
    HI = [];
    HO = eye(6);
    HA = [];
    for j = 1:1:6
        % in order [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,Ia, fv,fc,fok
        % ](1:14), change order to compara with python
        H_I = [HH(:,10*(j-1) + 1 : 10*j),HO(:,j)];
        HA = [HA,H_I];
    end
    regression = [regression;HA];
    wrench(i,:) = (HH(:,index) * MDP)';
end

%% cal joint torque from current
tau = [];
tau_cad = [];
for i = 1:data_num
    tau = [tau;current_all(i,:)'];
    tau_cad = [tau_cad; wrench(i,:)'];
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
regression = [regression(:,33),regression];
regression(:,34) = [];

[Col,beta] = getMiniPara(regression);

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
    % %可行性矫正(BPFC)
    cvx_begin sdp
    variable x1(1)
    variable x2(n_b)
    variable x3(n_d)
    minimize(x1)
    subject to
    blkdiag([vec(x1),(para_OLS - vec(x2))';para_OLS - vec(x2), eye(n_b)],  getDext(P * K * [vec(x2);vec(x3)],6,eps)) >= 0
    cvx_end
    para.bpfc = x2;
    
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
%% ************************ post process ***********************
%% 最小参数集的符号表达式
if 1
    XX = sym('XX', [1 6],'real');
    XY = sym('XY', [1 6],'real');
    XZ = sym('XZ', [1 6],'real');
    YY = sym('YY', [1 6],'real');
    YZ = sym('YZ', [1 6],'real');
    ZZ = sym('ZZ', [1 6],'real');
    MX = sym('MX', [1 6],'real');
    MY = sym('MY', [1 6],'real');
    MZ = sym('MZ', [1 6],'real');
    M = sym('M', [1 6],'real');
    
    Ia = sym('Ia', [1 6],'real');
    fv = sym('fv', [1 6],'real');
    fc = sym('fc', [1 6],'real');
    fok = sym('fok', [1 6],'real');
    
    Sp = [];
    for i = 1:1:6
        %         Sp = [Sp;[XX(i),XY(i),XZ(i),YY(i),YZ(i),ZZ(i),MX(i),MY(i),MZ(i),M(i),Ia(i),fv(i),fc(i),fok(i)]'];
        Sp = [Sp;[XX(i),XY(i),XZ(i),YY(i),YZ(i),ZZ(i),MX(i),MY(i),MZ(i),M(i),fok(i)]'];
%         Sp = [Sp;[XX(i),XY(i),XZ(i),YY(i),YZ(i),ZZ(i),MX(i),MY(i),MZ(i),M(i)]'];

    end

    Sp = [Sp(33);Sp];
    Sp(34) = [];
    
    X1 = Sp(Col.i)
    X2 = Sp(Col.c);
    expersion = vpa(X1 + beta * X2);
end
%% CAD para change order
% %% CAD PARA
% Ij = Robot.Para.DP.J;
% MS = Robot.Para.DP.MS;
% M = Robot.Para.DP.M;
% para_cad = Robot.Para_cad;
% % change order
% for i = 1:Robot.DOF
%     P{i} = [Ij{i}(1,1) Ij{i}(1,2) Ij{i}(1,3) Ij{i}(2,2) Ij{i}(2,3) Ij{i}(3,3) MS{i}(1) MS{i}(2) MS{i}(3) M(i)]';
% end
% Robot.Para_cad = [P{1}; P{2}; P{3}; P{4}; P{5}; P{6}]; % 60 * 1
% 
% para_cad_ = para_cad(Col.i) + beta * para_cad(Col.c);


%% cal error
para_all = [para.OLS, para.fbpe_ols]
for i = 1:1:size(para_all,2)
    E(i) = norm(tau - W1 * para_all(:,i))^2;
end
E

%% standard deviations of estimate para
q1_square = (norm(tau - W1 * para_OLS)^2)/(size(tau,1) - size(W1,2));
e_beta = sqrt(q1_square .* diag(inv(W1'*W1)));
D_beta = e_beta./abs(para_OLS)

%% validation
NUM = 3;

q_all = load(['aubo_q_',num2str(NUM),'.txt']);
qd_all = load(['aubo_qd_',num2str(NUM),'.txt']);
qdd_all = load(['aubo_qdd_',num2str(NUM),'.txt']);
current_all = load(['aubo_current_',num2str(NUM),'.txt']);
data_num = size(q_all,1);

% identification matrix
T_cad = [];
regression = [];

for i = 1:1:data_num
    motionPara.q = q_all(i,:)';
    motionPara.qd = qd_all(i,:)';
    motionPara.qdd = qdd_all(i,:)';
    HH = IdentificationMatrix(motionPara,identificationModel);
    HF = frictionMatrix(motionPara);
    HI = inertiaMatrix(motionPara);
    HA = [];
    for j = 1:1:6
        % order [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,Ia, fv,fc,fok ,
        % ](1:14),[],[]
        H_I = [HH(:,10*(j-1) + 1 : 10*j), HI(:,j), HF(:,3*(j-1) + 1 : 3*j)];
        HA = [HA,H_I];
    end
    regression = [regression;HA];
    %     regression = [regression;[HH,HF,HI]];
end
tau = current_all;
for i = 1:3
    A = regression(:,Col.i);
    b = A * para(:,i);
    b1 = (reshape(b,[6,2000]))';
    Er(i) = norm(tau - b1)/norm(tau);
end
Er


%% plot
if 0
    close all
    figure(1)
    
    for i = 1:1:6
        subplot(2,3,i);
        plot(tau(:,i),'r')
        hold on
        plot(b1(:,i),'b')
        hold on
        plot(b2(:,i),'g')
        hold on
        plot(error1(:,i),'b')
        plot(error2(:,i),'m')
        
        title(['joint ',num2str(i),'of data' ,num2str(NUM)])
        xlabel('Time')
        ylabel('Torque(Nm)')
        hold off
    end
    legend('tau measure(tau)','est1(tau) lg','est2(tau) zy','err1','err2')
end
















