% ĩ�˸����˶���6�ؽڲ���������
clc
clear all
identificationModel = 'Internal';
robotType = 'I5';
Robot = get_cad_model_para(robotType);     
Traj = set_excitation_traj_feature();
sampleRate = 200;
ARM_DOF = Robot.DOF;
DH = Robot.DH;

%% ����ģ�����ݣ�
N = 100;
toolPara = ones(10,1);

move_joint = [1,4]';
n_move = size(move_joint,1);

all_joint  = 1:1:ARM_DOF;
n_null = all_joint;
n_null(move_joint) = [];


%% ��ʶ��
addpath(genpath('C:\Users\Sherry\Desktop\MyRobot\dataLib\payloadCalibrationData'))

datafile = 'jointStatusRecord_tool.txt';
datafile_no = 'jointStatusRecord_nopayload.txt';
motionParaCoeff = 'qq_lizy1.mat';

postData = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate); % Ԥ����
postData_no = post_sensor_data_process(Robot, Traj, datafile_no, motionParaCoeff, sampleRate); % Ԥ����
postData.currentData = postData.currentData;
postData_no.currentData = postData_no.currentData;
tauTool = postData.currentData - postData_no.currentData;

HH = [];
WW = [];

for i = 1:1:size(postData.motionTraj.q,2)
    motionPara.q = postData.motionTraj.q(:,i);
    motionPara.qd = postData.motionTraj.qd(:,i);
    motionPara.qdd = postData.motionTraj.qdd(:,i);
    % forward �������˵Ľ��ٶȣ��Ǽ��ٶȣ��ؽں����ĵ��ٶȺͼ��ٶ�
    motionPara.q(n_null) = 0;
    motionPara.qd(n_null) = 0;
    motionPara.qdd(n_null) = 0;
    
    link = get_link_velocity(Robot, motionPara);
    
    T = [[]];
    for j = 1:1:ARM_DOF
        T{j} = homogeneous_transfer(DH(j,1),DH(j,2),DH(j,3),DH(j,4) + motionPara.q(j));
    end
    
    Wii = get_rigid_body_wrench_linear_matrix(link.w{ARM_DOF},link.wd{ARM_DOF},link.vd{ARM_DOF});  % wii wrench in joint 6;
    HH = [HH;Wii];% ,wrench in joint 6; if have force sensor in flange ; only need to jonit 5,6
    
    for j = ARM_DOF:-1:1
        if(ARM_DOF == j)
            T1 = eye(4);
        else
            T1 = T{j+1}*T1;
        end
        HA{j} = trans_wrench_linear_matrix(T1, Wii);

    end
    for k = 1:1:6
        S1 = [0,0,0,0,0,1] * HA{k}; % �����˶� �� joint j ����������
        WW = [WW;S1];
    end
end

[Col,beta] = get_mini_para_set_numerical(HH);
[Col1,beta1] = get_mini_para_set_numerical(WW);

tau = [];
for i = 1:size(tauTool,2)
    tau = [tau;tauTool(:,i)];
end

W1 = WW;
Cond = cond(W1)
para_OLS = inv(W1'*W1) * W1'*tau

% ��������
center = para_OLS(7:9)/para_OLS(10)
% I C M
Para_tool = [para_OLS(1:6);center;para_OLS(10)]


% 
% function Wii = get_rigid_body_wrench_linear_matrix(w,wd,vd)
% % Fi = Wii * Para_i
% Wii = [zeros(3,6) skew(wd)+skew(w)*skew(w) vd;skewStar(wd)+skew(w)*skewStar(w) -skew(vd) zeros(3,1)]; %Wii
% end
% 
% function HA = trans_wrench_linear_matrix(T, Wii)
% RR = [T(1:3,1:3) zeros(3,3); skew(T(1:3,4))*T(1:3,1:3) T(1:3,1:3)]; %TAi
% HA = RR * Wii;
% end








