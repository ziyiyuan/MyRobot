clc; clear all; close all
format short
%% initialize
global Robot Traj %#ok<NUSED>
robotType = 'I5';
ParaCAD(robotType); % robot para
trajParaOption(); %
%%
load('qq1.mat');
sampleRate = 200;
motionPara = ObtainMotionPara(qq, sampleRate);
%% 计算衔接点p v a, 用于III 型算法计算衔接线段,注意两段需要分别规划；第一段 qd1 = -qd1;
% q1 = motionPara.q(1,:);
% qd1 = motionPara.qd(1,:);
% qdd1 = motionPara.qdd(1,:);
% qt = motionPara.q(end,:);
% qdt = motionPara.qd(end,:);
% qddt = motionPara.qdd(end,:);
%% 加载衔接曲线段；
load('pva1.txt');
load('pva2.txt');
% p3 = pva3(1:3:end,:);
% v3 = pva3(2:3:end,:);
% a3 = pva3(3:3:end,:);
% figure 
% subplot(3,1,1)
% plot(p1)
% subplot(3,1,2)
% plot(v1)
% subplot(3,1,3)
% plot(a1)
% hold off

p2 = pva2(1:3:end,:);
v2 = pva2(2:3:end,:);
a2 = pva2(3:3:end,:);

p1i = pva1(1:3:end,:);
v1i = pva1(2:3:end,:);
a1i = pva1(3:3:end,:);

p1 = p1i(end:-1:1,:);
v1 = -v1i(end:-1:1,:); % 速度对称翻转
a1 = a1i(end:-1:1,:);
% 曲线拼接
T = 2;%% 定义周期
pt = [];
pdt = [];
pddt = [];

for i= 1:1:T
    pt = [pt;motionPara.q];
    pdt = [pdt;motionPara.qd];
    pddt = [pddt;motionPara.qdd];
end
p_all = [p1;pt;p2];
v_all = [v1;pdt;v2];
a_all = [a1;pddt;a2];

%% plot
figure 
subplot(3,1,1)
plot(p_all)
subplot(3,1,2)
plot(v_all)
subplot(3,1,3)
plot(a_all)
hold off

%% validation : 计算力和力矩是否超量程
identifyModel = 'External';

if strcmp(identifyModel,'Internal')
    torLimit = Robot.Limit.torque;
    posLimit = Robot.Limit.q;
elseif strcmp(identifyModel,'External')
    torLimit = Robot.Limit.sensor;  %%%input the sensor force limit
    posLimit = Robot.Limit.sensorP; %% 防止碰到基座传感器；
end
velLimit = Robot.Limit.qd;
accLimit = Robot.Limit.qdd;

ARM_DOF = Robot.DOF;
valid = 1;
for i = 1:1:size(p_all,1)
    kinematicsPara.q = p_all(i,:);
    kinematicsPara.qd = v_all(i,:);
    kinematicsPara.qdd = a_all(i,:);
    if strcmp(identifyModel,'Internal')
        torque = identificationModel(kinematicsPara,'Internal'); %% torque in joint
    elseif strcmp(identifyModel,'External')
        torque =  identificationModel(kinematicsPara,'External',FKLink(kinematicsPara.q,1)); %% iter NE  wrench inbase
        % torque = GetBaseWrenchBasepara(kinematicsPara);
    end
    
    for j = 1:ARM_DOF
        if abs(torque(j)) > torLimit(j)
            valid = 0;
            break;
        end
    end
    totalTorque(:,i) = torque;   
end
figure(2)
plot(totalTorque')
title('torque')
max(totalTorque')

%% 生成 offt 文件
if valid == 1
    fid=fopen(['C:\Users\Sherry\Desktop\MyRobot\Dyn_i_0707\20200812/', 'data1_0820','.offt'],'w');
    for i = 1:size(p_all,1)
        fprintf(fid, '%f%s%f%s%f%s%f%s%f%s%f\r\n',(p_all(i,1)),',',(p_all(i,2)),',',(p_all(i,3)),',',(p_all(i,4)),',',(p_all(i,5)),',',(p_all(i,6)));
    end
    fclose(fid);
end
%%




