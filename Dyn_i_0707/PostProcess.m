clc;clear all; close all; format short
%% initiall
global Robot Traj %#ok<NUSED>
robotType = 'I5';
ParaCAD(robotType); % robot para
trajParaOption(); %
%% load data
data_all = load('jointStatusRecord_lizy1.txt');
load('qq_lizy1.mat');

qc = data_all(:,1:6);
qs = data_all(:,7:12);
Ic = data_all(:,13:18);
sensordata = data_all(:,19:24);

CN = Traj.sampleRate * Traj.TrajectoryPeriod; % 采样频率 * 轨迹周期
CyclePeriod = 40;
%% plot raw data
e = qc - qs;
max(max(e))
norm(qc - qs);
N = 8000;
figure(1)
plot(qc(1:N,1),'r')
hold on
plot(qs(1:N,1),'b')
motionTraj = ObtainMotionPara(qq,Traj.sampleRate);
hold on
plot(motionTraj.q(:,1))
legend('qc','qs','q');

%
P = CyclePeriod - 1;
count_N = P * 2000;% 对比1000 个点
%% 对 qc 和 qs 相对于优化的轨迹进行对齐
e1 = 1000;
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
figure(2)
plot(qc(start_qc:start_qc + 2000 - 1,1))
hold on
plot(motionTraj.q(:,1))
legend('qc','q')
% 对qs 对齐
q = [];
e1 = 1;
for i = 1:1:CN
    q = qs(i : count_N + i - 1,: );
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
start_qs = index;
figure(3)
plot(qs(start_qs:start_qs + 2000 - 1,1))
hold on
plot(motionTraj.q(:,1))
legend('qs','q')
close all
%% average qc data ans sensor data and current data
% average sensor data
q = [];
q = qc(start_qc: CyclePeriod * CN + start_qc - 1,: );
sum = zeros(CN, 6);
for i = 1:1:CyclePeriod
    st = [CN * (i-1) + 1 ,CN * i];
    ss = q(CN * (i-1) + 1 : CN * i,:);
    sum = sum + ss;
end
q = sum/CyclePeriod;

figure(3)
plot(motionTraj.q(:,1))
hold on  
plot(q(:,1))
max(max(motionTraj.q - q))
 %% average sensor data
sensor_avg = sensordata(start_qc: CyclePeriod * CN + start_qc - 1,: );
sum = zeros(CN, 6);
for i = 1:1:CyclePeriod
    st = [CN * (i-1) + 1 ,CN * i];
    ss = sensor_avg(CN * (i-1) + 1 : CN * i,:);
    sum = sum + ss;
end
sensor_avg = - sum/CyclePeriod; % 反作用力

figure(4)
plot(sensor_avg)
legend('Fx','Fy','Fz','Tx','Ty','Tz')
% 传感器数据滤波
%%
sp = 20;
for i= 1:1:CN-sp
    sum = zeros(4, 6);
    %         sensor_avg_1(i,j) = sensor_avg_1(i,j)*0.5 + sensor_avg_1(i-1,j)*0.5;
    for k = 1:sp
        sum(1,:) = sum(1,:) + sensor_avg(i+k-1,:);
        sum(2,:) = sum(2,:) + motionTraj.q(i+k-1,:);
        sum(3,:) = sum(3,:) + motionTraj.qd(i+k-1,:);
        sum(4,:) = sum(4,:) + motionTraj.qdd(i+k-1,:);
    end
    sensor_avg_2(i,:) = sum(1,:)/sp;
    motionTraj_f.q(i,:) = sum(2,:)/sp;
    motionTraj_f.qd(i,:) = sum(3,:)/sp;
    motionTraj_f.qdd(i,:) = sum(4,:)/sp;
end
figure(5)
plot(sensor_avg_2)
legend('Fx','Fy','Fz','Tx','Ty','Tz')

figure(6)
plot(motionTraj.q(:,1))
hold on
plot(motionTraj_f.q(:,1))




figure(6)
title_name = {'Fx','Fy','Fz','Tx','Ty','Tz'};
for i = 1:1:6
    subplot(2,3,i);
    plot(sensor_avg_2(:,i))
    hold on
    plot(sensor_avg(:,i))
    title(title_name{i})
    xlabel('Time')
    ylabel('Torque(Nm)')
    hold off
end
legend('sensor_avg_2','Sensor')

%% 传感器偏执 G + T_off = F
% G = CalG([0,0,0,0,0,0]') %%%
motionPara.q = zeros(6,1);
motionPara.qd = zeros(6,1);
motionPara.qdd = zeros(6,1);
G = identificationModel(motionPara,'External',FKLink(motionPara.q,1))

S_offset = sensordata(1,:) + G';

for i = 1: size(sensor_avg_2,1)
    sensor_avg_3(i,:) = sensor_avg_2(i,:) + S_offset;
end

%% 1
W = [];
[index, MDP] = GetRobotIdyMimParaSet('External');
for i = 1:1:size(motionTraj_f.q,1)
    motionPara.q = motionTraj_f.q(i,:);
    motionPara.qd = motionTraj_f.qd(i,:);
    motionPara.qdd = motionTraj_f.qdd(i,:);
    HH = IdentificationMatrix(motionPara,'External');
    W = [W;HH];
    wrench(i,:) = (HH(:,index) * MDP)';
end


figure(5)
plot(wrench)
legend('Fx','Fy','Fz','Tx','Ty','Tz')

figure(6)
title_name = {'Fx','Fy','Fz','Tx','Ty','Tz'};
for i = 1:1:6
    subplot(2,3,i);
    plot(wrench(:,i))
    hold on
    plot(sensor_avg_2(:,i))
    title(title_name{i})
    xlabel('Time')
    ylabel('Torque(Nm)')
    hold off
end
legend('CAD','Sensor')
%% ======================esimate ========================

figure(7)
title_name = {'Fx','Fy','Fz','Tx','Ty','Tz'};
for i = 1:1:6
    subplot(2,3,i);
    plot(wrench(:,i))
    hold on
    plot(sensor_avg_3(:,i))
    title(title_name{i})
    xlabel('Time')
    ylabel('Torque(Nm)')
    hold off
end
legend('CAD','Sensor')
























