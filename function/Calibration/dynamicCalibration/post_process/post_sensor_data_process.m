function postData = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate)
%
% clc
% clear all
% robotType = 'I5';
% Robot = get_cad_model_para(robotType);
% Traj = set_excitation_traj_feature();
% sampleRate = 200;
% 
% %% load data
% datafile = 'jointStatusRecord_lizy2.txt';
% motionParaCoeff = 'qq_lizy2.mat';
% 
% postData = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate)
%% initiall
data_all = load(datafile);
trajCoeff = load(motionParaCoeff);

qc = data_all(:,1:6)';
qs = data_all(:,7:12)';
Ic = data_all(:,13:18)';
sensordata = data_all(:,19:24)';
motionTraj = cal_motionPara_from_fourier_series(Robot, Traj, trajCoeff.qq, sampleRate);

N_data = size(qc,2);
CN = sampleRate * Traj.TrajectoryPeriod; % 采样频率 * 轨迹周期,单个周期内的采样点
CyclePeriod = 40;
if N_data < CyclePeriod * CN
    CyclePeriod = 20;
end

%% plot row data
figure(1)
plot(qc(1,:),'r')
title('row joint data')
pause(2)
%
N = 3000;
figure(2)
plot(qc(1,1:N),'r')
hold on
plot(motionTraj.q(1,:))
legend('qc','q');
%% 对 qc 和 qs 相对于优化的轨迹进行对齐
close all
P = CyclePeriod - 1;
count_N = P * CN;% 对齐参与计算点的个数；
e1 = 1000;
for i = 1:1:N_data - CyclePeriod * CN
    q = qc(:,i : count_N + i - 1);
    sum = zeros(6,CN);
    for j = 1:1:P
        ss = q(:, CN * (j-1) + 1 : CN * j);
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
plot(qc(1, start_qc:start_qc + 2000 - 1))
hold on
plot(motionTraj.q(1,:))
legend('qc','q')
title('joint data after aligned ')

%% average qc data and sensor data and current data
% average q data
close all
q = qc(:,start_qc: CyclePeriod * CN + start_qc - 1);
sum = zeros(6,CN);
for i = 1:1:CyclePeriod
    st = [CN * (i-1) + 1 ,CN * i];
    ss = q(:,CN * (i-1) + 1 : CN * i);
    sum = sum + ss;
end
q = sum/CyclePeriod;

figure(1)
plot(motionTraj.q(1,:))
hold on
plot(q(1,:))
legend('qs','qc');
title('joint data after averaged ')
pause(2)

% average sensor data
sensor_avg = sensordata(:,start_qc: CyclePeriod * CN + start_qc - 1);
sum = zeros(6,CN);
for i = 1:1:CyclePeriod
    ss = sensor_avg(:, CN * (i-1) + 1 : CN * i);
    sum = sum + ss;
end
sensor_avg = - sum/CyclePeriod; % 反作用力

figure(2)
plot(sensor_avg')
legend('Fx','Fy','Fz','Tx','Ty','Tz')
title('sensor data after averaged ')
pause(2)

% average current data
current_avg = Ic(:,start_qc: CyclePeriod * CN + start_qc - 1);
sum = zeros(6,CN);
for i = 1:1:CyclePeriod
    ss = current_avg(:, CN * (i-1) + 1 : CN * i);
    sum = sum + ss;
end
current_avg = sum/CyclePeriod;

figure(3)
plot(current_avg')
legend('I1','I2','I3','I4','I5','I6')
title('current data after averaged ')
pause(2)

%% 传感器数据滤波，关节角,电流， 采用同样的滤波方式，保持相位一致
close all
fc = 10 * Traj.OrderNumber * Traj.frequency;  % 截止频率
fs = sampleRate; % 采样频率
order = 6;
[b,a] = butter(order,fc/(fs/2));
freqz(b,a)
sensor_avg_f = filter(b,a,sensor_avg')';
current_avg_f = filter(b,a,current_avg')';
motionTraj_f.q = filter(b,a,motionTraj.q')';
motionTraj_f.qd = filter(b,a,motionTraj.qd')';
motionTraj_f.qdd = filter(b,a,motionTraj.qdd')';

figure(1)
plot(sensor_avg_f')
legend('Fx','Fy','Fz','Tx','Ty','Tz')
title('filtered sensor data')

figure(2)
title('compara sensor data, filtered before and after')
for i = 1:1:6
    subplot(2,3,i);
    plot(sensor_avg(i,:))
    hold on
    plot(sensor_avg_f(i,:))
    hold off
end
pause(2)

figure(3)
plot(current_avg_f')
legend('I1','I2','I3','I4','I5','I6')
title('filtered current data')

figure(4)
title('compara current data, filtered before and after')
for i = 1:1:6
    subplot(2,3,i);
    plot(current_avg(i,:))
    hold on
    plot(current_avg_f(i,:))
    hold off
end
pause(2)

figure(5)
plot(motionTraj.q(1,:))
hold on
plot(motionTraj_f.q(1,:))
title('compara joint data, filtered before and after') % 观察相位差；

% 去掉第一个周期失真数据
sensor_avg_s = sensor_avg_f(:,fs+1:end);
current_avg_s = current_avg_f(:,fs+1:end);
motionTraj_s.q = motionTraj_f.q(:,fs+1:end);
motionTraj_s.qd = motionTraj_f.qd(:,fs+1:end);
motionTraj_s.qdd = motionTraj_f.qdd(:,fs+1:end);

%% 传感器偏置 -sensordata + T_off = G
% G = cal_G(Robot,[0,0,0,0,0,0]')%%% 奇异时算不了
motionPara.q = zeros(6,1);
motionPara.qd = zeros(6,1);
motionPara.qdd = zeros(6,1);
G = get_wrench_from_diff_identificationModel(Robot, motionPara, 'External');
S_offset = sensordata(:,1) + G;

for i = 1: size(sensor_avg_s,2)% 反向之后
    sensor_avg_s_minus_offset(:,i) = sensor_avg_s(:,i) + S_offset;
end

%% CAD 数据力矩曲线和测量力矩对比
close all
for i = 1:1:size(sensor_avg_s,2)
    motionPara.q = motionTraj_s.q(:,i);
    motionPara.qd = motionTraj_s.qd(:,i);
    motionPara.qdd = motionTraj_s.qdd(:,i);
    wrench(:,i) = get_wrench_from_diff_identificationModel(Robot, motionPara, 'External');
    torque(:,i) = get_wrench_from_diff_identificationModel(Robot, motionPara, 'Internal') .* Robot.Para.TC;
end

figure(1)
title_name = {'Fx','Fy','Fz','Tx','Ty','Tz'};
for i = 1:1:6
    subplot(2,3,i);
    plot(wrench(i,:))
    hold on
    plot(sensor_avg_s(i,:))
    title(title_name{i})
    xlabel('Time')
    ylabel('Wrench(Nm)')
    hold off
end
legend('CAD','Sensor')
pause(5)

figure(2)
title_name = {'I1','I2','I3','I4','I5','I6'};
for i = 1:1:6
    subplot(2,3,i);
    plot(torque(i,:))
    hold on
    plot(current_avg_s(i,:))
    title(title_name{i})
    xlabel('Time')
    ylabel('Torque(Nm)')
    hold off
end
legend('CAD','Current')
pause(5)

postData.motionTraj = motionTraj_s;
postData.sensorData = sensor_avg_s;
postData.currentData = current_avg_s;
postData.sensorOffSet = S_offset;
close all
end