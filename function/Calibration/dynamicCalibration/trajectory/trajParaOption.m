function trajParaOption()
% 设置动力学参数辨识轨迹的参数
global Traj
Traj.OrderNumber = 5;  % 5次傅里叶级数式
Traj.frequency = 0.1; %基频 f = 0.1 Hz
Traj.sampleRate = 200;% 采样频率 200 Hz; 0.005s 采一个点，根据系统频率确定；
Traj.optimal_sample = 8; %约束轨迹采样频率；
Traj.CyclePeriod = 40; % %40 个周期；

Traj.waveFrequency = Traj.frequency * 2 * pi; %圆频率 w = 2 *pi *f
Traj.TrajectoryPeriod = 1 / Traj.frequency; % 周期 T = 1/f;
end
