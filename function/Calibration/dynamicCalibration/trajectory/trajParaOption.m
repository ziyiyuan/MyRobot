function trajParaOption()
% ���ö���ѧ������ʶ�켣�Ĳ���
global Traj
Traj.OrderNumber = 5;  % 5�θ���Ҷ����ʽ
Traj.frequency = 0.1; %��Ƶ f = 0.1 Hz
Traj.sampleRate = 200;% ����Ƶ�� 200 Hz; 0.005s ��һ���㣬����ϵͳƵ��ȷ����
Traj.optimal_sample = 8; %Լ���켣����Ƶ�ʣ�
Traj.CyclePeriod = 40; % %40 �����ڣ�

Traj.waveFrequency = Traj.frequency * 2 * pi; %ԲƵ�� w = 2 *pi *f
Traj.TrajectoryPeriod = 1 / Traj.frequency; % ���� T = 1/f;
end
