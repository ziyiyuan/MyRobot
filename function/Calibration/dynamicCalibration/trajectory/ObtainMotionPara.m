function motionPara = ObtainMotionPara(qq, sampleRate)
% ���� 5 �� ����Ҷ������ϵ�� 6 *��1 + 5 * 2�� ��,�Լ�ÿ�����ڲ������������
% ���һ�����������ڸ����λ���ٶȼ��ٶȣ�T * sampleRate ���㣻
% obtain the joint angle in one period
% load qq.mat first
% input qq coeff
global Robot Traj

ARM_DOF = Robot.DOF;

OrderNumber = Traj.OrderNumber;
waveFrequency = Traj.waveFrequency;
TrajectoryPeriod = Traj.TrajectoryPeriod;

% sampleRate = Traj.sampleRate;

CN = TrajectoryPeriod * sampleRate;% ÿ�������ڵĲ����������
dt = linspace(0,TrajectoryPeriod,CN);
qt = zeros(ARM_DOF,CN); qtd = zeros(ARM_DOF,CN); qtdd = zeros(ARM_DOF,CN);

for i = 1 : CN
    sumq = zeros(ARM_DOF,1); sumqd = zeros(ARM_DOF,1); sumqdd = zeros(ARM_DOF,1);
    for k = 1 : OrderNumber
        %         Traj.sk(i,k) = sin(k * waveFrequency * dt(i));
        %         Traj.ck(i,k) = cos(k * waveFrequency * dt(i));
        sk = sin(k * waveFrequency * dt(i));
        ck = cos(k * waveFrequency * dt(i));
        for j = 1:ARM_DOF
            sumq(j) = sumq(j) + qq(5 * j + k + 1) * sk + qq(5 * j + k + 31) * ck;
            sumqd(j) = sumqd(j) + qq(5 * j + k + 1) * k * waveFrequency * ck - qq(5 * j + k + 31) * k * waveFrequency * sk;
            sumqdd(j) = sumqdd(j) - qq(5 * j + k + 1) * k^2 * waveFrequency^2 * sk - qq(5 * j + k + 31) * k^2 * waveFrequency^2 * ck;
        end
    end
    
    for j = 1:ARM_DOF
        qt(j,i) = qq(j) + sumq(j);
        qtd(j,i) = sumqd(j);
        qtdd(j,i) = sumqdd(j);
    end
    
end

motionPara.q = qt';
motionPara.qd = qtd';
motionPara.qdd = qtdd';
end