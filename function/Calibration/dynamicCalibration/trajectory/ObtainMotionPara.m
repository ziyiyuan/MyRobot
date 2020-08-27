function motionPara = ObtainMotionPara(qq, sampleRate)
% 输入 5 次 傅里叶级数的系数 6 *（1 + 5 * 2） 个,以及每个周期采样点个数；；
% 输出一个周期以内内个点的位置速度加速度；T * sampleRate 个点；
% obtain the joint angle in one period
% load qq.mat first
% input qq coeff
global Robot Traj

ARM_DOF = Robot.DOF;

OrderNumber = Traj.OrderNumber;
waveFrequency = Traj.waveFrequency;
TrajectoryPeriod = Traj.TrajectoryPeriod;

% sampleRate = Traj.sampleRate;

CN = TrajectoryPeriod * sampleRate;% 每个周期内的采样点个数；
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