function qqInitial = TrajInitialPara()
% 优化轨迹的参数初始值计算； 根据位置约束计算；
% syms q01 q02 q03 q04 q05 q06 real
% syms a11 a12 a13 a14 a15 a21 a22 a23 a24 a25 a31 a32 a33 a34 a35 a41 a42 a43 a44 a45 a51 a52 a53 a54 a55 a61 a62 a63 a64 a65 real
% syms b11 b12 b13 b14 b15 b21 b22 b23 b24 b25 b31 b32 b33 b34 b35 b41 b42 b43 b44 b45 b51 b52 b53 b54 b55 b61 b62 b63 b64 b65 real
%
% q0 = [q01 q02 q03 q04 q05 q06];
% ak = [a11 a12 a13 a14 a15;a21 a22 a23 a24 a25;a31 a32 a33 a34 a35;a41 a42 a43 a44 a45;a51 a52 a53 a54 a55;a61 a62 a63 a64 a65];
% bk = [b11 b12 b13 b14 b15;b21 b22 b23 b24 b25;b31 b32 b33 b34 b35;b41 b42 b43 b44 b45;b51 b52 b53 b54 b55;b61 b62 b63 b64 b65];
% qq = [q01 q02 q03 q04 q05 q06 a11 a12 a13 a14 a15 a21 a22 a23 a24 a25 a31 a32 a33 a34 a35 a41 a42 a43 a44 a45 a51 a52 a53 a54 a55 a61 a62 a63 a64 a65...
%       b11 b12 b13 b14 b15 b21 b22 b23 b24 b25 b31 b32 b33 b34 b35 b41 b42 b43 b44 b45 b51 b52 b53 b54 b55 b61 b62 b63 b64 b65];
global Robot Traj

ARM_DOF = Robot.DOF;
OrderNumber = Traj.OrderNumber;
waveFrequency = Traj.waveFrequency;

TrajectoryPeriod = Traj.TrajectoryPeriod;
sampleRate = Traj.sampleRate;

CN = TrajectoryPeriod * sampleRate;% 每个周期内的采样点个数；
dt = linspace(0,TrajectoryPeriod,CN);
qM = Robot.Limit.sensorP; %maxmun of q qd qdd

A = zeros(CN,2*OrderNumber+1);
for i = 1 : CN
    for k = 1 : OrderNumber
        A(i,2*k-1:2*k) = [sin(k * waveFrequency * dt(i)), cos(k * waveFrequency * dt(i))];
        if k == OrderNumber
            A(i,end) = 1;
        end
    end
end

for i = 1:1:ARM_DOF
    q = -qM(i) + 2*qM(i).*rand([CN,1]);
    dd = (inv(A'*A)*A'*q)';
    a(1,5*(i-1)+1:5*i) = dd(1:2:9);
    b(1,5*(i-1)+1:5*i) = dd(2:2:10);
    c(1,i) = dd(11);
end
qqInitial = [c,a,b];
end