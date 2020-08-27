function ParaCAD(robotType)
% robotType: I3 I5 I7 I10 I5L
% KP: kinematics parameters
% DP: dynamics parameters
% TP: tool parameters
global Robot 
if strcmp(robotType,'I3')
    a2 = 0.266; a3 = 0.2565; d1 = 0.157; d2 = 0.119; d5 = 0.1025; d6 = 0.094;
    Mt = 0;            tcx = 0.0;        tcy = 0;           tcz = 0;
    M1 = 2.6*1.5;      CX1 = 0;          CY1 = 0.006;       CZ1 = 0.004;
    M2 = 6.064*1.15;   CX2 = 0.125;      CY2 = 0;           CZ2 = 0.027;
    M3 = 2.504 ;       CX3 = 0.173;      CY3 = 0;           CZ3 = 0.095;
    M4 = 1.62;         CX4 = 0;          CY4 = 0.0025;      CZ4 = 0;
    M5 = 1.62;         CX5 = 0;          CY5 = 0.0025;      CZ5 = 0;
    M6 = 0.40 + Mt;    CX6 = tcx;        CY6 = tcy;         CZ6 = ((-d6+0.025)*(1.86-1.54) + Mt*tcz) / M6;
    
    I{6} = [[]];
    I{1} = [ 0.007,0, 0; 0,0.007,0.0036;0,0.0036,0.004];
    I{2} = [0.011, 0, 0.0036; 0, 0.107, 0;    0.0036,   0,     0.099];
    I{3} = [0.025, 0, 0.041;        0,   0.1,0.002;        0.041,    0.002,   0.076];
    I{4} = [ 0.0031, 0,  0;       0,  0.0012, 0.001;	   0.00, 0.001,  0.0030];
    I{5} = [  0.0031, 0,  0;       0,  0.0012, 0.001;	   0.00, 0.001,  0.0030];
    I{6} = [ 0.0004, 0, 0; 0, 0.0004, 0; 0, 0, 0.0007];
    IA1 = 0;    IA2 = 0;  IA3 = 1.1;    IA4 = 1.1;  IA5 = 1.1;    IA6 = 1.1;
    
    TC1 = 96; TC2 = 96; TC3 = 110; TC4 = 135; TC5 = 135; TC6 = 135;
elseif strcmp(robotType,'I5')
    a2 = 0.408 ; a3 = 0.376; d1 = 0.0985; d2 = 0.1215;d5 = 0.1025; d6 = 0.094;
    Mt = 5;
    tcx = 0.0;         tcy = 0;          tcz = 0;
    M1 = 5.05;      CX1 = 0;          CY1 = 0.0038;      CZ1 = 0.0006;
    M2 = 11.28 ;    CX2 = 0.2040;     CY2 = 0;           CZ2 = 0.0196;
    M3 = 2.88 ;     CX3 = 0.2699;     CY3 = 0;           CZ3 = 0.0992;
    M4 = 1.62;      CX4 = 0;          CY4 = -0.0997;     CZ4 = -0.0780;
    M5 = 1.62;      CX5 = 0;          CY5 = -0.0028;     CZ5 = -0.0030;
    M6 = 0.5 + Mt;  CX6 = tcx;        CY6 = tcy;         CZ6 = ((-d6+0.025)*(1.86-1.54) + Mt*tcz) / M6;
    
    I{6} = [[]];
    I{1} = [ 0.0148, 0,  0;  0,    0.0143, 0; 0, 0,   0.0095];
    I{2} = [0.0305, 0, 0; 0, 0.4505, 0; 0,   0,     0.4400];
    I{3} = [0.0033, 0, 0;  0,   0.0575,0; 0,    0,   0.0565];
    I{4} = [ 0.0023, 0,  0; 0,  0.0013, 0; 0, 0,  0.0022];
    I{5} = [ 0.0023, 0,   0;   0,  0.0013, 0;  0, 0,  0.0022];
    I{6} = [ 0.00139395,  0,  0; 0,     0.00139882, -0.00000211765; 0, -0.00000211765,     0.00224092];
    IA1 = 0;    IA2 = 0;  IA3 = 1.1;    IA4 = 1.1;  IA5 = 1.1;    IA6 = 1.1;
    TC1 = 110; TC2 = 110; TC3 = 110; TC4 = 135; TC5 = 135; TC6 = 135;
end

IA = [IA1, IA2, IA3, IA4, IA5, IA6]';
TC = [TC1, TC2, TC3, TC4, TC5, TC6]';
M = [M1,M2,M3,M4,M5,M6]';
c{6} = [[]];
MS{6} = [[]];
Ij{6} = [[]];
c{1} = [CX1,CY1,CZ1]';
c{2} = [CX2,CY2,CZ2]';
c{3} = [CX3,CY3,CZ3]';
c{4} = [CX4,CY4,CZ4]';
c{5} = [CX5,CY5,CZ5]';
c{6} = [CX6,CY6,CZ6]';

for i = 1:6
    MS{i} = M(i) * c{i};
    Ij{i} = I{i} - M(i)* skew(c{i})*skew(c{i}); %%  连杆坐标系的惯性张量
end

Robot.DOF = 6;

% kinematics modified DH  para
Robot.Para.KP.a = [0, 0, a2,a3,0, 0]';
Robot.Para.KP.d = [d1,d2,0, 0, d5,d6]';
Robot.Para.KP.alpha = [0,-pi/2,pi,pi,-pi/2,pi/2]';
Robot.Para.KP.beta = [pi,-pi/2,0,-pi/2,0,0]'; % 零位

% dynamic para
Robot.Para.DP.c = c; % 质心
Robot.Para.DP.M = M; % 质量
Robot.Para.DP.MS = MS; %质量*质心在连杆坐标系的描述
Robot.Para.DP.Ic = I; %刚体绕质心转动的惯性张量在质心坐标系的描述；
Robot.Para.DP.J = Ij; %刚体绕关节坐标系转动的惯性张量在关节坐标系的描述；

Robot.Para.TC = TC; %电机力矩常数； Tau = I / Tc

% totor para  and tool para
Robot.Para.TP.IA = IA; % 工具参数？

% DH MATRIX
Robot.DH = ParaDH(Robot.Para.KP)

%重力项
gx = 0; gy = 0; gz = -9.81;% default
Robot.gravity = [gx;gy;gz];

% all para
for i = 1:Robot.DOF
    P{i} = [Ij{i}(1,1) Ij{i}(1,2) Ij{i}(1,3) Ij{i}(2,2) Ij{i}(2,3) Ij{i}(3,3) MS{i}(1) MS{i}(2) MS{i}(3) M(i)]';
end
Robot.Para_cad = [P{1}; P{2}; P{3}; P{4}; P{5}; P{6}]; % 60 * 1

% 关节位置，速度，加速度限制  %% 用于辨识，更保守
Robot.Limit.q = [3,3,3,3,3,3];
Robot.Limit.qd = [2.2, 2.2, 2.2, 2.2 ,2.2, 2.2];
Robot.Limit.qdd = [4, 4, 4, 4, 4, 4];
Robot.Limit.torque = [150,150,150,40,40,40];

Robot.Limit.sensor = [1000,1000,1000,150,150,150];% 传感器量程；
Robot.Limit.sensorP = [90,90,90,90,90,90]*pi/180;%% 防止碰到基座传感器；
end

function DH = ParaDH(kinePara)
% input all kinematics para structure,
% output DH matrix

% Example Input (Aubo Robot):
% clear; clc;
% robotType = 'I5';
% robot = ParaCAD(robotType);
% DH = ParaDH(robot.Para.KP)
%
% output:
% DH =
%          0         0    0.1220    3.1416
%    -1.5708         0    0.1215   -1.5708
%     3.1416    0.4080         0         0
%     3.1416    0.3760         0   -1.5708
%    -1.5708         0    0.1025         0
%     1.5708         0    0.0940         0

a2 = kinePara.a(3);
a3 = kinePara.a(4);
d1 = kinePara.d(1);
d2 = kinePara.d(2);
d5 = kinePara.d(5);
d6 = kinePara.d(6);

DH = [kinePara.alpha(1)  0	 d1	kinePara.beta(1);
    kinePara.alpha(2)	 0	 d2	kinePara.beta(2);
    kinePara.alpha(3)   a2	 0	kinePara.beta(3);
    kinePara.alpha(4)   a3	 0	kinePara.beta(4);
    kinePara.alpha(5)	 0	 d5	kinePara.beta(5);
    kinePara.alpha(6)	 0	 d6	kinePara.beta(6)];
end

