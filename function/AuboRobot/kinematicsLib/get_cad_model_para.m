function Robot = get_cad_model_para(robotType)
% get_cad_model_para �õ���е��CADģ�͵Ĳ���
% ���������
%   robotType: ��е���ͺţ���I3��,'I5'...
% ���������
%   Robot�������ṹ��
% ����˵����
%   Robot = get_cad_model_para('I3') :��� I3 ��ģ�Ͳ�����
%   Robot = get_cad_model_para() :��� I5 ��ģ�Ͳ�����

% �汾��V1.0����д��2020/8/27���޸���2020/8/27�����ߣ�ziyi

if nargin > 1
    error('����������࣡');
elseif nargin == 0
    robotType = 'I5'; % Ĭ�������ΪI5
end

gx = 0; gy = 0; gz = -9.81;% default

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
    % COMMON
    a2 = 0.408 ; a3 = 0.376; d1 = 0.0985; d2 = 0.1215;d5 = 0.1025; d6 = 0.094;
    Mt = 0;
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
    I{6} = [ 0.0139395,  0,  0; 0,     0.0139882, 0; 0, 0,     0.0224092];
    IA1 = 0;    IA2 = 0;  IA3 = 1.1;    IA4 = 1.1;  IA5 = 1.1;    IA6 = 1.1;
    TC1 = 110; TC2 = 110; TC3 = 110; TC4 = 135; TC5 = 135; TC6 = 135;
    
    %     a2 = 0.408 ; a3 = 0.376; d1 = 0.122;d2 = 0.1215;d5 = 0.1025; d6 = 0.094;
    %     Mt = 5;
    %     tcx = 0.0;         tcy = 0;          tcz = 0;
    %     M1 = 5.05;      CX1 = 0;          CY1 = 0.0038;      CZ1 = 0.0006;
    %     M2 = 11.28 ;    CX2 = 0.2040;     CY2 = 0;           CZ2 = 0.0196;
    %     M3 = 2.88 ;     CX3 = 0.2699;     CY3 = 0;           CZ3 = 0.0992;
    %     M4 = 1.62;      CX4 = 0;          CY4 = -0.0997;     CZ4 = -0.0780;
    %     M5 = 1.62;      CX5 = 0;          CY5 = -0.0028;     CZ5 = -0.0030;
    %     M6 = 0.5 + Mt;  CX6 = tcx;        CY6 = tcy;         CZ6 = ((-d6+0.025)*(1.86-1.54) + Mt*tcz) / M6;
    %
    %     I{6} = [[]];
    %     I{1} = [ 0.0148, 0,  0;  0,    0.0143, 0; 0, 0,   0.0095];
    %     I{2} = [0.0305, 0, 0; 0, 0.4505, 0; 0,   0,     0.4400];
    %     I{3} = [0.0033, 0, 0;  0,   0.0575,0; 0,    0,   0.0565];
    %     I{4} = [ 0.0023, 0,  0; 0,  0.0013, 0; 0, 0,  0.0022];
    %     I{5} = [ 0.0023, 0,   0;   0,  0.0013, 0;  0, 0,  0.0022];
    %     I{6} = [ 0.00139395,  0,  0; 0,     0.00139882, -0.00000211765; 0, -0.00000211765,     0.00224092];
    %     IA1 = 0;    IA2 = 0;  IA3 = 1.1;    IA4 = 1.1;  IA5 = 1.1;    IA6 = 1.1;
    %     TC1 = 110; TC2 = 110; TC3 = 110; TC4 = 135; TC5 = 135; TC6 = 135;
    
    
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
    Ij{i} = I{i} - M(i)* skew(c{i})*skew(c{i}); %%  ��������ϵ�Ĺ�������
end

Robot.DOF = 6;

% kinematics modified DH  para
Robot.Para.KP.a = [0, 0, a2,a3,0, 0]';
Robot.Para.KP.d = [d1,d2,0, 0, d5,d6]';
Robot.Para.KP.alpha = [0,-pi/2,pi,pi,-pi/2,pi/2]';
Robot.Para.KP.beta = [pi,-pi/2,0,-pi/2,0,0]'; % ��λ

% dynamic para
Robot.Para.DP.c = c; % ����
Robot.Para.DP.M = M; % ����
Robot.Para.DP.MS = MS; %����*��������������ϵ������
Robot.Para.DP.Ic = I; %����������ת���Ĺ�����������������ϵ��������
Robot.Para.DP.J = Ij; %�����ƹؽ�����ϵת���Ĺ��������ڹؽ�����ϵ��������

Robot.Para.TC = TC; %������س����� Tau = I / Tc

% totor para  and tool para
Robot.Para.TP.IA = IA; % ���߲�����

% DH MATRIX
Robot.DH = get_DH_para(Robot.Para.KP);

%������
Robot.gravity = [gx;gy;gz];
% all para
for i = 1:Robot.DOF
    P{i} = [Ij{i}(1,1) Ij{i}(1,2) Ij{i}(1,3) Ij{i}(2,2) Ij{i}(2,3) Ij{i}(3,3) MS{i}(1) MS{i}(2) MS{i}(3) M(i)]';
end
Robot.Para_cad = [P{1}; P{2}; P{3}; P{4}; P{5}; P{6}]; % 60 * 1

% �ؽ�λ�ã��ٶȣ����ٶ�����  %% ���ڱ�ʶ��������
Robot.Limit.q = ones(6,1)*175/180*pi;
Robot.Limit.qd = [2.2, 2.2, 2.2, 2.2 ,2.2, 2.2];
Robot.Limit.qdd = [4, 4, 4, 4, 4, 4];
Robot.Limit.torque = [150,150,150,40,40,40];

Robot.Limit.sensor = [1000,1000,1000,150,150,150];% ���������̣�
Robot.Limit.sensorP = [90,90,90,90,90,90]*pi/180;%% ��ֹ����������������

Robot.Para.DP.Mreff = [4.8267, 10.5372, 2.8461, 1.6075, 1.6116, 0.1950]'; % ����
end

function DH = get_DH_para(kinePara)
% get_DH_para �����˶�ѧ���������DH��������
% ���������
%   kinePara:
% ���������
%   DH��
% ����˵����
%   DH = get_DH_para(kinePara)

% �汾��V1.0����д��2020/8/27���޸���2020/8/27�����ߣ�ziyi

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
