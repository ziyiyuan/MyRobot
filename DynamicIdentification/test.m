clc; clear all; close all
format short
addpath( genpath( '..\MyRobot' ) );
%% initialize
delete('Robot.mat');
robotType = 'I5';
Robot = ParaCAD(robotType);
%%
% identificationModel = 'Internal';
identificationModel = 'External';
%% CAD PARA
J = Robot.Para.DP.J;
MS = Robot.Para.DP.MS;
M = Robot.Para.DP.M;
para_cad = Robot.Para_cad;

%% load sensor data
load optTraPara0828;
sensorWrench=load('sensorWrench0828.TXT');
start_idx=147;periodNum=40;seq_len=size(optTraPara.pSeq,1);
%-----------------------------------------------------------
% averaged sensor wrench
%-----------------------------------------------------------
Wr = sensorWrench(start_idx:start_idx+seq_len*periodNum-1,:);
Wavg = zeros(seq_len,6);
for i=1:periodNum
    Wavg = Wavg+Wr((i-1)*seq_len+1:i*seq_len,:);
end
Wavg = Wavg/periodNum;
Wavg(:,3)=-Wavg(:,3);
Wavg(:,6)=-Wavg(:,6);
%% identification matrix
T_cad = [];
regression = [];
for i = 1:1:seq_len
    motionPara.q = optTraPara.pSeq(i,:)';
    motionPara.qd = optTraPara.vSeq(i,:)';
    motionPara.qdd = optTraPara.aSeq(i,:)';
    
    HH = IdentificationMatrix(motionPara,identificationModel);
    regression = [regression;HH];
end

[Col,beta] = getMiniPara(regression);
W1 = regression(:,Col.i);

%% ·ûºÅ±í´ïÊ½
XX = sym('XX', [1 6],'real');
XY = sym('XY', [1 6],'real');
XZ = sym('XZ', [1 6],'real');
YY = sym('YY', [1 6],'real');
YZ = sym('YZ', [1 6],'real');
ZZ = sym('ZZ', [1 6],'real');
MX = sym('MX', [1 6],'real');
MY = sym('MY', [1 6],'real');
MZ = sym('MZ', [1 6],'real');
M = sym('M', [1 6],'real') ;
Sp = [];
for i = 1:1:6
    Sp = [Sp;[XX(i),XY(i),XZ(i),YY(i),YZ(i),ZZ(i),MX(i),MY(i),MZ(i),M(i)]'];
end
X1 = Sp(Col.i);X2 = Sp(Col.c);
expersion = vpa(X1 + beta * X2);
%% cal para
ft = [];
for i = 1:2000
ft = [ft;Wavg(i,:)'];
end
para_est = inv(W1'*W1) * W1'*ft;
para_cad_ = para_cad(Col.i) + beta * para_cad(Col.c);
res = [para_est, para_cad_];

%% revise
ft_est = [];
for i = 1: seq_len
    T = W1(6*(i-1) + 1: 6*i,:) * para_est;
    ft_est = [ft_est;T'];
end
e1 = ft_est - Wavg;
max(e1)
norm(e1)
load('extBaseParaWrench.mat')
e_ft = extBaseParaWrench - ft_est;
max(e_ft)
norm(e_ft)








