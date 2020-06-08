% function   T0_i = FKall(q)
%%!!!!!!!!!!!!
% input dhpara and joint q(6*1 or 1*6),radian
% output flange in base T0_6

% Example Input (Aubo Robot):
% clear; clc;
% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
% q = [0,0,0,0,0,0]';
% T0_6 = FK(q)

% output:
% T0_6 =
% 
%     1.0000   -0.0000   -0.0000   -0.0000
%    -0.0000    0.0000   -1.0000   -0.2155
%     0.0000    1.0000    0.0000    1.0085
%          0         0         0    1.0000
load('Robot.mat');

DH = Robot.DH;
T0_1 = transfer(DH(1,1),DH(1,2),DH(1,3),DH(1,4) + q(1));
T1_2 = transfer(DH(2,1),DH(2,2),DH(2,3),DH(2,4) + q(2));
T2_3 = transfer(DH(3,1),DH(3,2),DH(3,3),DH(3,4) + q(3));
T3_4 = transfer(DH(4,1),DH(4,2),DH(4,3),DH(4,4) + q(4));
T4_5 = transfer(DH(5,1),DH(5,2),DH(5,3),DH(5,4) + q(5));
T5_6 = transfer(DH(6,1),DH(6,2),DH(6,3),DH(6,4) + q(6));

T0_2=T0_1*T1_2;
T0_3=T0_2*T2_3;
T0_4=T0_3*T3_4;
T0_5=T0_4*T4_5;
T0_6=T0_5*T5_6;
end