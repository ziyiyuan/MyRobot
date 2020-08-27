% function T0_6 = FK(q)
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
a2 = 0.390;
d1 = 0.181; d2 = 0.009; d4 = 0.2945; d5 = 0.105; d6 = 0.1;

DH = [0,0,d1,0;
    pi/2, 0 ,d2, pi/2;
    pi, a2, 0, pi/2;
    pi/2,0,  d4, 0;
    pi/2, 0, d5, 0;
    -pi/2,0,  d6, 0];

q = [0,0,0,0,0,0]';

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
T0_6=T0_5*T5_6

for i = 1:1:6
S(i) = Link([ 0, DH(i,3), DH(i,2), DH(i,1), 0], 'modified');
end
aubo_i= SerialLink(S, 'name', 'AUBO_i');
aubo_i.offset =  DH(:,4)';
qz = [0, 0, 0, 0, 0, 0]; % ¡„Œª
aubo_i.plot(qz); 
B = aubo_i.fkine(qz);

function T = transfer(alpha, a, d, theta)
ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
T = [ct,   -st,   0,    a;
    st*ca, ct*ca, -sa,  -sa*d;
    st*sa, ct*sa, ca,   ca*d;
    0,  0,  0,  1];
end