
clear; clc;

d1 = 0.181; d2 = 0.009; d4 = 0.2945; d5 = 0.105; d6 = 0.1;
a2 = 0.390; a6 = d5;

DH = [0,0,d1,0;
    pi/2, 0 ,d2, pi/2;
    pi, a2, 0, pi/2;
    pi/2,0,  d4, 0;
    pi/2, 0, 0, 0;
    -pi/2,0,  d6, -pi/2
    0, a6, 0, 0];

q = [0,0,0,0,0,0]';

T0_1 = transfer(DH(1,1),DH(1,2),DH(1,3),DH(1,4) + q(1));
T1_2 = transfer(DH(2,1),DH(2,2),DH(2,3),DH(2,4) + q(2));
T2_3 = transfer(DH(3,1),DH(3,2),DH(3,3),DH(3,4) + q(3));
T3_4 = transfer(DH(4,1),DH(4,2),DH(4,3),DH(4,4) + q(4));
T4_5 = transfer(DH(5,1),DH(5,2),DH(5,3),DH(5,4) + q(5));
T5_6s = transfer(DH(6,1),DH(6,2),DH(6,3),DH(6,4));
T6s_6 = transfer(DH(7,1),DH(7,2),DH(7,3),DH(7,4) + q(6));

T0_2=T0_1*T1_2;
T0_3=T0_2*T2_3;
T0_4=T0_3*T3_4;
T0_5=T0_4*T4_5;
T0_6s=T0_5*T5_6s;
T0_6 = T0_6s*T6s_6

for i = 1:1:7
S(i) = Link([ 0, DH(i,3), DH(i,2), DH(i,1), 0], 'modified');
end
aubo_i= SerialLink(S, 'name', 'AUBO_i');
aubo_i.offset =  DH(:,4)';
qz = [0, 0, 0, 0, 0, 0,0]; % ¡„Œª
aubo_i.plot(qz); 
B = aubo_i.fkine(qz);
