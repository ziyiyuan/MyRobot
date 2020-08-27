clc
clear all
syms a2 a3 d1 d2 d5 d6 p_offset
q = sym('q', [1,4]);
global Robot 


DH = [Robot.Para.KP.alpha(1)  0	 d1	Robot.Para.KP.beta(1);
    Robot.Para.KP.alpha(2)	 0	 d2	Robot.Para.KP.beta(2);
    Robot.Para.KP.alpha(3)   a2	 0	Robot.Para.KP.beta(3);
    Robot.Para.KP.alpha(4)   a3	 0	Robot.Para.KP.beta(4);
    Robot.Para.KP.alpha(5)	 0	 d5	Robot.Para.KP.beta(5);
    Robot.Para.KP.alpha(6)	 0	 d6	Robot.Para.KP.beta(6)];

T0_1 = transfer(DH(1,1),DH(1,2),DH(1,3),DH(1,4) + q(1));
T1_2 = transfer(DH(2,1),DH(2,2),DH(2,3),DH(2,4) + q(2));
T2_3 = transfer(DH(3,1),DH(3,2),DH(3,3),DH(3,4) + q(3));
T3_4 = transfer(DH(4,1),DH(4,2),DH(4,3),DH(4,4) + q(4));
T4_e = [eye(3),[0;-p_offset;0];[0,0,0,1]];
T0_4 = T0_1 * T1_2 * T2_3 * T3_4 * T4_e;
T4 = simplify(T0_4);
for i = 1:1:4
    for j = 1:1:4
        T(j + 4*(i-1),1) = T4(i,j);
    end
end













