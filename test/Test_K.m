% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
clc
clear
global Robot 
q = [0.6641,    0.3930,   -1.1606,    0.0172,   -1.5708,    0.6641]';
% q = [0,0,0,0,0,0];
T0_6 = FK(q)
T0_n = FKall(q,6)
e{1} = T0_6 - T0_n;

T0_4 = FKall(q,4)

a2 = Robot.Para.KP.a(3);
a3 = Robot.Para.KP.a(4);
d1 = Robot.Para.KP.d(1);
d2 = Robot.Para.KP.d(2);
T4 = TRANS(a2, a3, d1, d2, 0, q);
T4 = [T4(1:4)';T4(5:8)';T4(9:12)';T4(13:16)']
e{2} = T0_4 - T4



q_sols = IK_all(T0_6)
q_sol = IK_one(q_sols, q)
q_sol - q


M = CalM(q)
G = CalG(q)

J = CalJacobian(q,'center')



for i = 1:1:length(e)
    if norm(e{i}) >  1e-10
        error(['cal error' num2str(i)])
    end
end
