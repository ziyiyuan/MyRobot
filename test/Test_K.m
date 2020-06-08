% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
clc
clear
q = [0.6641,    0.3930,   -1.1606,    0.0172,   -1.5708,    0.6641]';
% q = [0,0,0,0,0,0];
T0_6 = FK(q)
q_sols = IK_all(T0_6)
q_sol = IK_one(q_sols, q)
q_sol - q


M = CalM(q)
G = CalG(q)

J = CalJacobian(q,'center')




