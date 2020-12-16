clc
clear all
ori_in(1) = 0.000000;
ori_in(2) = 0.000016;
ori_in(3) = 0.707105;
ori_in(4) = -0.707109;
R1 = QuattoRotMatrix(ori_in)
q = RotMatrixtoQuat(R1)
R2 = QuattoRotMatrix(q)
e = norm(R1 -R2)