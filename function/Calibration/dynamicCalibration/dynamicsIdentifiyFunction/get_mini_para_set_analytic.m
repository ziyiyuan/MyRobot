function [index, MDP] = get_mini_para_set_analytic(Robot, identificationModel)
% MinimalPara: robot identification minimal parameters set based
% on解析解得到最小参数集
% YY6=0; MZ6=0; M6=0; YY5=0; MZ5=0; M5=0; YY4=0; MZ4=0; M4=0; YY3=0; MZ3=0; M3=0; YY2=0; MZ2=0; M2=0; XX1=0; XY1=0; YY1=0;

M1 = Robot.Para.DP.M(1); M2 = Robot.Para.DP.M(2); M3 = Robot.Para.DP.M(3); M4 = Robot.Para.DP.M(4); M5 = Robot.Para.DP.M(5); M6 = Robot.Para.DP.M(6);
MX1 = Robot.Para.DP.MS{1}(1); MY1 = Robot.Para.DP.MS{1}(2); MZ1 = Robot.Para.DP.MS{1}(3);
MX2 = Robot.Para.DP.MS{2}(1); MY2 = Robot.Para.DP.MS{2}(2); MZ2 = Robot.Para.DP.MS{2}(3);
MX3 = Robot.Para.DP.MS{3}(1); MY3 = Robot.Para.DP.MS{3}(2); MZ3 = Robot.Para.DP.MS{3}(3);
MX4 = Robot.Para.DP.MS{4}(1); MY4 = Robot.Para.DP.MS{4}(2); MZ4 = Robot.Para.DP.MS{4}(3);
MX5 = Robot.Para.DP.MS{5}(1); MY5 = Robot.Para.DP.MS{5}(2); MZ5 = Robot.Para.DP.MS{5}(3);
MX6 = Robot.Para.DP.MS{6}(1); MY6 = Robot.Para.DP.MS{6}(2); MZ6 = Robot.Para.DP.MS{6}(3);

XX1 = Robot.Para.DP.J{1}(1,1); XY1 = Robot.Para.DP.J{1}(1,2); XZ1 = Robot.Para.DP.J{1}(1,3); YY1 = Robot.Para.DP.J{1}(2,2); YZ1 = Robot.Para.DP.J{1}(2,3); ZZ1 = Robot.Para.DP.J{1}(3,3);
XX2 = Robot.Para.DP.J{2}(1,1); XY2 = Robot.Para.DP.J{2}(1,2); XZ2 = Robot.Para.DP.J{2}(1,3); YY2 = Robot.Para.DP.J{2}(2,2); YZ2 = Robot.Para.DP.J{2}(2,3); ZZ2 = Robot.Para.DP.J{2}(3,3);
XX3 = Robot.Para.DP.J{3}(1,1); XY3 = Robot.Para.DP.J{3}(1,2); XZ3 = Robot.Para.DP.J{3}(1,3); YY3 = Robot.Para.DP.J{3}(2,2); YZ3 = Robot.Para.DP.J{3}(2,3); ZZ3 = Robot.Para.DP.J{3}(3,3);
XX4 = Robot.Para.DP.J{4}(1,1); XY4 = Robot.Para.DP.J{4}(1,2); XZ4 = Robot.Para.DP.J{4}(1,3); YY4 = Robot.Para.DP.J{4}(2,2); YZ4 = Robot.Para.DP.J{4}(2,3); ZZ4 = Robot.Para.DP.J{4}(3,3);
XX5 = Robot.Para.DP.J{5}(1,1); XY5 = Robot.Para.DP.J{5}(1,2); XZ5 = Robot.Para.DP.J{5}(1,3); YY5 = Robot.Para.DP.J{5}(2,2); YZ5 = Robot.Para.DP.J{5}(2,3); ZZ5 = Robot.Para.DP.J{5}(3,3);
XX6 = Robot.Para.DP.J{6}(1,1); XY6 = Robot.Para.DP.J{6}(1,2); XZ6 = Robot.Para.DP.J{6}(1,3); YY6 = Robot.Para.DP.J{6}(2,2); YZ6 = Robot.Para.DP.J{6}(2,3); ZZ6 = Robot.Para.DP.J{6}(3,3);

a2 = Robot.Para.KP.a(3); a3 = Robot.Para.KP.a(4);
d1 = Robot.Para.KP.d(1); d2 = Robot.Para.KP.d(2); d5 = Robot.Para.KP.d(5); d6 = Robot.Para.KP.d(6);

if strcmp(identificationModel,'Internal')
    IA1 = Robot.Para.TP.IA(1);
    IA2 = Robot.Para.TP.IA(2);
else
    IA1 = 0;
    IA2 = 0;
end

XX6 = XX6 - YY6;
LamMS136 = 2*d6;
LamM16 = d6^2;
XX5 = LamM16*M6 + LamMS136*MZ6 + XX5 - YY5 + YY6;
ZZ5 = LamM16*M6 + LamMS136*MZ6 + YY6 + ZZ5;
MY5 = -M6*d6 + MY5 - MZ6;
MR5 = M5 + M6;

LamM15 = d5^2;
LamMS135 = 2*d5;
XX4 = LamM15*MR5 + LamMS135*MZ5 + XX4 - YY4 + YY5;
ZZ4 = LamM15*MR5 + LamMS135*MZ5 + YY5 + ZZ4;
MY4 = MR5*d5 + MY4 + MZ5;
MR4 = M4 + MR5;

LamM44 = a3^2;
YYR3 = LamM44*MR4 + YY3 + YY4;
XX3 = XX3 + YY4 - YYR3;
XZ3 = MZ4*a3 + XZ3;
ZZ3 = LamM44*MR4 + ZZ3;
MX3 = MR4*a3 + MX3;
MZ3 = MZ3 - MZ4;
MR3 = M3 + MR4;

LamM43 = a2^2;
YYR2 = LamM43*MR3 + YY2 + YYR3;
XX2 = XX2 - YYR2 + YYR3;
XZ2 = MZ3*a2 + XZ2;
ZZ2 = IA2 + LamM43*MR3 + ZZ2;
MX2 = MR3*a2 + MX2;
MZ2 = MZ2 - MZ3;
MR2 = M2 + MR3;

LamMS132 = 2*d2;
LamM12 = d2^2;
ZZ1 = IA1 + LamM12*MR2 + LamMS132*MZ2 + YYR2 + ZZ1;
MY1 = MR2*d2 + MY1 + MZ2;
M1 = M1 + MR2;

index = 1:1:60;
if strcmp(identificationModel,'Internal')
    MDP(1)  = ZZ1;  MDP(2)  = MX1;  MDP(3)  = MY1;                                                               %MZ1 = 0;
    MDP(4)  = XX2;  MDP(5)  = XY2;  MDP(6)  = XZ2; MDP(7)  = YZ2; MDP(8)  = ZZ2;  MDP(9)  = MX2; MDP(10) = MY2;  %MZ2 = 0;
    MDP(11) = XX3;  MDP(12) = XY3;  MDP(13) = XZ3; MDP(14) = YZ3; MDP(15) = ZZ3;  MDP(16) = MX3; MDP(17) = MY3;  %MZ3 = 0;
    MDP(18) = XX4;  MDP(19) = XY4;  MDP(20) = XZ4; MDP(21) = YZ4; MDP(22) = ZZ4;  MDP(23) = MX4; MDP(24) = MY4;  %MZ4 = 0;
    MDP(25) = XX5;  MDP(26) = XY5;  MDP(27) = XZ5; MDP(28) = YZ5; MDP(29) = ZZ5;  MDP(30) = MX5; MDP(31) = MY5;  %MZ5 = 0;
    MDP(32) = XX6;  MDP(33) = XY6;  MDP(34) = XZ6; MDP(35) = YZ6; MDP(36) = ZZ6;  MDP(37) = MX6; MDP(38) = MY6;  %MZ6 = 0;
    %[0 0 0 0 0 ZZ1 MX1 MY1 0 0 ...
    % XX2 XY2 XZ2 0 YZ2 ZZ2 MX2 MY2 0 0
    % XX3 XY3 XZ3 0 YZ3 ZZ3 MX3 MY3 0 0
    % XX4 XY4 XZ4 0 YZ4 ZZ4 MX4 MY4 0 0
    % XX5 XY5 XZ5 0 YZ5 ZZ5 MX5 MY5 0 0
    % XX6 XY6 XZ6 0 YZ6 ZZ6 MX6 MY6 0 0]'
    u_para =  [1 2 3 4 5 9 10 14 19 20 24 29 30 34 39 40 44 49 50 54 59 60];
    
else
    MDP(1)  = XZ1;  MDP(2)  = YZ1;  MDP(3)  = ZZ1; MDP(4)  = MX1;  MDP(5) = MY1;  MDP(6) = MZ1;  MDP(7)  = M1;   %MZ1 = 0;
    MDP(8)  = XX2;  MDP(9)  = XY2;  MDP(10) = XZ2; MDP(11) = YZ2; MDP(12) = ZZ2;  MDP(13) = MX2; MDP(14) = MY2;  %MZ2 = 0;
    MDP(15) = XX3;  MDP(16) = XY3;  MDP(17) = XZ3; MDP(18) = YZ3; MDP(19) = ZZ3;  MDP(20) = MX3; MDP(21) = MY3;  %MZ3 = 0;
    MDP(22) = XX4;  MDP(23) = XY4;  MDP(24) = XZ4; MDP(25) = YZ4; MDP(26) = ZZ4;  MDP(27) = MX4; MDP(28) = MY4;  %MZ4 = 0;
    MDP(29) = XX5;  MDP(30) = XY5;  MDP(31) = XZ5; MDP(32) = YZ5; MDP(33) = ZZ5;  MDP(34) = MX5; MDP(35) = MY5;  %MZ5 = 0;
    MDP(36) = XX6;  MDP(37) = XY6;  MDP(38) = XZ6; MDP(39) = YZ6; MDP(40) = ZZ6;  MDP(41) = MX6; MDP(42) = MY6;  %MZ6 = 0;
    %[0 0 XZ1 0 YZ1 ZZ1 MX1 MY1 MZ1 M1 ...
    % XX2 XY2 XZ2 0 YZ2 ZZ2 MX2 MY2 0 0
    % XX3 XY3 XZ3 0 YZ3 ZZ3 MX3 MY3 0 0
    % XX4 XY4 XZ4 0 YZ4 ZZ4 MX4 MY4 0 0
    % XX5 XY5 XZ5 0 YZ5 ZZ5 MX5 MY5 0 0
    % XX6 XY6 XZ6 0 YZ6 ZZ6 MX6 MY6 0 0]'
    u_para =  [1 2 4 14 19 20 24 29 30 34 39 40 44 49 50 54 59 60];
end
index(u_para) = [];
MDP = MDP';
end

