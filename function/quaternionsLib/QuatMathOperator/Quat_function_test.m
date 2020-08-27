clc
clear all
q1 = [1;2;3;4];
q2 = [4;2;6;9];

%%一元运算符
%模长
norm1 = norm(q1)
%共轭四元素 q*q_star = norm(q)^2 = q_star*q
q1_star = QuatConjugate(q1)
q_star_q = sqrt(QuatLeftMultiplyQuat(q1_star) * q1)
q_q_star = sqrt(QuatLeftMultiplyQuat(q1) * q1_star)
q_q_star_1 = sqrt(quatMult(q1,q1_star) )
%逆 inv(q) = q_star/(norm(q)^2)
q1_inv =  QuatInverse(q1)
q1_1_inv = q1_star/(norm1^2)

%单位化
q1 = QuatUnitize(q1)
q2 = QuatUnitize(q2) 
q1_star = QuatUnitize(q1_star)
%单位化后的逆 q_inv = q_star
q1_inv =  QuatInverse(q1)

%对数 (单位化后)
logq1 = QuatLog(q1)
%指数（单位化后）
explogq1 = QuatExp(logq1)
q1-explogq1
%幂
q_t = QuatExp(QuatLog(q1)*4)
q1_t = QuatPower(q1,4)










