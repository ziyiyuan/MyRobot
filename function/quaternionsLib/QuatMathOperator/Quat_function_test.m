clc
clear all
q1 = [1;2;3;4];
q2 = [4;2;6;9];

%%һԪ�����
%ģ��
norm1 = norm(q1)
%������Ԫ�� q*q_star = norm(q)^2 = q_star*q
q1_star = QuatConjugate(q1)
q_star_q = sqrt(QuatLeftMultiplyQuat(q1_star) * q1)
q_q_star = sqrt(QuatLeftMultiplyQuat(q1) * q1_star)
q_q_star_1 = sqrt(quatMult(q1,q1_star) )
%�� inv(q) = q_star/(norm(q)^2)
q1_inv =  QuatInverse(q1)
q1_1_inv = q1_star/(norm1^2)

%��λ��
q1 = QuatUnitize(q1)
q2 = QuatUnitize(q2) 
q1_star = QuatUnitize(q1_star)
%��λ������� q_inv = q_star
q1_inv =  QuatInverse(q1)

%���� (��λ����)
logq1 = QuatLog(q1)
%ָ������λ����
explogq1 = QuatExp(logq1)
q1-explogq1
%��
q_t = QuatExp(QuatLog(q1)*4)
q1_t = QuatPower(q1,4)










