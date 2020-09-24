function T = forward_kinematics(q, DHMatrix, i)
% forward_kinematics ����
% ���������
%   q: ����Ĺؽڽǣ�6*1
%   DHMatrix: DH��������˳��Ϊ [alpha, a, d, theta]��6*4
%   i���� i ���ؽڽ��ڻ�����ϵ������
% ���������
%   T����α任����
% ����˵����
%   T = forward_kinematics(q, DHMatrix) :������йؽ�����ϵ�ڻ�����ϵ��������
%   T = forward_kinematics(q, DHMatrix, i) ������� i ���ؽ�����ϵ�ڻ�����ϵ��������

% �汾��V1.0����д��2020/8/27���޸���2020/8/27�����ߣ�ziyi

if nargin > 3
    error('����������࣡');
end

T0_1 = homogeneous_transfer(DHMatrix(1,1),DHMatrix(1,2),DHMatrix(1,3),DHMatrix(1,4) + q(1));
T1_2 = homogeneous_transfer(DHMatrix(2,1),DHMatrix(2,2),DHMatrix(2,3),DHMatrix(2,4) + q(2));
T2_3 = homogeneous_transfer(DHMatrix(3,1),DHMatrix(3,2),DHMatrix(3,3),DHMatrix(3,4) + q(3));
T3_4 = homogeneous_transfer(DHMatrix(4,1),DHMatrix(4,2),DHMatrix(4,3),DHMatrix(4,4) + q(4));
T4_5 = homogeneous_transfer(DHMatrix(5,1),DHMatrix(5,2),DHMatrix(5,3),DHMatrix(5,4) + q(5));
T5_6 = homogeneous_transfer(DHMatrix(6,1),DHMatrix(6,2),DHMatrix(6,3),DHMatrix(6,4) + q(6));

Ti{1} = T0_1;
Ti{2} = Ti{1}*T1_2;
Ti{3} = Ti{2}*T2_3;
Ti{4} = Ti{3}*T3_4;
Ti{5} = Ti{4}*T4_5;
Ti{6} = Ti{5}*T5_6;

if nargin == 2
    T = Ti;
else
    T = Ti{i};
end
end