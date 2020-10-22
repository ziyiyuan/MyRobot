% ���㵥����������ؾ���
% get_rigid_body_wrench_linear_matrix ���㵥����������ؾ���
% ���������
%   w:  ������ٶȣ�����������õ㣩
%   wd: ����Ǽ��ٶȣ�����������õ㣩
%   vd: ������ٶȣ�����������õ㣩
% ���������
%   Wii��6*10 �ľ���
% ����˵����
%   Wii = get_rigid_body_wrench_linear_matrix(w,wd,vd)

% �汾��V1.0����д��2020/10/22���޸���2020/-/-�����ߣ�ziyi

function Wii = get_rigid_body_wrench_linear_matrix(w,wd,vd)
% Fi = Wii * Para_i��para in order Ixx Ixy Ixz Iyy Iyz Izz mcx mcy mcz m
Wii = [zeros(3,6) skew(wd)+skew(w)*skew(w) vd;skewStar(wd)+skew(w)*skewStar(w) -skew(vd) zeros(3,1)]; %Wii
end