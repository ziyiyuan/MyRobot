% ���ش��ݾ���
% get_rigid_body_wrench_linear_matrix ���㵥����������ؾ���
% ���������
%   w:  ������ٶȣ�����������õ㣩
%   wd: ����Ǽ��ٶȣ�����������õ㣩
%   vd: ������ٶȣ�����������õ㣩
% ���������
%   Wii��6*10 �ľ���
% ����˵����
%   HA = trans_wrench_linear_matrix(T, Wii)

% �汾��V1.0����д��2020/10/22���޸���2020/-/-�����ߣ�ziyi

function HA = trans_wrench_linear_matrix(T, Wii)
RR = TransferForce(T); %TAi
HA = RR * Wii;
end