function Tadd = homogeneous_transfer(alpha, a, d, theta)
% homogeneous_transfer ��mordified DH �����õ���α任����; ֻ������ modified DH
% ���������
%   alpha: rotx angle
%   a: trans in x axis;
%   d��trans in z axis;
%   theta��rotz angle
% ���������
%   Tadd����α任����
% ����˵����
%   Tadd = homogeneous_transfer(alpha, a, d, theta) :����������һ������ϵ����α任����

% �汾��V1.0����д��2020/8/27���޸���2020/8/27�����ߣ�ziyi

ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
Tadd = [ct,   -st,   0,    a;
    st*ca, ct*ca, -sa,  -sa*d;
    st*sa, ct*sa, ca,   ca*d;
    0,  0,  0,  1];
end
