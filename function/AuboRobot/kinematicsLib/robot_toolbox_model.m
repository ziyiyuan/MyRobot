function aubo_i = robot_toolbox_model(robotType)
% robot_toolbox_model ����Peter Corke �Ĺ��߰�����������ģ�ͣ���������λ״̬
% ���������
%   robotType: ��е���ͺţ���I3��,'I5'...
% ���������
%   aubo_i��toolbox ģ�ͽṹ�壻
% ����˵����
%   Robot = get_cad_model_para('I3') :��� I3 ��ģ�ͣ�
%   Robot = get_cad_model_para() :��� I5 ��ģ�Ͳ�����

% �汾��V1.0����д��2020/8/27���޸���2020/8/31�����ߣ�ziyi

if nargin > 1
    error('����������࣡');
elseif nargin == 0
    robotType = 'I5'; % Ĭ�������ΪI5
end

Robot = get_cad_model_para(robotType);
DH = Robot.DH; % alpha a d theta

%% standard DH para of aubo_i
if 0
    a2 = Robot.Para.KP.a(3);
    a3 = Robot.Para.KP.a(4);
    d1 = Robot.Para.KP.d(1);
    d2 = Robot.Para.KP.d(2);
    d5 = Robot.Para.KP.d(5);
    d6 = Robot.Para.KP.d(6);
    L1 = Link('d', d1, 'a', 0, 'alpha', -pi/2, 'offset', pi);
    L2 = Link('d', d2, 'a', a2, 'alpha', pi, 'offset', -pi/2);
    L3 = Link('d', 0, 'a', a3, 'alpha',pi, 'offset', 0);
    L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2);
    L5 = Link('d', d5, 'a', 0, 'alpha', pi/2, 'offset', 0);
    L6 = Link('d', d6, 'a', 0, 'alpha',0, 'offset', 0);
    aubo_i= SerialLink([L1 L2 L3 L4 L5 L6], 'name', ['AUBO_',robotType]);
    qz = [0, 0, 0, 0, 0, 0];
    aubo_i.plot(qz);
%     A = aubo_i5.fkine(qz)
end

%% modified model// theta d a alpha revelute
if 1
    for i = 1:1:6
        S(i) = Link([ 0, DH(i,3), DH(i,2), DH(i,1), 0], 'modified');
    end
    aubo_i= SerialLink(S, 'name', robotType);
    aubo_i.offset =  DH(:,4)';
    qz = [0, 0, 0, 0, 0, 0]; % ��λ
    aubo_i.plot(qz);
end
% B = aubo_i.fkine(qz);
end