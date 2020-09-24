function  J = cal_jacobian(q,DHMatrix,massCenter)
% cal_jacobian �����ſ˱Ⱦ���
% ���������
%   q: ����Ĺؽڽǣ�6*1
%   DHMatrix: DH��������˳��Ϊ [alpha, a, d, theta]��6*4
%   massCenter����������������ϵ�����ꣻ
% ���������
%   J���ſ˱Ⱦ���6*6*6
% ����˵����
%   J = cal_jacobian(q,DHMatrix,massCenter) :��������������Ĵ����ſ˱Ⱦ�����������ѧ M ����
%   J = cal_jacobian(q,DHMatrix) �����������������ϵ�����ſ˱Ⱦ���

% �汾��V1.0����д��2020/8/27���޸���2020/8/27�����ߣ�ziyi

calCenter = 1;
if nargin > 3
    error('����������࣡');
elseif nargin < 3
    calCenter = 0;
end

T0 = forward_kinematics(q, DHMatrix);

z0 = [[]]; o0 = [[]];
P = [[]];
for  i = 1:1:6
    
    z0{i} = T0{i}(1:3,3); % ��ת�� in base
    o0{i} = T0{i}(1:3,4); % λ��
    if calCenter
        P{i} = T0{i}(1:3,4) + T0{i}(1:3,1:3) * massCenter{i}; % ��������������ϵ������
    else
        P{i} = o0{i}; % joint coordinate position in base
    end
end

% cal jacobian
JLv = [[]];
JLw = [[]];
Jall = [[]];
for i = 1:1:6
    JLv{i} = zeros(3,6);
    JLw{i} = zeros(3,6);
    for j = 1:1:i
        JLv{i}(:,j) = cross(z0{j},(P{i}-o0{j}));
        JLw{i}(:,j) = z0{j};
    end
    Jall{i} = [JLv{i};JLw{i}];
end
J = Jall;
end