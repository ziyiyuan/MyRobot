% ��Ԫ���������㣻
% һ����λ��Ԫ����t ���ݵ�ͬ�ڽ�������ת?��������t �������Ҳ���
%�ı�������ת��
function q_power_t = QuatPower(q, t)
angle = acos(q(1,1));
axis = q(2:4,1)/sin(angle);
q_power_t = [cos(t*angle);sin(t*angle)*axis];
end