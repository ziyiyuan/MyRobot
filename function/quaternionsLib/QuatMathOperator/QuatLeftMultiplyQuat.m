% ���һ����Ԫ���ľ�����ʽ
% ����һ��4*4 �����þ���
% q1 * q2 = QuatLeftMultiplyQuat(q1) * q2
function q = QuatLeftMultiplyQuat(quat)
    a = quat(1,1);
    b = quat(2,1);
    c = quat(3,1);
    d = quat(4,1);
    q = [a -b -c -d
         b  a -d  c
         c  d  a -b
         d -c  b  a];
end