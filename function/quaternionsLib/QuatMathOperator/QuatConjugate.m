% ���룺 q1(4,1);
% ����� q1 �� ������Ԫ����
function q = QuatConjugate(q1)
    q = [q1(1,1);-q1(2,1);-q1(3,1);-q1(4,1)];
end