% ��Ԫ������
function q_1 =  QuatInverse(q1)
q_1 = QuatConjugate(q1)/(norm(q1)*norm(q1));
end
