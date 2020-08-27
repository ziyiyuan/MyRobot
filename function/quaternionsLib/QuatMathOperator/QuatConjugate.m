% 输入： q1(4,1);
% 输出： q1 的 共轭四元数；
function q = QuatConjugate(q1)
    q = [q1(1,1);-q1(2,1);-q1(3,1);-q1(4,1)];
end