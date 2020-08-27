% 左乘一个四元数的矩阵形式
% 返回一个4*4 的作用矩阵；
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