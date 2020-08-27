% ÓÒ³ËÐý×ª¾ØÕó£º
% q2 * q1 = QuatRigitMultiplyQuat(q1) * q2
function q = QuatRigitMultiplyQuat(quat)
    a = quat(1,1);
    b = quat(2,1);
    c = quat(3,1);
    d = quat(4,1);
    q = [a -b -c -d
         b  a  d -c
         c  -d  a b
         d c  -b  a];
end