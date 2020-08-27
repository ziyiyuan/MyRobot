% q = q1 * q2;
function q = quatMult(q1,q2)
    q = QuatLeftMultiplyQuat(q1)*q2;
end