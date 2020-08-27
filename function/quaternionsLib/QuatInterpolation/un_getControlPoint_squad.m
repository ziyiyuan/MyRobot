function s2 = getControlPoint(q1,q2,q3)
inv_q2 = QuatInverse(q2)
temp1 = QuatLeftMultiplyQuat(inv_q2)*q3;
temp2 = QuatLeftMultiplyQuat(inv_q2)*q1;
temp3 = -(Quatlog(temp1) + Quatlog(temp2))/4;
s2 = QuatLeftMultiplyQuat(q2) * Quatexp(temp3);
end