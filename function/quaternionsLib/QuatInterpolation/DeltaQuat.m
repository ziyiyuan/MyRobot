%dq *  q1 = q2;% in global ;; i
function dq = DeltaQuat(q1,q2)
dq = QuatLeftMultiplyQuat(q2) * QuatInverse(q1)
end

