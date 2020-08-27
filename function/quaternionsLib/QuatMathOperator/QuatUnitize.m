% 四元数单位化；
function q = QuatUnitize(q1)
q = q1./norm(q1);
end