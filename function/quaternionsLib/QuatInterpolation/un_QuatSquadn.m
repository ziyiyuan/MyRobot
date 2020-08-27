% º∆À„øÿ÷∆∂•µ„£ø
function q = QuatSquad(q1,s1,s2,q2,t)
q12 = QuatSlerp(q1,q2,t);
s12 = QuatSlerp(s1,s2,t)
q = QuatSlerp(q12,s12,2*t*(1-t));
end