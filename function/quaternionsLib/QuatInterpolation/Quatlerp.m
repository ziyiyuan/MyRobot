function q = Quatlerp(q0,q1,t)
    q = (1-t)*q0 + t*q1;
end