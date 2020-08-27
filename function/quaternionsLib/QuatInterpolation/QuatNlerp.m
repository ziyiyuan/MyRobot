function q = QuatNlerp(q0,q1,t)
    q = Quatlerp(q0,q1,t);
    q = q/norm(q);
end