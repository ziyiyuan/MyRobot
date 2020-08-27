
function q = QuatSlerp(q0,q1,t)
    dp = dot(q0,q1);
        if (dp < 0.0) % not the shortest path
        q0 = -q0; % q and -q represent the same rotation
        dp = -dp; % now dot product is positive
        end 
    if (dp > 0.9995) % divide by sin(theta) is dangerous
        q = nlerp(q0,q1,t); % so use nlerp instead
        return
    end
    theta = acos(dp);
    sin_theta = sin(theta);
    q = (sin((1-t)*theta)/sin_theta)*q0 + (sin(t*theta)/sin_theta)*q1;
end