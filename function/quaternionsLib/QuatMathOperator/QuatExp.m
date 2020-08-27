%四元数的指数形式转化为四元数；
% q = e^(u*theta) = [cos(theta); sin(theta)* u]
function expq = QuatExp(u_theta)
angle = norm(u_theta);
if (angle < eps)
    expq = [1;0;0;0];
else
    axis = u_theta / angle;
    expq = [cos(angle);sin(angle)*axis];
end
end