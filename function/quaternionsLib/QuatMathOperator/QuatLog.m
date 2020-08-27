%返回四元数的指数形式 q 相当于旋转2*theta; 
function u_theta = QuatLog(q)
angle = acos(q(1,1));
if angle < eps
    u_theta = [0;0;0]'
else
    axis = q(2:4,1)/sin(angle);
    u_theta = angle*axis;
end
end