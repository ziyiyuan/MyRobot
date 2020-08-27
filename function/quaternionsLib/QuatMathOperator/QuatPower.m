% 四元数的幂运算；
% 一个单位四元数的t 次幂等同于将它的旋转?度缩放至t 倍，并且不会
%改变它的旋转轴
function q_power_t = QuatPower(q, t)
angle = acos(q(1,1));
axis = q(2:4,1)/sin(angle);
q_power_t = [cos(t*angle);sin(t*angle)*axis];
end