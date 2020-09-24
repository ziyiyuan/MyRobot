function detJ = get_jacobian_det(q, Robot)
% determinant;
a2 = Robot.Para.KP.a(3);
a3 = Robot.Para.KP.a(4);
d5 = Robot.Para.KP.d(5);
detJ = (a2 * sin(q(2)) + a3 * sin(q(2) - q(3)) + d5 * sin(q(2) - q(3) + q(4)))*sin(q(3))*sin(q(5));
end