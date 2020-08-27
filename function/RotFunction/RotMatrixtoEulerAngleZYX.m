function [qx,qy,qz] = RotMatrixtoEulerAngleZYX(R)
% rz * ry * rx
% rz rx = [-pi,pi]; ry = [-pi/2,pi/2];
if  1 - abs(R(3,1)) > eps
    qy = atan2(-R(3,1),sqrt(R(1,1) * R(1,1) + R(2,1) * R(2,1)));
    qz = atan2(R(2,1), R(1,1));
    qx = atan2(R(3,2),R(3,3));
else
    if R(3,1) == -1
        qy = pi/2;
        qz = 0;
        qx = atan2(R(1,2),R(2,2));
    elseif R(3,1) == 1
        qy = -pi/2;
        qz = 0;
        qx = atan2(R(1,2),R(2,2));
    end
end
end