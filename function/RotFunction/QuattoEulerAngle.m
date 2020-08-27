function [qx,qy,qz]= QuattoEulerAngle(q)
R = QuattoRotMatrix(q);
[qx,qy,qz] = RotMatrixtoEulerAngleZYX(R);
end