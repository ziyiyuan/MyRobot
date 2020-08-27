%Rzyx = Rz * Ry * Rx, 动系 zyx   =  定系 x y z
function R = EulerAngletoRotMatrixZYX(qx,qy,qz)
R = RotZ(qz) * RotY(qy) * RotX(qx);
end