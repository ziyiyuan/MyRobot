%Rzyx = Rz * Ry * Rx, ��ϵ zyx   =  ��ϵ x y z
function R = EulerAngletoRotMatrixZYX(qx,qy,qz)
R = RotZ(qz) * RotY(qy) * RotX(qx);
end