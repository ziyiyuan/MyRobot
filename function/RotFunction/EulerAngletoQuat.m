function q = EulerAngletoQuat(qx,qy,qz)
R = EulerAngletoRotMatrixZYX(qx,qy,qz);
q = RotMatrixtoQuat(R);
end