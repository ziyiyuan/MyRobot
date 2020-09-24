function Hoff = offset_matrix(motionPara)
DOF = size(motionPara.q,1);
Hoff = eye(DOF);
end