% motionPara
% HI = inertiaMatrix(motionPara)
% ת�ӹ����ı�ʶ����
function HI = inertia_matrix(motionPara)
qdd = motionPara.qdd;
HI = diag(qdd);
end