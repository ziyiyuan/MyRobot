% motionPara
% HI = inertiaMatrix(motionPara)
% ת�ӹ����ı�ʶ����
function HI = inertiaMatrix(motionPara)
qdd = motionPara.qdd;
HI = diag(qdd);
end