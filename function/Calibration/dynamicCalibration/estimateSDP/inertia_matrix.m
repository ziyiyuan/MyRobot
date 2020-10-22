% motionPara
% HI = inertiaMatrix(motionPara)
% 转子惯量的辨识矩阵
function HI = inertia_matrix(motionPara)
qdd = motionPara.qdd;
HI = diag(qdd);
end