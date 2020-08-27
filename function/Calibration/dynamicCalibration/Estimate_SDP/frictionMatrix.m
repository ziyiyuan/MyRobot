% motionPara
% HF = frictionMatrix1(motionPara)
% 线性摩擦力模型的辨识矩阵
function HF = frictionMatrix(motionPara)
qd = motionPara.qd;
DOF = size(qd,1);
for i = 1:DOF
    if i == 1
        HF = [qd(i), sign(qd(i)), 1];
    else
        HF_i = [qd(i), sign(qd(i)), 1];
        HF = blkdiag(HF,HF_i);
    end
end
end
