function regression = cal_identification_matrix(Robot, motionTraj, identificationModel, identifyPara)

regression = [];
for i = 1:1:size(motionTraj.q,2)
    motionPara.q = motionTraj.q(:,i);
    motionPara.qd = motionTraj.qd(:,i);
    motionPara.qdd = motionTraj.qdd(:,i);
    
    HH = cal_ele_identification_matrix(Robot, motionPara,identificationModel);
    HI = inertia_matrix(motionPara);
    HF = friction_matrix(motionPara);
    Hoff = offset_matrix(motionPara);
    % in order [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,Ia, fv,fc,fok
    % ](1:14), change order to compara with python
    HA = [];
    for j = 1:1:6
        H_i = [];
        if identifyPara.linkModel
            H_i = [H_i, HH(:,10*(j-1) + 1 : 10*j)];
        end
        if identifyPara.roterInertiaModel
            H_i = [H_i, HI(:,j)];
        end
        if identifyPara.frictionModel
            H_i = [H_i, HF(:,2*(j-1) + 1 : 2*j)];
        end
        if identifyPara.offsetModel
            H_i = [H_i, Hoff(:,j)];
        end
        HA = [HA,H_i];
    end
    regression = [regression;HA];
end
end