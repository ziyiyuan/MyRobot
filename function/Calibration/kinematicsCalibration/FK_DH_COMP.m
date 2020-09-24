function T06 = FK_DH_COMP(DHBeta,joint)   
T06 = eye(4);
    for i=1:6
        Tii = [RotY(DHBeta(i,5)) zeros(3,1);0 0 0 1] * homogeneous_transfer(DHBeta(i,1),DHBeta(i,2),DHBeta(i,3),DHBeta(i,4) + joint(i));
        T06 = T06*Tii;
    end
end