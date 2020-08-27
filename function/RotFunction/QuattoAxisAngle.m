function [axis,angle] = QuattoAxisAngle(q) %%��λ��Ԫ��ת��Ϊ��ǣ�
    angle = acos(q(1,1))*2;
    if(angle < 1e-8)
        axis = [1,0,0];
    else
    axis  = q(2:4,1)/sin(angle/2);
    end
end
