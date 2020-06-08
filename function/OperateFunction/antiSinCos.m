function  angle = antiSinCos(sA, cA)
% ÷µ”Ú (-pi,pi] ,∑µªÿ atan2, use in Ik
    eps = 1e-5;
    r = sqrt(sA*sA+cA*cA);
    sA  = sA / r;
    cA  = cA / r;
    if (abs(cA) < eps)
        angle = pi/2.0*sign(sA);
    elseif(abs(sA) < eps)
        if sign(cA)== 1
            angle =  0;
        else
            angle = pi;
        end
    else
        angle = atan2(sA, cA);
    end
