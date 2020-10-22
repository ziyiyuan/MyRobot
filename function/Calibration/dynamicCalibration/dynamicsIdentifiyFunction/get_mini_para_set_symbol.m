function [X1,expersion] = get_mini_para_set_symbol(identifyPara,Col,beta)

    XX = sym('XX', [1 6],'real');
    XY = sym('XY', [1 6],'real');
    XZ = sym('XZ', [1 6],'real');
    YY = sym('YY', [1 6],'real');
    YZ = sym('YZ', [1 6],'real');
    ZZ = sym('ZZ', [1 6],'real');
    MX = sym('MX', [1 6],'real');
    MY = sym('MY', [1 6],'real');
    MZ = sym('MZ', [1 6],'real');
    M = sym('M', [1 6],'real');
    
    Ia = sym('Ia', [1 6],'real');
    fv = sym('fv', [1 6],'real');
    fc = sym('fc', [1 6],'real');
    fok = sym('fok', [1 6],'real');
    
    Sp = [];
    for i = 1:1:6
        if identifyPara.linkModel
            Sp = [Sp;[XX(i),XY(i),XZ(i),YY(i),YZ(i),ZZ(i),MX(i),MY(i),MZ(i),M(i)]'];
        end
        if identifyPara.roterInertiaModel
            Sp = [Sp;Ia(i)];
        end
        if identifyPara.frictionModel
            Sp = [Sp;[fv(i),fc(i)]'];
        end
        if identifyPara.offsetModel
            Sp = [Sp;fok(i)];
        end
    end
    X1 = Sp(Col.i);X2 = Sp(Col.c);
    expersion = vpa(X1 + beta * X2);
end