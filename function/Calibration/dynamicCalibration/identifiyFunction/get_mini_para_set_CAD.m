function parasetCAD = get_mini_para_set_CAD(Robot,Col,beta)
Sp = Robot.Para_cad;
X1 = Sp(Col.i);X2 = Sp(Col.c);
parasetCAD = vpa(X1 + beta * X2);
end