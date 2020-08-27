function dim = test_dim_of_regression_get_minipara(regression, para_cad, identificationModel)
%test 辨识矩阵维度对最小参数集的影响，数值分析得到最小参数集最小的回归矩阵的维度
[index, MDP] = GetRobotIdyMimParaSet(identificationModel);
ss = find(index == 9);
index(ss) = [];
MDP(ss) = [];
j = 1;
for i = 1:1:100
    dim = 120*i;
    WW = regression(1:dim,:);
    [Col,beta] = getMiniPara(WW);
    if(size(Col.i,2) < 41)
        continue;
    end
    para_cad_ = para_cad(Col.i) + beta * para_cad(Col.c);
    % e1 = [index', Col.i']
    % para = [MDP, para_cad_]
    error =  norm(MDP - para_cad_);
    if error < 1e-8
        return;
    end
    e(j,1:3) = [j, dim, error];
    j = j+1;
end
plot(e(:,2),e(:,3));
save e e
end
