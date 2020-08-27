function [finalPara, optimizationIndex] = FindperiodicEexcitationTrajectory(initialPara, identificationType, MaxFunEvals, identifyModel)
%
%     initialPara = (rand(1,66)*2 - 1);
%     MaxFunEvals = 12000;
opt = optimset('display','on','LargeScale','off','MaxFunEvals',MaxFunEvals,'TolX',1e-3,'Algorithm', 'active-set'); %�Ż���������
if strcmp(identificationType,'robot')  % robot identification
    [finalPara, conditionalNumber] = fmincon('resultFunction',initialPara,[],[],[],[],[],[], 'constrainConditions',opt);
%     [finalPara, conditionalNumber] = fmincon(@(qq)resultFunction(qq, identifyModel),initialPara,[],[],[],[],[],[], @(qq)constrainConditions(qq, identifyModel), opt);
elseif strcmp(identificationType,'tool') % tool identification
    [finalPara, conditionalNumber] = fmincon('payloadIdentification',initialPara,[],[],[],[],[],[], 'constrainConditions',opt);
end
optimizationIndex = conditionalNumber;
end