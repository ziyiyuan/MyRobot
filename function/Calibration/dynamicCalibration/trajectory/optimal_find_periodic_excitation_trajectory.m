function [finalPara, optimizationIndex] = optimal_find_periodic_excitation_trajectory(Robot, Traj, identificationType, identificationModel, MaxFunEvals, initialPara,sampleRate)
opt = optimset('display','on','LargeScale','off','MaxFunEvals',MaxFunEvals,'TolX',1e-3,'Algorithm', 'active-set'); %优化参数设置

fun = @(motionParaCoeff)opitimal_objective_function(Robot, Traj, motionParaCoeff, identificationModel,sampleRate);
nonlcon = @(motionParaCoeff)optimal_constrain_conditions(Robot,Traj, motionParaCoeff, identificationModel, sampleRate);

if strcmp(identificationType,'robot')  % robot identification
    [finalPara, conditionalNumber] = fmincon(fun,initialPara,[],[],[],[],[],[], nonlcon,opt);
    
    %     [finalPara, conditionalNumber] = fmincon('opitimal_result_function',initialPara,[],[],[],[],[],[], 'optimal_constrain_conditions',opt);
    %     [finalPara, conditionalNumber] = fmincon(@(qq)resultFunction(qq, identifyModel),initialPara,[],[],[],[],[],[], @(qq)constrainConditions(qq, identifyModel), opt);
elseif strcmp(identificationType,'tool') % tool identification
    [finalPara, conditionalNumber] = fmincon('payloadIdentification',initialPara,[],[],[],[],[],[], 'constrainConditions',opt);
end
optimizationIndex = conditionalNumber;
end
