function f = opitimal_objective_function(Robot, Traj, motionParaCoeff, identificationModel,sampleRate)
global  optimal_res
% Ŀ�꺯�����Ż�׼���Ա�ʶ������������СΪ׼��

motionPara = cal_motionPara_from_fourier_series(Robot,Traj, motionParaCoeff, sampleRate);
optimal_res.motionPara = motionPara;
%% get the identification matrix
regression = zeros(6 * size(motionPara.q,2),60);
for  i = 1 : size(motionPara.q,2)
    kinematicsPara.q = motionPara.q(:,i);
    kinematicsPara.qd = motionPara.qd(:,i);
    kinematicsPara.qdd = motionPara.qdd(:,i);
    paraMatrix = cal_identification_matrix(Robot, kinematicsPara,identificationModel);
    regression(i*6-5:i*6,:) = paraMatrix;
end
optimal_res.count = optimal_res.count + 1;
if optimal_res.count == 1
    [Col,beta] = get_mini_para_set_numerical(regression);
    optimal_res.Col = Col;
end
W1 = regression(:,optimal_res.Col.i);

f = cond(W1)^2;

optimal_res.cond(optimal_res.count) = f;
if mod(optimal_res.count,100) == 0
    disp(['total count:',num2str(optimal_res.count), ' condition number:', num2str(f)]);
    toc
    figure(1)
    plot(optimal_res.cond)
    hold on
end
end