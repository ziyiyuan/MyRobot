% function f = resultFunction(qq, identifyModel)
function f = resultFunction(qq)

% 目标函数，优化准则：以辨识矩阵条件数最小为准则；
global optimal_res Traj identifyModel

% tic
optimal_res.motionPara = ObtainMotionParaForOptimal(qq,  Traj.optimal_sample);

regression = zeros(6 * size(optimal_res.motionPara.q,1),60);
for  i = 1 : size(optimal_res.motionPara.q,1)
    %% get the identification matrix
    kinematicsPara.q = optimal_res.motionPara.q(i,:);
    kinematicsPara.qd = optimal_res.motionPara.qd(i,:);
    kinematicsPara.qdd = optimal_res.motionPara.qdd(i,:);
    
    if strcmp(identifyModel,'Internal')
        ParaMatrix = IdentificationMatrix(kinematicsPara,'Internal');
    elseif strcmp(identifyModel,'External')
        ParaMatrix = IdentificationMatrix(kinematicsPara,'External');
    end
    regression(i*6-5:i*6,:) = ParaMatrix;
end
optimal_res.count = optimal_res.count + 1;

[Col,beta] = getMiniPara(regression);

W1 = regression(:,Col.i);
TT = W1'*W1;
D = svd(TT);
dMin = D(size(TT,2));
f = D(1)/dMin;

optimal_res.cond = [optimal_res.cond; f];
if mod(optimal_res.count,100) == 0
    disp(['total count:',num2str(optimal_res.count), ' condition number:', num2str(f)]);
    toc
%     figure(1)
%     plot(optimal_res.cond)
end

% if(count > 2*MaxFunEvals/3.0)
%     if f < maxF
%         maxF = f;
%         res.f = f;
%         res.qq = qq;
%         res.count = count;
%     end
% end
% if mod(count,50) == 0
%     count
%     figure(1)
%     fa = [fa;f];
%     plot(fa)
%     pause(0.05);
% end
% t2 = toc
end