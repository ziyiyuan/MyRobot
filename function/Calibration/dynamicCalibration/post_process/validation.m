function E = validation(Robot,identificationModel, postData, paraEst)
%% validation
identifyPara.linkModel = 1;
identifyPara.offsetModel = 1;
if strcmp(identificationModel,'External')
    identifyPara.frictionModel = 0;
    identifyPara.roterInertiaModel = 0;
elseif strcmp(identificationModel,'Internal')
    identifyPara.frictionModel = 1;
    identifyPara.roterInertiaModel = 1;
end

%% load data
if strcmp(identificationModel,'External')
    wrench_all = postData.sensorData;
    title_name = {'Fx','Fy','Fz','Tx','Ty','Tz'};
elseif strcmp(identificationModel,'Internal')
    wrench_all = postData.currentData;
    title_name = {'Torque1','Torque2','Torque3','Torque4','Torque5','Torque6'};
end
data_num = size(postData.motionTraj.q,2);

%% cal identification matrix and joint torque/wrench from current
regression = cal_identification_matrix(Robot, postData.motionTraj, identificationModel, identifyPara);

tau = [];
for i = 1:data_num
    tau = [tau;wrench_all(:,i)];
end

[Col,beta] = get_mini_para_set_numerical(regression);
wrenchEst = [[]];
A = regression(:,Col.i);
for i = 1:size(paraEst,2)
    tauEst = A * paraEst(:,i);
    E(i) = norm(tau - tauEst)/norm(tau);
    wrenchEst{i} = reshape(tauEst,[size(wrench_all,1),size(wrench_all,2)]);
end


figure(1)
for i = 1:1:6
    subplot(2,3,i);
    plot(wrench_all(i,:))
    hold on
    plot(wrenchEst{1}(i,:))
        hold on
    plot(wrenchEst{2}(i,:))
    title(title_name{i})
    xlabel('Time')
    ylabel('Wrench/Torque(Nm)')
    hold off
end
legend('measure','OLS_estimate','SDP_estimate')
pause(5)
end