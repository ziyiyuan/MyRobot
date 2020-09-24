function compare_wrench_based_cad_and_measure(Robot, motionTraj_s,sensor_avg_s,current_avg_s)
for i = 1:1:size(motionTraj_s.q,2)
    motionPara.q = motionTraj_s.q(:,i);
    motionPara.qd = motionTraj_s.qd(:,i);
    motionPara.qdd = motionTraj_s.qdd(:,i);
    wrench(:,i) = get_wrench_from_diff_identificationModel(Robot, motionPara, 'External');
    %         torque(:,i) = get_wrench_from_diff_identificationModel(Robot, motionPara, 'Internal') .* Robot.Para.TC;
    torque(:,i) = get_wrench_from_diff_identificationModel(Robot, motionPara, 'Internal');
end

if isempty(sensor_avg_s)
else
    figure(1)
    title_name = {'Fx','Fy','Fz','Tx','Ty','Tz'};
    for i = 1:1:6
        subplot(2,3,i);
        plot(wrench(i,:))
        hold on
        plot(sensor_avg_s(i,:))
        title(title_name{i})
        xlabel('Time')
        ylabel('Wrench(Nm)')
        hold off
    end
    legend('CAD','Sensor')
end

if isempty(current_avg_s)
else
    figure(2)
    title_name = {'I1','I2','I3','I4','I5','I6'};
    for i = 1:1:6
        subplot(2,3,i);
        plot(torque(i,:))
        hold on
        plot(current_avg_s(i,:))
        title(title_name{i})
        xlabel('Time')
        ylabel('Torque(Nm)')
        hold off
    end
    legend('CAD','Current')
end
end