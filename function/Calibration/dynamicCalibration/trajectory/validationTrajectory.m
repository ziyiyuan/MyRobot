function valid = validationTrajectory(qq, identifyModel)
global Robot Traj

if strcmp(identifyModel,'Internal')
    torLimit = Robot.Limit.torque;
    posLimit = Robot.Limit.q;
elseif strcmp(identifyModel,'External')
    torLimit = Robot.Limit.sensor;  %%%input the sensor force limit
    posLimit = Robot.Limit.sensorP; %% 防止碰到基座传感器；
end
velLimit = Robot.Limit.qd;
accLimit = Robot.Limit.qdd;

ARM_DOF = Robot.DOF;

OrderNumber = Traj.OrderNumber;
waveFrequency = Traj.waveFrequency;
TrajectoryPeriod = Traj.TrajectoryPeriod;

CN = TrajectoryPeriod * Traj.sampleRate; % 每个周期内的采样点个数；
dt = linspace(0,TrajectoryPeriod,CN);

qt = zeros(ARM_DOF,CN); qtd = zeros(ARM_DOF,CN); qtdd = zeros(ARM_DOF,CN);
totalTorque = zeros(6,CN);

valid = true;

for i = 1:CN
    for j = 1:ARM_DOF
        sumq = 0; sumqd = 0; sumqdd=0;
        for k = 1 : OrderNumber
            sumq = sumq + qq(5 * j + k + 1) * sin(k * waveFrequency * dt(i)) + qq(5 * j + k + 31) * cos(k * waveFrequency * dt(i));
            sumqd = sumqd + qq(5 * j + k + 1) * k * waveFrequency * cos(k * waveFrequency * dt(i)) - qq(5 * j + k + 31) * k * waveFrequency * sin(k * waveFrequency * dt(i));
            sumqdd = sumqdd - qq(5 * j + k + 1) * k^2 * waveFrequency^2 * sin(k * waveFrequency * dt(i)) - qq(5 * j + k + 31) * k^2 * waveFrequency^2 * cos(k * waveFrequency * dt(i));
        end
        qt(j,i) = qq(j) + sumq;
        qtd(j,i) = sumqd;
        qtdd(j,i) = sumqdd;
        if abs(qt(j,i)) > posLimit(j) || abs(qtd(j,i)) > velLimit(j) || abs(qtdd(j,i)) > accLimit(j)
            valid = false;
        end
    end
    %% torque validation
    
    kinematicsPara.q = qt(:,i)';
    kinematicsPara.qd = qtd(:,i)';
    kinematicsPara.qdd = qtdd(:,i)';
    
    if strcmp(identifyModel,'Internal')
        torque = identificationModel(kinematicsPara,'Internal'); %% torque in joint
    elseif strcmp(identifyModel,'External')
        torque =  identificationModel(kinematicsPara,'External',FKLink(kinematicsPara.q,1)); %% iter NE  wrench inbase
        % torque = GetBaseWrenchBasepara(kinematicsPara);
    end
    
    for j = 1:ARM_DOF
        if abs(torque(j)) > torLimit(j)
            valid = false;
            break;
        end
    end
    totalTorque(:,i) = torque;
end

%% plot
if valid == true
    figure(1)
    title('qt')
    figure(2)
    title('qtd')
    figure(3)
    title('qtdd')
    figure(4)
    title('totalTorque')
    
    for i = 1:1:ARM_DOF
        figure(1)
        plot(qt(:,i));
        
        figure(2)
        plot(qtd(:,i));
        
        figure(3)
        plot(qtdd(:,i));
        
        figure(4)
        plot(totalTorque(i,:));
    end
    
    optTraPara.pSeq = qt;
    optTraPara.vSeq = qtd;
    optTraPara.aSeq = qtdd;
    save optTraPara optTraPara
    
    posN = optTraPara.pSeq(CN,:);
    velN =  optTraPara.vSeq(CN,:);
    accN = optTraPara.aSeq(CN,:);
    pos1 = optTraPara.pSeq(1,:);
    vel1 =  -optTraPara.vSeq(1,:);
    acc1 = optTraPara.aSeq(1,:);
    
    fid = fopen('.\pva.txt','w');
    fprintf(fid, '%s%f%s%f%s%f%s%f%s%f%s%f%s\r\n','double pos1[] = {',(pos1(1)),',',(pos1(2)),',',(pos1(3)),',',(pos1(4)),',',pos1(5),',',pos1(6),'};');
    fprintf(fid, '%s%f%s%f%s%f%s%f%s%f%s%f%s\r\n','double vel1[] = {',(vel1(1)),',',(vel1(2)),',',(vel1(3)),',',(vel1(4)),',',(vel1(5)),',',vel1(6),'};');
    fprintf(fid, '%s%f%s%f%s%f%s%f%s%f%s%f%s\r\n','double acc1[] = {',(acc1(1)),',',(acc1(2)),',',(acc1(3)),',',(acc1(4)),',',(acc1(5)),',',acc1(6),'};');
    fprintf(fid, '%s%f%s%f%s%f%s%f%s%f%s%f%s\r\n','double posN[] = {',(posN(1)),',',(posN(2)),',',(posN(3)),',',(posN(4)),',',posN(5),',',posN(6),'};');
    fprintf(fid, '%s%f%s%f%s%f%s%f%s%f%s%f%s\r\n','double velN[] = {',(velN(1)),',',(velN(2)),',',(velN(3)),',',(velN(4)),',',(velN(5)),',',velN(6),'};');
    fprintf(fid, '%s%f%s%f%s%f%s%f%s%f%s%f%s\r\n','double accN[] = {',(accN(1)),',',(accN(2)),',',(accN(3)),',',(accN(4)),',',(accN(5)),',',accN(6),'};');
    fclose(fid);
    
    
    fid = fopen('.\record.offt','w');
    for i = 1:CN
        fprintf(fid, '%f%s%f%s%f%s%f%s%f%s%f\r\n',(qt(i,1)),',',(qt(i,2)),',',(qt(i,3)),',',(qt(i,4)),',',(qt(i,5)),',',(qt(i,6)));
    end
    fclose(fid);
end
end

