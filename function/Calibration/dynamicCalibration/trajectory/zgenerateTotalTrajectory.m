function generateTotalTrajectory(filename)
    global  Traj
    
    CyclePeriod = Traj.CyclePeriod; % %40 ¸öÖÜÆÚ£»
    res = load('res.txt');   %%The add joint angle to the start of the period trajectory
    res1 = load('res1.txt'); %%add to the end ;
    resR = flipud(res); %%resort in inverse order
    fid=fopen(['D:\', filename,'.rec'],'w');
    data = [resR;qt';qt';res1];   %%plot two period
    plot(data(:,1),'r-')
    hold on
    plot(data(:,2),'g-')
    hold on
    plot(data(:,3),'b-')
    hold on
    plot(data(:,4),'c-')
    hold on
    plot(data(:,5),'y-')
    hold on
    plot(data(:,6),'k-')
    for i = 1:size(resR,1)
        fprintf(fid, '%f%s%f%s%f%s%f%s%f%s%f\r\n',(resR(i,1)),',',(resR(i,2)),',',(resR(i,3)),',',(resR(i,4)),',',(resR(i,5)),',',(resR(i,6)));
    end
    for j = 1:CyclePeriod
        for i = 1:CN
            fprintf(fid, '%f%s%f%s%f%s%f%s%f%s%f\r\n',(qt(1,i)),',',(qt(2,i)),',',(qt(3,i)),',',(qt(4,i)),',',(qt(5,i)),',',(qt(6,i)));
        end
    end
    for i = 1:size(res1,1)
        fprintf(fid, '%f%s%f%s%f%s%f%s%f%s%f\r\n',(res1(i,1)),',',(res1(i,2)),',',(res1(i,3)),',',(res1(i,4)),',',(res1(i,5)),',',(res1(i,6)));
    end
    fclose(fid);
end
