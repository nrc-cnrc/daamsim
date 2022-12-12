% Find RR Array batch file
clear all;
%Start by making an array of intruder speeds
intruderSpeedKts = [5:10:295];
%RR_array = zeros(length(intruderSpeedKts),1); %For memory alloc
fprintf(1,'Set up your DAA system params in calculateRrArray.m for now until they are functionized\n');
%for (i=1:length(intruderSpeedKts))
daaSpec.maxBank_deg = 45;
daaSpec.range_m = 1000;
daaSpec.FOV_deg = 60;
daaSpec.ownSize_m = 2;
daaSpec.ownSpeed_kts = 60;
daaSpec.maxRollRate_deg = 10; % 10 Deg/s roll rate
for (i=1:30) % Going through these just a little bit at a time
    fprintf(1,'Evaluating Intruder speed %d kts\n',intruderSpeedKts(i));
    [RR_array(i), Azimuth(i,:),R_min_close(i,:),R_min_over(i,:),AlphaOncoming(i,:),AlphaOvertake(i,:),VcloseOncoming(i,:), VcloseOvertake(i,:) ]=calculateRrArrayWithDetails(daaSpec,intruderSpeedKts(i),0); % Zero arg means don't plot
    % Save the results
    save rrBatchDetailsWorkspace
    
end