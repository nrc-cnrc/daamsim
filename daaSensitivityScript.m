%DAA Spec Sensitivity Plotter
% The idea here is to build a mesh of DAA specs (range/FOV) and plot the total RR against it.
% This could be useful for speccing out systems


daaSpec.maxBank_deg = 45;
daaSpec.range_m = 1000;
daaSpec.FOV_deg = 60;
daaSpec.ownSize_m = 2;
daaSpec.ownSpeed_kts = 60;
fovIncrement = 5; % Degrees
numFovs = 360/fovIncrement;
rangeIncrement = 50; % meters
maxRange = 3000;
numRanges = maxRange/rangeIncrement;

for i=1:numFovs
    fov(i)=i*fovIncrement;
    daaSpec.FOV_deg=fov(i);
   for k=1:numRanges
       range(k) = k*rangeIncrement;
       daaSpec.range_m = range(k);
       for caseNumber=1:length(intruderSpeedKts)
            [daaRr(caseNumber), dsaaRr(caseNumber)]=plotDaaRrVsAzimuthWseeAndAvoid(Azimuth(caseNumber,:),R_min_close(caseNumber,:),Azimuth(caseNumber,:),R_min_over(caseNumber,:),VcloseOncoming(caseNumber,:),VcloseOvertake(caseNumber,:),daaSpec,intruderSpeedKts(caseNumber),0);
       end % for loop
       %  Now calculate the total RR by the airspace distribution
       [rr(i,k)]= calculateTotalRRwSeeAndAvoid(intruderSpeedKts,daaRr,dsaaRr,daaSpec);
   end
end

figure
surfc(range,fov,rr)
myTitle = sprintf('RR Sensitivity for %d kts Own, %d deg Bank',daaSpec.ownSpeed_kts,daaSpec.maxBank_deg);
xlabel('Range (m)')
ylabel('FOV (deg)')
zlabel('RR')
title(myTitle);
    
figure;contour(range,fov,rr,[0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1])
grid on
colorbar
myTitleLine1 = sprintf('RR Sensitivity for %d kts Own, %d deg Bank',daaSpec.ownSpeed_kts,daaSpec.maxBank_deg);
myTitleLine2 = sprintf('Lines At RR Increments of  0.1');
title({myTitleLine1; myTitleLine2});
xlabel('DAA Range(m)')
ylabel('DAA FOV (deg)');
