%Batch job assembler


daaSpec.maxBank_deg = 45;
daaSpec.range_m = 1000;
daaSpec.FOV_deg = 60;
daaSpec.ownSize_m = 2;
daaSpec.ownSpeed_kts = 60;

% Now make a pretty plot

[Xclose,Yclose]=pol2cart(Azimuth*pi/180,R_min_close);
[Xover,Yover]=pol2cart(Azimuth*pi/180,R_min_over);
%Build the Z matrix of intruder speeds in kts
Z = ones(30,1439);
for(i=1:30)
Z(i,:)=Z(i,:)*intruderSpeedKts(i);
end
figure
surfc(Xclose,Yclose,Z,R_min_close)
shading flat
hold on
surfc(Xover,Yover,Z,R_min_over)
shading flat
xlabel('R min(m)');
ylabel('R min (m)');
zlabel('Intruder Speed (kts)')
colorbar;
%
myTitleLine = sprintf('R min %d kts ownship, %d deg Bank, Various Intruder Speeds', daaSpec.ownSpeed_kts,daaSpec.maxBank_deg);
title(myTitleLine);

%calculate the daa and dsaa rr for all cases
for caseNumber=1:length(intruderSpeedKts)
    [daaRr(caseNumber), dsaaRr(caseNumber)]=plotDaaRrVsAzimuthWseeAndAvoid(Azimuth(caseNumber,:),R_min_close(caseNumber,:),Azimuth(caseNumber,:),R_min_over(caseNumber,:),VcloseOncoming(caseNumber,:),VcloseOvertake(caseNumber,:),daaSpec,intruderSpeedKts(caseNumber),0);
end % for loop


% Now let's build a pass fail 3D graph
myFigHandle = figure;
%Putting in our DAA system specs here
FOV = daaSpec.FOV_deg;
Range = daaSpec.range_m; % meters
OwnSpeed = daaSpec.ownSpeed_kts; % kts

% Note that this plot only considers techincal DAA mitigations!
for i=1:length(intruderSpeedKts)
    IntSpeed = intruderSpeedKts(i);
    overplot3DaaRrVsAzimuth(Azimuth(i,:),R_min_close(i,:),Azimuth(i,:), R_min_over(i,:),FOV,Range, OwnSpeed,IntSpeed,0,myFigHandle)
end

grid on;
xlabel('R_m_i_n (m)');
ylabel('R_m_i_n (m)');
zlabel('Intruder Spd (kts)');
myTitleLine1 = sprintf('R min Pass/Fail for %d kts ownship, %d deg Bank', daaSpec.ownSpeed_kts,daaSpec.maxBank_deg);
myTitleLine2 = sprintf('Various Intruder Speeds, No See & Avoid');
title({myTitleLine1, myTitleLine2});
hold on; % So we can plot the FOV/RangeSpec

%Make some polys for plotting
alphaVal = 0.15;
polyY1=[0 Range*sind(FOV/2) Range*sind(FOV/2) 0];
polyX1=[0 Range*cosd(FOV/2) Range*cosd(FOV/2) 0];
polyZ1=[0 0 intruderSpeedKts(end) intruderSpeedKts(end)];
polyY2=[0 Range*sind(-FOV/2) Range*sind(-FOV/2) 0];
polyX2=[0 Range*cosd(-FOV/2) Range*cosd(-FOV/2) 0];
h1 =fill3(polyX1,polyY1,polyZ1,'k');
h1.FaceAlpha=alphaVal; % Transparent
h2 =fill3(polyX2,polyY2,polyZ1,'k');
h2.FaceAlpha=alphaVal; % Transparent
% Now lets make some faces for the front of the range
angleSteps = [-FOV/2:5:FOV/2]; %Hope 5 deg steps looks ok

polyXangle = [Range*cosd(angleSteps) fliplr(Range*cosd(angleSteps))];
polyYangle = [Range*sind(angleSteps) fliplr(Range*sind(angleSteps))];
polyZangle = [zeros(1,length(angleSteps)) intruderSpeedKts(end)*ones(1,length(angleSteps))];
h3 = fill3(polyXangle,polyYangle,polyZangle,'k');
h3.FaceAlpha = alphaVal;

%Plot the risk ratio bar chart

%And the Bar char with See and Avoid
plotRrBinPassFailChart(intruderSpeedKts,daaRr,daaSpec)
plotRrBinPassFailChartSeeAndAvoid(intruderSpeedKts,daaRr,dsaaRr,daaSpec);







