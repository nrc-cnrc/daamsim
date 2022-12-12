function [daaRr, dsaaRr]= plotDaaRrVsAzimuthWseeAndAvoid(azimuthDegOncoming,RminOncoming,azimuthDegOvertake,RminOvertake,VcloseOncoming,VcloseOvertake, daaSpec,IntSpeed,showPlot)
%[daaRr, dsaaRr]= plotDaaRrVsAzimuth(azimuthDegOncoming,RminOncoming,azimuthDegOvertake, RminOvertake,FOV,Range, OwnSpeed,IntSpeed,showPlot)
% daaRr is the RR for just the technical DAA system
% dsaaRr includes cases that are mitigated by intruder see and avoid
%Units should be in meters...that's what my labels say!....make showPlot 0 to not show plots
% NOTE: We need something passed for overtake cases. They can be empty vectors [] for cases where no overtakes are possible
%Does a polar plot for a given Rmin azimuth combination
% NOTE2: This code curently assumes a fixed range over the whole FOV...it's Iryna's job to convert this to make it variable range over the FOV
% NOTE3: This code also assumes a fixed azimuth interval for both oncoming and overtaking...don't change this, or the maths won't work

% DAA System Specs Here
FOV = daaSpec.FOV_deg; % Deg
Range = daaSpec.range_m; % Range in meters
OwnSpeed = daaSpec.ownSpeed_kts;  % Leave in kts as it's only used for plot title

%Specs for See and avoid here:
detectionThreshArcMin = 30; % Fairly conservative
intruderManeuverDelay = 12.5; % Right out of AC 90-48D
[detectionRange,maxClosingVelForSeeAndAvoid] = calculateSeeAndAvoidForRpaSizeAndIntruderManeuverDelay(daaSpec.ownSize_m,detectionThreshArcMin,intruderManeuverDelay);


% Now we need to figure out what to do

%For plotting
daaSystemAzimuth = [-FOV/2:1:FOV/2]; % A vector of azimuths covered by the DAA system
daaRange = Range*ones(length(daaSystemAzimuth),1);
% Now we want to look at the azimuthDeg vector to see if there is a corresponding daaSystemAzimuth for whatever value is in there
%Start with Oncoming because there will always be oncoming cases.

numberOfAzimuthsEvaluated = 0;
numberOfAzimuthsPassed = 0;
indexOfOncomingAzimuthsPassed = [];


for (i=1:length(azimuthDegOncoming))
    if (~isnan(RminOncoming(i)))
        if (abs(azimuthDegOncoming(i))<=(FOV/2))
            % The case is within the FOV
            % Now evaluate for range
            if (RminOncoming(i)<=Range)
                numberOfAzimuthsPassed = numberOfAzimuthsPassed + 1;
                indexOfOncomingAzimuthsPassed = [indexOfOncomingAzimuthsPassed i];
            end
        end
        numberOfAzimuthsEvaluated = numberOfAzimuthsEvaluated + 1;
    end
    
end % for loop through all oncoming azimuths

%Now lets calculate which oncoming azimuths were below the closing speed threshold for See and avoid to work
seeAndAvoidOncomingIndex = find(abs(VcloseOncoming)<maxClosingVelForSeeAndAvoid);


% Now we need to see if there are any Overtake azimuths
if (~isempty(azimuthDegOvertake))
    indexOfOvertakeAzimuthsPassed = [];
    for (i=1:length(azimuthDegOvertake))
        if (~isnan(RminOvertake(i)))
            if (abs(azimuthDegOvertake(i))<=(FOV/2)) %% The geometry is in the FOV
                if (RminOvertake(i)< Range)
                    numberOfAzimuthsPassed = numberOfAzimuthsPassed + 1;
                    indexOfOvertakeAzimuthsPassed =[indexOfOvertakeAzimuthsPassed i];
                end
            end
            numberOfAzimuthsEvaluated = numberOfAzimuthsEvaluated + 1;
        end
    end % for loop
    seeAndAvoidOvertakeIndex = find(abs(VcloseOvertake)<maxClosingVelForSeeAndAvoid);
end % if we have overtakes


%Now we plot the results
if (showPlot ~= 0)
    figure;
    myPolarAxis = polaraxes;
    %Start by plotting the failed cases
    polarplot(myPolarAxis,azimuthDegOncoming*pi/180,RminOncoming,'.r');
    hold on
    if (~isempty(azimuthDegOvertake))
        polarplot(myPolarAxis,azimuthDegOvertake*pi/180,RminOvertake,'.r');
    end
    
    %Now add in the cases that are mitigated by see and avoid
    %Now overplot the see and avoid canses in blue?
    polarplot(myPolarAxis,azimuthDegOncoming(seeAndAvoidOncomingIndex)*pi/180,RminOncoming(seeAndAvoidOncomingIndex),'.b');
    if (~isempty(azimuthDegOvertake))
        polarplot(myPolarAxis,azimuthDegOvertake(seeAndAvoidOvertakeIndex)*pi/180,RminOncoming(seeAndAvoidOvertakeIndex),'.b');
    end
    
    %Now plot the DAA system range/fov in black --
    polarplot(myPolarAxis,[0 daaSystemAzimuth*pi/180 0],[0; daaRange; 0],'--k');
    %Now overplot the passed cases in green
    polarplot(myPolarAxis,azimuthDegOncoming(indexOfOncomingAzimuthsPassed)*pi/180,RminOncoming(indexOfOncomingAzimuthsPassed),'.g');
    if (~isempty(azimuthDegOvertake))
        polarplot(myPolarAxis,azimuthDegOvertake(indexOfOvertakeAzimuthsPassed)*pi/180,RminOvertake(indexOfOvertakeAzimuthsPassed),'.g');
    end
    
    myPolarAxis.ThetaZeroLocation = 'top';
    myPolarAxis.ThetaDir = 'clockwise';
    
    totalPassedOncoming = union(indexOfOncomingAzimuthsPassed,seeAndAvoidOncomingIndex);
    totalPassedOvertake = union(indexOfOvertakeAzimuthsPassed, seeAndAvoidOvertakeIndex);
    totalPassedDetectAndSeeAndAvoid = length(totalPassedOncoming) + length(totalPassedOvertake);

    daaRr = (numberOfAzimuthsEvaluated-numberOfAzimuthsPassed)/numberOfAzimuthsEvaluated;
    dsaaRr = (numberOfAzimuthsEvaluated-totalPassedDetectAndSeeAndAvoid)/numberOfAzimuthsEvaluated;
    %Now formulate a label
    myLabel = sprintf('OwnSpeed:%1.1f kts, IntSpeed:%1.1f kts, FOV:%1.0f, Range:%1.0f, DAA RR:%1.2f, D/SAA RR:%1.2f',OwnSpeed,IntSpeed,FOV,Range, daaRr, dsaaRr);
    title(myLabel);
else
    totalPassedOncoming = union(indexOfOncomingAzimuthsPassed,seeAndAvoidOncomingIndex);
    totalPassedOvertake = union(indexOfOvertakeAzimuthsPassed, seeAndAvoidOvertakeIndex);
    totalPassedDetectAndSeeAndAvoid = length(totalPassedOncoming) + length(totalPassedOvertake);
    daaRr = (numberOfAzimuthsEvaluated-numberOfAzimuthsPassed)/numberOfAzimuthsEvaluated;
    dsaaRr = (numberOfAzimuthsEvaluated-totalPassedDetectAndSeeAndAvoid)/numberOfAzimuthsEvaluated;
end

end

