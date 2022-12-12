function [daaRr]= overplot3DaaRrVsAzimuth(azimuthDegOncoming,RminOncoming,azimuthDegOvertake, RminOvertake,FOV,Range, OwnSpeed,IntSpeed,showPlot,mainPlotHandle)
%[daaRr]= plotDaaRrVsAzimuth(azimuthDegOncoming,RminOncoming,azimuthDegOvertake, RminOvertake,FOV,Range, OwnSpeed,IntSpeed,showPlot)
%Units should be in meters...that's what my labels say!....make showPlot 0 to not show plots
% NOTE: We need something passed for overtake cases. They can be empty vectors [] for cases where no overtakes are possible
%Does a polar plot for a given Rmin azimuth combination
% NOTE2: This code curently assumes a fixed range over the whole FOV...it's Iryna's job to convert this to make it variable range over the FOV
% NOTE3: This code also assumes a fixed azimuth interval for both oncoming and overtaking...don't change this, or the maths won't work


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
end % if we have overtakes


%Now we plot the results
if (showPlot ~= 0)
    polarFigure = figure;
    myPolarAxis = polaraxes;
    
    polarplot(myPolarAxis,azimuthDegOncoming*pi/180,RminOncoming,'.r');
    hold on
    %Now plot the DAA system range/fov in black --
    polarplot(myPolarAxis,[0 daaSystemAzimuth*pi/180 0],[0; daaRange; 0],'--k');
    %Now overplot the passed cases in green
    polarplot(myPolarAxis,azimuthDegOncoming(indexOfOncomingAzimuthsPassed)*pi/180,RminOncoming(indexOfOncomingAzimuthsPassed),'.g');
    %Now do the 3d cartesian plot
    
    [XonComing,YonComing]=pol2cart(azimuthDegOncoming*pi/180,RminOncoming);
    [XonComingPassed,YonComingPassed]=pol2cart(azimuthDegOncoming(indexOfOncomingAzimuthsPassed)*pi/180,RminOncoming(indexOfOncomingAzimuthsPassed));
    speedVect = IntSpeed * ones(1,length(azimuthDegOncoming)); % Make a vector of speeds (isobar on the plot)
    
    figure(mainPlotHandle); % Go back to the main plot and update it
    plot3(XonComing,YonComing,speedVect,'-r'); % Default to fail, and overwrite in green for pass
    hold on
    plot3(XonComingPassed,YonComingPassed,speedVect(indexOfOncomingAzimuthsPassed),'.g');
    
    if (~isempty(azimuthDegOvertake))
        figure(polarFigure); % Back to the polar plot
        polarplot(myPolarAxis,azimuthDegOvertake*pi/180,RminOvertake,'.r');
        %Now overplot the passed cases in green
        polarplot(myPolarAxis,azimuthDegOvertake(indexOfOvertakeAzimuthsPassed)*pi/180,RminOvertake(indexOfOvertakeAzimuthsPassed),'.g');
        figure(mainPlotHandle); % Go back to the main plot and update it
        [XovertakePassed,YovertakePassed]=pol2cart(azimuthDegOvertake(indexOfOvertakeAzimuthsPassed)*pi/180,RminOvertake(indexOfOvertakeAzimuthsPassed));
        [XovertakeAll,YovertakeAll]=pol2cart(azimuthDegOvertake*pi/180,RminOvertake);
        speedVect = IntSpeed * ones(1,length(azimuthDegOvertake)); % Make a vector of speeds (isobar on the plot)
        plot3(XovertakeAll,YovertakeAll,speedVect,'-r');
        hold on
        plot3(XovertakePassed,YovertakePassed,speedVect(indexOfOvertakeAzimuthsPassed),'.g');
        
    end
    myPolarAxis.ThetaZeroLocation = 'top';
    myPolarAxis.ThetaDir = 'clockwise';
    
    figure(polarFigure); % Back to the polar plot
    daaRr = (numberOfAzimuthsEvaluated-numberOfAzimuthsPassed)/numberOfAzimuthsEvaluated;
    %Now formulate a label
    myLabel = sprintf('OwnSpeed = %1.1f, IntSpeed = %1.1f, FOV = %1.0f, Range = %1.0f, Risk Ratio = %1.2f',OwnSpeed,IntSpeed,FOV,Range, daaRr);
    title(myLabel);
else % Don't plot
    [XonComing,YonComing]=pol2cart(azimuthDegOncoming*pi/180,RminOncoming);
    [XonComingPassed,YonComingPassed]=pol2cart(azimuthDegOncoming(indexOfOncomingAzimuthsPassed)*pi/180,RminOncoming(indexOfOncomingAzimuthsPassed));
    speedVect = IntSpeed * ones(1,length(azimuthDegOncoming)); % Make a vector of speeds (isobar on the plot)
    
    figure(mainPlotHandle); % Go back to the main plot and update it
    plot3(XonComing,YonComing,speedVect,'-r'); % Default to fail, and overwrite in green for pass
    hold on
    plot3(XonComingPassed,YonComingPassed,speedVect(indexOfOncomingAzimuthsPassed),'.g');
    
    if (~isempty(azimuthDegOvertake))
        [XovertakePassed,YovertakePassed]=pol2cart(azimuthDegOvertake(indexOfOvertakeAzimuthsPassed)*pi/180,RminOvertake(indexOfOvertakeAzimuthsPassed));
        [XovertakeAll,YovertakeAll]=pol2cart(azimuthDegOvertake*pi/180,RminOvertake);
        speedVect = IntSpeed * ones(1,length(azimuthDegOvertake)); % Make a vector of speeds (isobar on the plot)
        plot3(XovertakeAll,YovertakeAll,speedVect,'-r');
        hold on
        plot3(XovertakePassed,YovertakePassed,speedVect(indexOfOvertakeAzimuthsPassed),'.g');
        
    end % if
    daaRr = (numberOfAzimuthsEvaluated-numberOfAzimuthsPassed)/numberOfAzimuthsEvaluated;
end

end

