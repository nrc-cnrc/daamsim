function [detectionRange,maxClosingVelForSeeAndAvoid] = calculateSeeAndAvoidForRpaSizeAndIntruderManeuverDelay(rpaSizeMeters,detectionThresholdArcMin,maneuverDelaySecs)
%calculateSeeAndAvoidFor...(rpaSize (meters),detectionThreshold (arc minutes), maneuverDelay (seconds))
%   Calculates the range at which the RPA should be detected by a human pilot
%  
% When choosing detection threshold and maneuver delay consider the following
%
% Here's some relevant info from an Embry Riddle thesis on the topic of visual detection of RPA's
%While these models are more comprehensive and complex then needed for the purpose of this study, 
%the overall results from these six models are useful. This study focuses on the limitations 
%imposed by human vision since the detection of small UA encroaches on those limits. 
%While each model presented by Greening produced its own unique probability of detection
%or probability of recognition curve, the summarized results from the various models are: 
%1. Targets with visual angles less than one arc minute are unlikely to be seen; 
%2. Targets with visual angles greater than 10 arc minutes are likely to be detected (but not necessarily recognized) (Greening, 1976, p. 139); 
%3. Targets become recognizable between 30% to 40% of the time when they render a visual angle of 15 arc minutes or more; and 
%4. In four of the six models, targets become recognizable 50% to 100% of the time when the visual angle exceeds 30 arc minutes (Greening, 1976, p. 140).
%
% You might assume that 30 arc seconds is the right number to choose...but 
% there is likely also some buffer built into the manauver time delay
% Typically you'd choose something like 12.5 seconds per AC-9048D
% https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_90-48D.pdf
% This breaks the time delay sequence down to:
%   See object 0.1 seconds
%   Recognize aircraft 1.0 seconds
%   Become aware of collision course 5.0 seconds
%   Decision to turn left or right 4.0 seconds
%   Muscular reaction 0.4 seconds
%   Aircraft lag time 2.0 seconds

detectionThreshDeg = detectionThresholdArcMin/60;

detectionRange = rpaSizeMeters/2*cotd(detectionThreshDeg/2);
maxClosingVelForSeeAndAvoid = detectionRange/maneuverDelaySecs;

end

