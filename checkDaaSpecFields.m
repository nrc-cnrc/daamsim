function checkDaaSpecFields(daaSpec)
% Structure definition for defining ownship characteristics
% daaSpec.ownSpeed_kts = ownship speed in knots
% daaSpec.ownSize_m = characteristic size...used in see and avoid calcs
% daaSpec.FOV_deg = FOV in degrees
% daaSpec.range_m = range in meters
% daaSpec.rollRateLimit_deg = max roll rate in deg/s (not yet supported)
% daaSpec.maxBank_deg = max bank angle in turn
% daaSpec.maxVertRate = rate of climb/descent...for vertical maneuvers (not yet supported)

if(~isfield(daaSpec,{'ownSpeed_kts', 'ownSize_m', 'FOV_deg', 'range_m', 'maxBank_deg'} ))
   fprintf(1,'\nThe following fields are required for the daaSpec struct:\n');
   fprintf(1,'ownSpeed_kts = ownship speed in knots\n');
   fprintf(1,'ownSize_m = characteristic size...used in see and avoid calcs\n');
   fprintf(1,'FOV_deg = FOV in degrees\n');
   fprintf(1,'range_m = range in meters\n');
   fprintf(1,'maxBank_deg = max bank angle in turn\n');
   error('The daaSpec struct is missing necessary fields. A list of required fields is above.');
end

