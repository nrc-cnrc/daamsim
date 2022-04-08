function [kappaDeg ] = calculate_kappa(azimuthDeg, alphaDeg)
% Copyright (c) 2022 National Research Council Canada

    
    if (azimuthDeg >= 0) 
        kappaDeg = 180 - azimuthDeg - alphaDeg; % All angles in a triangle sum to 180 deg
    else 
        kappaDeg = -180 -azimuthDeg -alphaDeg;  % Kappa should be negative for a negative azimuth
    end
       

end

