% Copyright (c) 2022 National Research Council Canada
function [ac] = Interpolate(acData, timevector)
                      
            acTime=acData.GpsTime_s;
            acLatSamp=acData.Lat_deg;
            acLongSamp=acData.Lon_deg;
            acAltSamp=acData.Alt_m;   
            acTrkSamp=acData.Track_deg; 
            acSpdSamp=acData.GndSpeed_kts;            
            acVzSamp=acData.Vel_Up_mps;
            acVeSamp=acData.Vel_E_mps;
            acVnSamp=acData.Vel_N_mps;
            acXSamp=acData.UTM_E;
            acYSamp=acData.UTM_N;
            
            %next 4 fields are zeros at this point
            acSigma_cross=acData.sigma_cross;
            acSigma_along=acData.sigma_along;
            acX_cpa=acData.X_cpa;
            acY_cpa=acData.Y_cpa;
            
            count=numel(acData.GpsTime_s);
      

            acLatIntrp  = zeros(count,1);
            acLongIntrp  = zeros(count,1);   
            acAltIntrp  = zeros(count,1);
            acTrkIntrp  = zeros(count,1,'single');
            acSpdIntrp  =zeros(count,1,'single');
            acVzIntrp  = zeros(count,1,'single'); 
            acVeIntrp  = zeros(count,1, 'single'); 
            acVnIntrp  = zeros(count,1, 'single');
            acXIntrp  = zeros(count,1); 
            acYIntrp  = zeros(count,1);
            acSigma_cross  = zeros(count,1); 
            acSigma_along  = zeros(count,1);
            acX_cpa  = zeros(count,1); 
            acY_cpa  = zeros(count,1); 
            
            
            acLatIntrp  = interp1(acTime, acLatSamp, timevector , 'linear', 'extrap');
            acLongIntrp  = interp1(acTime, acLongSamp, timevector , 'linear', 'extrap');    
            acAltIntrp  = interp1(acTime, acAltSamp, timevector , 'linear', 'extrap');
            acTrkIntrp  = interp1(acTime, acTrkSamp, timevector , 'linear', 'extrap');  
            acSpdIntrp  = interp1(acTime, acSpdSamp, timevector , 'linear', 'extrap'); 
            acVzIntrp  = interp1(acTime, acVzSamp, timevector , 'linear', 'extrap');  
            acVeIntrp  = interp1(acTime, acVeSamp, timevector , 'linear', 'extrap'); 
            acVnIntrp  = interp1(acTime, acVnSamp, timevector , 'linear', 'extrap'); 
            acXIntrp  = interp1(acTime, acXSamp, timevector , 'linear', 'extrap'); 
            acYIntrp  = interp1(acTime, acYSamp, timevector , 'linear', 'extrap');  
            acSigma_cross  = interp1(acTime, acSigma_cross, timevector , 'linear', 'extrap'); 
            acSigma_along  = interp1(acTime, acSigma_along, timevector , 'linear', 'extrap'); 
            acX_cpa  = interp1(acTime, acX_cpa, timevector , 'linear', 'extrap'); 
            acY_cpa  = interp1(acTime, acY_cpa, timevector , 'linear', 'extrap');
      
       
            ac = struct( 'GpsTime_s', timevector,'Lat_deg', acLatIntrp, 'Lon_deg', acLongIntrp, ...
            'X_cpa', acX_cpa,'UTM_E', acXIntrp, 'UTM_N', acYIntrp,'Alt_m', acAltIntrp,...
            'Vel_E_mps', acVeIntrp, 'Vel_N_mps', acVnIntrp,'Vel_Up_mps', acVzIntrp,...
            'GndSpeed_kts', acSpdIntrp,'Track_deg', acTrkIntrp, 'Y_cpa', acY_cpa,'sigma_cross', acSigma_cross,...
            'sigma_along', acSigma_along);
   
    
         
end	
