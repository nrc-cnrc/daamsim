% Copyright (c) 2022 National Research Council Canada
function [pref_man_time, pref_man_turn, common] = conflict_prediction(i, host,intr, time_resol, nz, wind_speed, wind_dir, maxBank, maxRollRate)

coder.extrinsic('avoidSimplified');
                
              pref_man_time=0;
              pref_man_turn=0;
                           
                                         
             % UTM position and velocity of ownship
              host_pos = [host.UTM_E(i);host.UTM_N(i)]; 
              host_posz = host.Alt_m(i);    
              host_vel = [host.Vel_E_mps(i);host.Vel_N_mps(i)]; 
              host_velz =host.Vel_Up_mps(i);

              % UTM position and velocity of intruder
              intr_pos= [intr.UTM_E(i);intr.UTM_N(i)]; 
              intr_posz = intr.Alt_m(i);    
              intr_vel = [intr.Vel_E_mps(i);intr.Vel_N_mps(i)]; 
              intr_velz = intr.Vel_Up_mps(i);

              % relative position and velocity 
              pos_rel = (host_pos-intr_pos); 
              pos_rel_z = (host_posz-intr_posz);
              vel_rel = (host_vel-intr_vel); 
              vel_rel_z = (host_velz-intr_velz);              
              
              
              DMOD=152.4;  
              VDMOD=30.48; 
              risk_bubble=DMOD;
              % horizontal dimension
              ppos = dot(pos_rel,pos_rel);
              vvel = dot(vel_rel,vel_rel);
              posvel = dot(pos_rel,vel_rel);
              Range_xy = sqrt(ppos); % horizontal range
              Rate_xy=posvel/Range_xy;
             if vvel > 0 
                Tcpa = single(-posvel/vvel); % time to closest point of approach
             else 
                Tcpa = single(0);
             end
             
             s_tcpa = pos_rel+Tcpa*vel_rel; % relative horizontal position at closest point of approach
             HMD = sqrt(dot(s_tcpa,s_tcpa)); % horizontal distance at TCPA
             
             %intruder Hor location at Tcpa
             X_i_tcpa=intr.UTM_E(i)+Tcpa*intr.Vel_E_mps(i);
             Y_i_tcpa=intr.UTM_N(i)+Tcpa*intr.Vel_N_mps(i);
             Z_i_tcpa=intr.Alt_m(i)+Tcpa*intr_velz;
             
                   
             
             %Host Hor location at Tcpa
             X_h_tcpa=host.UTM_E(i)+Tcpa*host.Vel_E_mps(i);
             Y_h_tcpa=host.UTM_N(i)+Tcpa*host.Vel_N_mps(i);
             Z_h_tcpa=host.Alt_m(i)+Tcpa*host_velz;
             
             % calculate azimuth with respect to intruder
             speed_hor_h=sqrt(host.Vel_E_mps(i)*host.Vel_E_mps(i)+host.Vel_N_mps(i)*host.Vel_N_mps(i));
             track_h=180/pi*atan2(host.Vel_E_mps(i),host.Vel_N_mps(i));           
             azimuth2_h = 180/pi*atan2((intr.UTM_E(i)-host.UTM_E(i)),(intr.UTM_N(i) - host.UTM_N(i))); %relative to north            
             hostAzimuthTrk = azimuth2_h - track_h;
             speed_hor_i=sqrt(intr.Vel_E_mps(i)*intr.Vel_E_mps(i)+intr.Vel_N_mps(i)*intr.Vel_N_mps(i));
                      
             VMD=Z_h_tcpa-Z_i_tcpa; %Vertical distance at the time of CPA
             
         
                if posvel < 0   % posvel < 0 iff aircraft are converging in horizontal plane
                    Taumod = single((DMOD^2-dot(pos_rel,pos_rel))/dot(pos_rel,vel_rel)); % modified tau
                else 
                    Taumod = single(-1);
                end

                
                 if Taumod < 0
                     
                     Taumod=single(-1);
                     
                 end  
                 
               if (Taumod~=single(-1))
                   
                   risk_bubble=double(Taumod*max(intr.sigma_cross(i), intr.sigma_along(i))+DMOD);
                   if risk_bubble>1000
                       risk_bubble=1000;    
                       
                   end
                   
                   if risk_bubble>HMD
                       
%                        disp("Conflict predicted");
                          
                        vx_w=wind_speed*sin(wind_dir/180*pi);
                        vy_w=wind_speed*cos(wind_dir/180*pi); 
                        
                        [turn_angles, cutoff, t2, t2_miss_dist, azimuth, infov]=avoidSimplified(host.UTM_E(i),...
                            host.UTM_N(i),host.Vel_E_mps(i), host.Vel_N_mps(i),...
                            intr.UTM_E(i),intr.UTM_N(i), intr.Vel_E_mps(i), intr.Vel_N_mps(i), intr.sigma_along,...
                            intr.sigma_cross, time_resol,nz,DMOD, vx_w, vy_w, maxBank, maxRollRate);
                                              
                        pref_man_time=t2(1);
                        pref_man_turn=turn_angles(1);
                        
%                         disp("Preffered maneuver turn degrees");  
%                         disp(pref_man_turn);  
%                         disp("Preffered maneuver time seconds");  
%                         disp(pref_man_time);
                                    
                   end
                   
               end


             common = struct( 'i_X_cpa', X_i_tcpa,'i_Y_cpa', Y_i_tcpa, 'h_X_cpa', X_h_tcpa, 'h_Y_cpa', Y_h_tcpa, 'DMOD', risk_bubble);
        




