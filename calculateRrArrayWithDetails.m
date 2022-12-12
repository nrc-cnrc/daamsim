function [myRrVal,azimuth_vect,R_min_m,R_min_m_over,alphaOncomingVect, alphaOvertakeVect,clos_vel,clos_vel_over ]=calculateRrArrayWithDetails(daaSpec, intruderSpeedKts,plotFigs)
%[myRrVal]=calculateRrArray(daaSpec,intruderSpeedKts,plotFigs)
% Structure definition for defining ownship characteristics
% daaSpec.ownSpeed_kts = ownship speed in knots
% daaSpec.ownSize_m = characteristic size...used in see and avoid calcs
% daaSpec.FOV_deg = FOV in degrees
% daaSpec.range_m = range in meters
% daaSpec.rollRateLimit_deg = max roll rate in deg/s
% daaSpec.maxBank_deg = max bank angle in turn
% daaSpec.maxVertRate = rate of climb/descent...for vertical maneuvers (not yet supported)
% Set plotFigs to 0 to suppress plotting
% Input intruder Speed is in knots 
%Authors: Iryna Borshchova, Kris Ellis
%Date: Nov 29, 2021
%Organization: FRL NRC Canada

checkDaaSpecFields(daaSpec); % Check to make sure we have all the struct fields we need

if (plotFigs ~= 0)
    showFigures = true;
else
  showFigures = false;  
end

%animation to show collision avoidance scenario
SHOW_ANIM=0;

% DAA System Specs Here
FOV = daaSpec.FOV_deg; % Deg
Range = daaSpec.range_m; % Range in meters
maxBank = daaSpec.maxBank_deg;
maxRollRate = -1; % As a default
if (isfield(daaSpec,'maxRollRate_deg'))
    maxRollRate = daaSpec.maxRollRate_deg;
end


% Own UAS with the following characteristics
nz = 1/cosd(daaSpec.maxBank_deg);   % 1.5g turn considered reasonable for a UAS
time_resol=0.1; %time resolution for approximation

ground_speed_h_vect = daaSpec.ownSpeed_kts *0.514444 ;  % Convert to m/s
ground_int_speed = intruderSpeedKts*0.514444; %Intruder ground speed from Kts to m/s 
sigma_al=0; %assume perfect position information for now
sigma_cross=0;                 
DMOD=152.4;  %collision bubble              
t_sim=190; 
post_col=5; % 5 sec post-collision
% wind 
wind_speed=0; % in M/s
wind_dir=0; % Is direction wind is coming from
%Rounding alpha
Ndecimals = 2 ;
f = 10.^Ndecimals ;


azimuth_vect =[-179.75:0.25:179.75];
sensor_rate=0; %sec, rate of revisit
scans_track=0; % scans needed to establish the track after detection
t_track=sensor_rate*scans_track; %sec required for a sensor to establish a track on the intruder target
t_warn=0; %sec, to give to a pilot upon execution of CA maneuver ; 0 means automatic
t_delta=t_track+t_warn;
   
% figure for polar plot
if (showFigures)
    figure(1);
    rng = polaraxes;
    figure(2);
    d_hdg = polaraxes;
    
    figure(3);
    closvel = polaraxes;
end %if show figs
% Find wind velocity components
vx_w=wind_speed*sin(wind_dir/180*pi);
vy_w=wind_speed*cos(wind_dir/180*pi); 

R_min_m = NaN(1, length(azimuth_vect));
R_min_m_over = NaN(1, length(azimuth_vect));
%loop through UAV speeds

alphaOncomingVect = zeros(1,length(azimuth_vect)); %mem assignment
alphaOvertakeVect = zeros(1,length(azimuth_vect)); %mem assignment
for ii=1:length(ground_speed_h_vect)
    
    ground_speed_h = ground_speed_h_vect(ii);
     
% Loop through each azimuth:    
    
    for k=1:length(azimuth_vect)  

        % initialize arrays
        arraySize = (t_sim+post_col)/time_resol;
        y_h = zeros(1, arraySize);
        y_i = zeros(1, arraySize);
        x_h = zeros(1, arraySize);
        x_i = zeros(1, arraySize);
        
       
        vy_h = zeros(1, arraySize);
        vy_i = zeros(1, arraySize);
        vx_h = zeros(1, arraySize);
        vx_i = zeros(1, arraySize);
        bankAngle = zeros(1, arraySize); % Added Dec 8 2021 For roll rate limiting
     
                
        % calculate the collision Alpha using intruder and ownship ground
        % speeds

        [alpha]=round(calculate_alpha(ground_speed_h, ground_int_speed, azimuth_vect(k))*f)/f; %% Some rounding done here to prevent super small values near 0/180
        alphaOncomingVect(k) = alpha;
    if (alpha ~=0 &&  alpha ~=180 &&  alpha ~=-180)
            
            %calculate the Beta angle (wrap to 180)
            beta = 180-alpha;
            if (beta) > 180
               beta = beta - 360;
            elseif beta < -180
               beta = beta + 360;
            end
            psi_i= mod((360-alpha)*pi/180, 2*pi);   % intruder heading in rads
            psi_h = 0;      % ownship heading north initially; 
            beta_rad = beta*pi/180;
            % origin at Tcpa, ownship heading north for simplicity
                
            % let's calculate ownship initial conditions 
            %   i) positions (earth referenced)
            %  ii) Velocities (earth referenced)
            vx_h(1)=ground_speed_h*sin(psi_h);  
            vy_h(1)=ground_speed_h*cos(psi_h);              
            x_h(1) = 0;
            y_h(1) = -ground_speed_h*t_sim;        
            bankAngle(1) = 0; % Not turning yet....
      
            % let's calculate intruder initial conditions; 
            % have ground speed info
            %   i) positions (earth referenced)
            %  ii) Velocities (earth referenced)
            
            %offset from CPA
            vx_i(1)=ground_int_speed*sin(psi_i);   
            vy_i(1)=ground_int_speed*cos(psi_i);
            x_i(1) = ground_int_speed*t_sim*sin(beta_rad); 
            y_i(1) = ground_int_speed*t_sim*cos(beta_rad);          
           
         
            % calculate avoidance options 
            [turn_angles, cutoff, t2, t2_miss_dist, azimuth, infov]=avoidSimplified(x_h(1), y_h(1),vx_h(1), vy_h(1),...
                       x_i(1),y_i(1), vx_i(1), vy_i(1), sigma_al,...                      
                       sigma_cross, time_resol,nz,DMOD, vx_w, vy_w, maxBank, maxRollRate);        
           % pref manuevre turn and time is the first element in the array
                  pref_man_time=t2(1);
            pref_man_turn=turn_angles(1); 
 

               % the next block is needed to establish minimim range
                % requirements
                host_vel = [vx_h(1);vy_h(1)]; 
                intr_vel = [vx_i(1);vy_i(1)]; 
                vel_rel = (host_vel-intr_vel);
                host_pos = [x_h(1);y_h(1)]; 
                intr_pos = [x_i(1);y_i(1)];          
                pos_rel = (host_pos-intr_pos); 
                vvel = dot(vel_rel,vel_rel);
                posvel = dot(pos_rel,vel_rel);

                if vvel > 0 
                   Tcpa = -posvel/vvel; % time to closest point of approach
                else 
                   Tcpa= 0;
                end
                
                posvel = dot(pos_rel,vel_rel);
                if posvel < 0   % posvel < 0 iff aircraft are converging in horizontal plane
                    Taumod = (DMOD^2-dot(pos_rel,pos_rel))/dot(pos_rel,vel_rel); % modified tau
                else 
                    Taumod = -1;
                end

                
                 if Taumod < 0
                     
                     Taumod=-1;
                     
                 end            

                tm(k)=Taumod-pref_man_time;
       
                % calculate min sensor range
                R_min_m(k)=tm(k)*sqrt(vvel); 
                clos_vel(k)=sqrt(vvel);    
                
                
                if pref_man_turn>=0
                    Delta_hdg_r(k)=pref_man_turn;
                    azim_r(k)=azimuth_vect(k);
                    Delta_hdg_l(k)=NaN;
                    azim_l(k)=NaN;
                    
                else
                    Delta_hdg_l(k)=pref_man_turn;
                    azim_l(k)=azimuth_vect(k);
                    Delta_hdg_r(k)=NaN;
                    azim_r(k)=NaN;
                end
                %those are needed to plot a mesh
                
                    
              
                
                % calculate avoidance turns with wind

                for time=2:(t_sim+post_col)/time_resol 
                % this block is for the straight line before the turn
                     if(time<(pref_man_time)/time_resol)

                            x_h(time)=x_h(time-1) + vx_h(time-1)*time_resol;
                            y_h(time)=y_h(time-1) + vy_h(time-1)*time_resol;
                            vx_h(time)=vx_h(time-1);
                            vy_h(time)=vy_h(time-1); 
                            %update intruder info
                            y_i(time) = y_i(time-1) +ground_int_speed*cos(psi_i)*time_resol;
                            x_i(time) = x_i(time-1) + ground_int_speed*sin(psi_i)*time_resol;
                            bankAngle(time) = 0; % Still not turning

                     else
                          %calculate track to know when to stop turning

                          track=wrapTo180(atan2(vx_h(time-1),vy_h(time-1))*180/pi);           

                         if (abs(track)<abs(wrapTo180(pref_man_turn+psi_h)))                          

                            g=9.80665;
                            air_speed_h=sqrt((vx_h(time-1)+vx_w)*(vx_h(time-1)+vx_w)+(vy_h(time-1)+vy_w)*(vy_h(time-1)+vy_w));
                            % KE Made some mods here Dec 8 2021 to allow
                            % for a rate of bank to be specified
                            %
                            % Check to see if we have a max roll rate in
                            % our daaSped
                            if (isfield(daaSpec,'maxRollRate_deg'))
                               % Here we have a roll rate to work with
                               bankAngle(time) = bankAngle(time-1)+daaSpec.maxRollRate_deg*time_resol; % calculate the bank angle increment
                               if (bankAngle(time)>daaSpec.maxBank_deg)
                                  bankAngle(time)=daaSpec.maxBank_deg; % Limit to the max bank angle
                               end
                               R = air_speed_h*air_speed_h/(g*tand(bankAngle(time)));
                            else
                                R = air_speed_h*air_speed_h/g/sqrt(nz*nz-1.0);
                            end
                            omega = -sign(pref_man_turn)*air_speed_h/R;                            

                            %use coordinated turn model
                            x_h(time)=x_h(time-1) + time_resol*vx_h(time-1)-omega*time_resol*time_resol/2*vy_h(time-1);        
                            vx_h(time)=vx_h(time-1)*(1-omega*time_resol*omega*time_resol/2) -omega*time_resol*vy_h(time-1);        
                            y_h(time)=vx_h(time-1)*omega*time_resol*time_resol/2+y_h(time-1)+time_resol*vy_h(time-1);
                            vy_h(time)=vx_h(time-1)*omega*time_resol+(1-omega*time_resol*omega*time_resol/2)*vy_h(time-1);
                            %update intruder info
                            y_i(time) = y_i(time-1) +ground_int_speed*cos(psi_i)*time_resol;
                            x_i(time) = x_i(time-1) + ground_int_speed*sin(psi_i)*time_resol;


                          else  %end of turn; back to straight line
                              
                            % KE Notes that this is where we would need to
                            % add another turn model to return to wings
                            % level....job for another day
                            %
                            
                            
                            vx_h(time)=vx_h(time-1);
                            vy_h(time)=vy_h(time-1);
                            x_h(time)=x_h(time-1) + vx_h(time-1)*time_resol;
                            y_h(time)=y_h(time-1) + vy_h(time-1)*time_resol;
                            %update intruder info
                            y_i(time) = y_i(time-1) +ground_int_speed*cos(psi_i)*time_resol;
                            x_i(time) = x_i(time-1) + ground_int_speed*sin(psi_i)*time_resol; 


                          end   

                      end
                end
                
                if (SHOW_ANIM)
            % plot avoidance trajectories
            figure()
            for i=1:10:length(x_i)               
                    clf
                    plot(x_i,y_i,':r',x_h,y_h,':g');
                    hold on
                    circle2(x_i(i),y_i(i), DMOD);
                    hold on
                    plot(x_i(i),y_i(i),'rd','MarkerFaceColor','r');
                    hold on
                    plot(x_h(i),y_h(i),'gd', 'MarkerFaceColor','g');
                    hold on                 

                    
                    %plot sensor FOV
                    track=atan2(vy_h(i),vx_h(i));
                    plot_fov(track-FOV*pi/180/2,track+FOV*pi/180/2,x_h(i),y_h(i),R_min_m(k));                    


                    xlabel('East Pos (m)')
                    ylabel('North Pos (m)')
                    title(['Collision Avoidance Animation, Azimuth ' num2str(azimuth_vect(k)) ', Own speed ' num2str(ground_speed_h) ' (mps)'])
                    axis equal
                    grid on
                    pause(0.05);	
                    hold off                  
            end  
            
        end %end for animation

        else  
                     R_min_m(k)=NaN;
                     tm(k)=NaN;
                     Delta_hdg_r(k)=NaN; 
                     azim_r(k)=NaN;
                     Delta_hdg_l(k)=NaN; 
                     azim_l(k)=NaN;
                     az(ii,k)=NaN;
                    clos_vel(k)=NaN;
               

      end % end for alpha      
      
      
      
   
    % calculate the collision Alpha using intruder and ownship ground
        % speeds  for overtake

   
[alpha]=round(calculate_alpha_ov(ground_speed_h, ground_int_speed, azimuth_vect(k))*f)/f;
 alphaOvertakeVect(k) =alpha;
 if (alpha ~=0 &&  alpha ~=180 &&  alpha ~=-180)
            
            %calculate the Beta angle (wrap to 180)
            beta = 180-alpha;
            if (beta) > 180
               beta = beta - 360;
            elseif beta < -180
               beta = beta + 360;
            end
            psi_i= mod((360-alpha)*pi/180, 2*pi);   % intruder heading in rads
            psi_h = 0;      % ownship heading north initially; 
            beta_rad = beta*pi/180;
            % origin at Tcpa, ownship heading north for simplicity
                
            % let's calculate ownship initial conditions 
            %   i) positions (earth referenced)
            %  ii) Velocities (earth referenced)
            vx_h(1)=ground_speed_h*sin(psi_h);  
            vy_h(1)=ground_speed_h*cos(psi_h);              
            x_h(1) = 0;
            y_h(1) = -ground_speed_h*t_sim;        
           
      
            % let's calculate intruder initial conditions; 
            % have ground speed info
            %   i) positions (earth referenced)
            %  ii) Velocities (earth referenced)
            
            %offset from CPA
            vx_i(1)=ground_int_speed*sin(psi_i);   
            vy_i(1)=ground_int_speed*cos(psi_i);
            x_i(1) = ground_int_speed*t_sim*sin(beta_rad); 
            y_i(1) = ground_int_speed*t_sim*cos(beta_rad);          
           
         
            % calculate avoidance options 
            [turn_angles, cutoff, t2, t2_miss_dist, azimuth, infov]=avoidSimplified(x_h(1), y_h(1),vx_h(1), vy_h(1),...
                       x_i(1),y_i(1), vx_i(1), vy_i(1), sigma_al,...
                       sigma_cross, time_resol,nz,DMOD, vx_w, vy_w,maxBank, maxRollRate);        
           % pref manuevre turn and time is the first element in the array
                  pref_man_time=t2(1);
            pref_man_turn=turn_angles(1); 
 

               % the next block is needed to establish minimim range
                % requirements
                host_vel = [vx_h(1);vy_h(1)]; 
                intr_vel = [vx_i(1);vy_i(1)]; 
                vel_rel = (host_vel-intr_vel);
                host_pos = [x_h(1);y_h(1)]; 
                intr_pos = [x_i(1);y_i(1)];          
                pos_rel = (host_pos-intr_pos); 
                vvel = dot(vel_rel,vel_rel);
                posvel = dot(pos_rel,vel_rel);

                if vvel > 0 
                   Tcpa = -posvel/vvel; % time to closest point of approach
                else 
                   Tcpa= 0;
                end
                
                posvel = dot(pos_rel,vel_rel);
                if posvel < 0   % posvel < 0 iff aircraft are converging in horizontal plane
                    Taumod = (DMOD^2-dot(pos_rel,pos_rel))/dot(pos_rel,vel_rel); % modified tau
                else 
                    Taumod = -1;
                end

                
                 if Taumod < 0
                     
                     Taumod=-1;
                     
                 end            

                tm(k)=Taumod-pref_man_time;

              
                % calculate min sensor range
                R_min_m_over(k)=tm(k)*sqrt(vvel);  
                clos_vel_over(k)=sqrt(vvel);     
                
                
                if pref_man_turn>=0
                    Delta_hdg_r(k)=pref_man_turn;
                    azim_r(k)=azimuth_vect(k);
                    Delta_hdg_l(k)=NaN;
                    azim_l(k)=NaN;
                    
                else
                    Delta_hdg_l(k)=pref_man_turn;
                    azim_l(k)=azimuth_vect(k);
                    Delta_hdg_r(k)=NaN;
                    azim_r(k)=NaN;
                end
                %those are needed to plot a mesh
                
                   
               
                
                % calculate avoidance turns with wind

                for time=2:(t_sim+post_col)/time_resol 
                % this block is for the straight line before the turn
                     if(time<(pref_man_time)/time_resol)

                            x_h(time)=x_h(time-1) + vx_h(time-1)*time_resol;
                            y_h(time)=y_h(time-1) + vy_h(time-1)*time_resol;
                            vx_h(time)=vx_h(time-1);
                            vy_h(time)=vy_h(time-1); 
                            %update intruder info
                            y_i(time) = y_i(time-1) +ground_int_speed*cos(psi_i)*time_resol;
                            x_i(time) = x_i(time-1) + ground_int_speed*sin(psi_i)*time_resol;

                     else
                          %calculate track to know when to stop turning

                          track=wrapTo180(atan2(vx_h(time-1),vy_h(time-1))*180/pi);           

                         if (abs(track)<abs(wrapTo180(pref_man_turn+psi_h)))                          

                            g=9.80665;
                            air_speed_h=sqrt((vx_h(time-1)+vx_w)*(vx_h(time-1)+vx_w)+(vy_h(time-1)+vy_w)*(vy_h(time-1)+vy_w)); 
                                                        % KE Made some mods here Dec 8 2021 to allow
                            % for a rate of bank to be specified
                            %
                            % Check to see if we have a max roll rate in
                            % our daaSped
                            if (isfield(daaSpec,'maxRollRate_deg'))
                               % Here we have a roll rate to work with
                               bankAngle(time) = bankAngle(time-1)+daaSpec.maxRollRate_deg*time_resol; % calculate the bank angle increment
                               if (bankAngle(time)>daaSpec.maxBank_deg)
                                  bankAngle(time)=daaSpec.maxBank_deg; % Limit to the max bank angle
                               end
                               R = air_speed_h*air_speed_h/(g*tand(bankAngle(time)));
                            else
                                R = air_speed_h*air_speed_h/g/sqrt(nz*nz-1.0);
                            end  
                            omega = -sign(pref_man_turn)*air_speed_h/R;                            

                            %use coordinated turn model
                            x_h(time)=x_h(time-1) + time_resol*vx_h(time-1)-omega*time_resol*time_resol/2*vy_h(time-1);        
                            vx_h(time)=vx_h(time-1)*(1-omega*time_resol*omega*time_resol/2) -omega*time_resol*vy_h(time-1);        
                            y_h(time)=vx_h(time-1)*omega*time_resol*time_resol/2+y_h(time-1)+time_resol*vy_h(time-1);
                            vy_h(time)=vx_h(time-1)*omega*time_resol+(1-omega*time_resol*omega*time_resol/2)*vy_h(time-1);
                            %update intruder info
                            y_i(time) = y_i(time-1) +ground_int_speed*cos(psi_i)*time_resol;
                            x_i(time) = x_i(time-1) + ground_int_speed*sin(psi_i)*time_resol;


                          else  %end of turn; back to straight line                           
                            vx_h(time)=vx_h(time-1);
                            vy_h(time)=vy_h(time-1);
                            x_h(time)=x_h(time-1) + vx_h(time-1)*time_resol;
                            y_h(time)=y_h(time-1) + vy_h(time-1)*time_resol;
                            %update intruder info
                            y_i(time) = y_i(time-1) +ground_int_speed*cos(psi_i)*time_resol;
                            x_i(time) = x_i(time-1) + ground_int_speed*sin(psi_i)*time_resol; 


                          end   

                      end
                end
                
                if (SHOW_ANIM)
            % plot avoidance trajectories
            figure()
            for i=1:10:length(x_i)               
                    clf
                    plot(x_i,y_i,':r',x_h,y_h,':g');
                    hold on
                    circle2(x_i(i),y_i(i), DMOD);
                    hold on
                    plot(x_i(i),y_i(i),'rd','MarkerFaceColor','r');
                    hold on
                    plot(x_h(i),y_h(i),'gd', 'MarkerFaceColor','g');
                    hold on                 

                    
                    %plot sensor FOV
                    track=atan2(vy_h(i),vx_h(i));
                    plot_fov(track-FOV*pi/180/2,track+FOV*pi/180/2,x_h(i),y_h(i),R_min_m(k));                    


                    xlabel('East Pos (m)')
                    ylabel('North Pos (m)')
                    title(['Collision Avoidance Animation, Azimuth ' num2str(azimuth_vect(k)) ', Own speed ' num2str(ground_speed_h) ' (mps)'])
                    axis equal
                    grid on
                    pause(0.05);	
                    hold off                  
            end  
            
        end %end for animation

        else  
                     R_min_m_over(k)=NaN;
                     tm(k)=NaN;
                     Delta_hdg_r(k)=NaN; 
                     azim_r(k)=NaN;
                     Delta_hdg_l(k)=NaN; 
                     azim_l(k)=NaN;
                     az(ii,k)=NaN;
            
                     clos_vel_over(k)=NaN;
              

      end % end for alpha  
      
         end 	%for azimuth
if (showFigures)
    polarplot(d_hdg,azim_r'*pi/180, Delta_hdg_r, 'LineWidth', 0.5, 'Color', 'g');
    d_hdg.ThetaDir = 'clockwise';
    d_hdg.ThetaZeroLocation = 'top';
    grid on
    hold (d_hdg, 'on')
    
    polarplot(d_hdg,azim_l'*pi/180, abs(Delta_hdg_l), 'LineWidth', 0.5, 'Color', 'r');
    d_hdg.ThetaDir = 'clockwise';
    d_hdg.ThetaZeroLocation = 'top';
    grid on
    hold (d_hdg, 'on')
    
    
    polarplot(rng,azimuth_vect'*pi/180, R_min_m'/1000, 'LineWidth', 0.5);
    rng.ThetaDir = 'clockwise';
    rng.ThetaZeroLocation = 'top';
    grid on
    hold (rng, 'on')
    
    polarplot(rng,azimuth_vect'*pi/180, R_min_m_over'/1000, 'LineWidth', 0.5);
    rng.ThetaDir = 'clockwise';
    rng.ThetaZeroLocation = 'top';
    grid on
    hold (rng, 'on')
    
    polarplot(closvel,azimuth_vect'*pi/180, clos_vel', 'LineWidth', 0.5);
    closvel.ThetaDir = 'clockwise';
    closvel.ThetaZeroLocation = 'top';
    grid on
    hold (closvel, 'on')
    
    polarplot(closvel,azimuth_vect'*pi/180, clos_vel_over', 'LineWidth', 0.5);
    closvel.ThetaDir = 'clockwise';
    closvel.ThetaZeroLocation = 'top';
    grid on
    hold (closvel, 'on')
end %if show figs

end% for loop ground speed vector	

if (showFigures)
    legend(d_hdg,'Right turn','Left turn', 'Location', 'northeastoutside');
    title(d_hdg,['Delta Hdg (deg), Load Factor ' num2str(nz) ', Intruder Speed ' num2str(ground_int_speed) ' (mps)'])
    hold (d_hdg, 'off')
    %
    legend(rng,[num2str(ground_speed_h_vect),repmat(' mps',[length(ground_speed_h_vect'),1])], 'Location', 'northeastoutside');
    title(rng,['Rmin (km), Load Factor ' num2str(nz) ', Intruder Speed ' num2str(ground_int_speed) ' (mps)'])
    hold (rng, 'off')
    
    legend(closvel,[num2str(ground_speed_h_vect),repmat(' mps',[length(ground_speed_h_vect'),1])], 'Location', 'northeastoutside');
    title(closvel,['Closing Vel(m/s), Load Factor ' num2str(nz) ', Intruder Speed ' num2str(ground_int_speed) ' (mps)'])
    hold (closvel, 'off')
end %if show figs

myRrVal = plotDaaRrVsAzimuth(azimuth_vect,R_min_m,azimuth_vect,R_min_m_over,daaSpec,ground_int_speed,1); % 0 means don't plot

end




function h = circle2(x,y,r)

d = r*2;
px = x-r;
py = y-r;
h = rectangle('Position',[px py d d],'Curvature',[1,1], 'EdgeColor', 'r');
% daspect([1,1,1])

end


  




