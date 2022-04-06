clear all;
load('Intercept112.mat','Intruder');


Intruder = rmfield(Intruder, 'registration');
Intruder = rmfield(Intruder, 'type');
Intruder = rmfield(Intruder, 'Pitch_deg');
Intruder = rmfield(Intruder, 'UTM_Zone');
Intruder = rnfield(Intruder, 'Alt_ft', 'X_cpa');
Intruder = rnfield(Intruder, 'FlightPath_deg', 'Y_cpa');
Intruder=rnfield(Intruder, 'Heading_deg', 'sigma_along');
Intruder=rnfield(Intruder, 'Roll_deg', 'sigma_cross');

count=numel(Intruder.GpsTime_s); 
Intruder.sigma_cross=zeros(count,1,'double');
Intruder.sigma_along=zeros(count,1,'double');
Intruder.X_cpa=zeros(count,1,'double');
Intruder.Y_cpa=zeros(count,1,'double');


load('Intercept112.mat','Host');

Host = rmfield(Host, 'registration');
Host = rmfield(Host, 'type');
Host = rmfield(Host, 'Pitch_deg');
Host = rmfield(Host, 'UTM_Zone');
Host = rnfield(Host, 'Alt_ft','X_cpa');
Host = rnfield(Host, 'FlightPath_deg','Y_cpa');
Host=rnfield(Host, 'Heading_deg', 'sigma_along');
Host=rnfield(Host, 'Roll_deg', 'sigma_cross');

count=numel(Host.GpsTime_s); 
Host.sigma_cross=zeros(count,1,'double');
Host.sigma_along=zeros(count,1,'double');
Host.X_cpa=zeros(count,1,'double');
Host.Y_cpa=zeros(count,1,'double');
Host.Alt_m=zeros(count,1,'double');

