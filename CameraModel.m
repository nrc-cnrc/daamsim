% Copyright (c) 2022 National Research Council Canada
%Developed by Iryna Borshchova 
%This camera model replays real data and reprojects it on the simulated image plane
warning('off','all')
clear all;

% Those are paths to folders with ownship and intruder truth data inside of
% DAAMSIM folder
srtPath='C:\Users\borshchovi\Documents\MATLAB\IrynasCodes\DAAMSIM\167.254.100.2_0008.srt';
intruderdata = readmatrix('C:\Users\borshchovi\Documents\MATLAB\IrynasCodes\DAAMSIM\167.254.100.2_0008_GTIntruder.csv');
cameradata=readmatrix('C:\Users\borshchovi\Documents\MATLAB\IrynasCodes\DAAMSIM\167.254.100.2.csv');

%Those are exterior parameters for each camera; they were hand-tuned, and
%you can tweak them as you please 
rollCam = -1.73;
pitchCam = -3.8; 
headCam = 0.67;	

srtParseType='SRTParse_PICASIntruder';
imageData = LoadSRTFile(srtPath, srtParseType);
a = 6378137;
b = 6356752.3142;
image_w=2448;
image_h=2050;


%Camera interior
 K = [ cameradata(1), 0, cameradata(7);
     0,   cameradata(3),   cameradata(9);
     0, 0, 1.00000000000];

 matrix = [0, 1, 0;
			0, 0, 1;
			1, 0, 0];
        
 N0 = [0;0;1];
 E0 = [0;1;0];
 D0 = [-1;0;0];
        
%Let's count how many data points are there

n = size(imageData,1);

for i=1:1:n-1
           
        
            %Read host location from log
            lat=imageData{i+1,1}.host_lat;
            lon=imageData{i+1,1}.host_lon;
            height = imageData{i+1,1}.host_alt_gps_ft*0.3048; %convert to m
            roll = imageData{i+1,1}.host_roll;
            pitch = imageData{i+1,1}.host_pitch;
            heading =imageData{i+1,1}.host_hdg;
            %Read intruder location from log
            lat_i = intruderdata(i+1);
            lon_i = intruderdata(n+i+1);
            height_i = intruderdata(n*2+i+1)*0.3048;	%convert to m

        
        %Rotate around world axis
      
        R1 = AxisAngleRotation(N0, lon);
        N1 = R1*N0;
        E1 = R1*E0;
        D1 = R1*D0;

        R2 = AxisAngleRotation(-E1, lat);
        N = R2*N1;
        E = E1;
        D = R2*D1;       
             
        
        %Got new origin 
        x0=N;y0=E;z0=D;
        
        %Rotate around body axis

        Rz = AxisAngleRotation(z0, heading);
        x1 = Rz*x0;
        y1 = Rz*y0;
        z1 = z0;

        Ry = AxisAngleRotation(y1, pitch);
        x2 = Ry*x1;
        y2 = y1;
        z2 = Ry*z1;

        Rx = AxisAngleRotation(x2, roll);
        x = x2;
        y = Rx*y2;
        z = Rx*z2;

        %Got new origin
        xCam = x; yCam = y; zCam = z;
        
        %Rotate around camera axis
        
        RzCam = AxisAngleRotation(zCam, headCam);
        xCam1 = RzCam*xCam;
        yCam1 = RzCam*yCam;
        zCam1 = zCam;
        
        RyCam = AxisAngleRotation(yCam1, pitchCam);
        xCam2 = RyCam*xCam1;
        yCam2 = yCam1;
        zCam2 = RyCam*zCam1;
        
        RxCam = AxisAngleRotation(xCam2, rollCam);
        xCam = xCam2;
        yCam = RxCam*yCam2;
        zCam = RxCam*zCam2;
        
        %Transform Host location to ECEF
        ECEF_X = (a/sqrt(cosd(lat)^2+b^2/a^2 * sind(lat)^2) + height)*cosd(lat)*cosd(lon);
        ECEF_Y = (a/sqrt(cosd(lat)^2+b^2/a^2 * sind(lat)^2) + height)*cosd(lat)*sind(lon);
        ECEF_Z = (b/sqrt(cosd(lat)^2*a^2/b^2 + sind(lat)^2) + height)*sind(lat);
        
        r = [ECEF_X; ECEF_Y; ECEF_Z];
        
        %Transform Intruder location to ECEF

        ECEF_X_i = (a/sqrt(cosd(lat_i)^2+b^2/a^2 * sind(lat_i)^2) + height_i)*cosd(lat_i)*cosd(lon_i);
        ECEF_Y_i = (a/sqrt(cosd(lat_i)^2+b^2/a^2 * sind(lat_i)^2) + height_i)*cosd(lat_i)*sind(lon_i);
        ECEF_Z_i = (b/sqrt(cosd(lat_i)^2*a^2/b^2 + sind(lat_i)^2) + height_i)*sind(lat_i);
        
        r_i = [ECEF_X_i; ECEF_Y_i; ECEF_Z_i];
        
        %Find relative location of intruder and host
        rdiff = r_i-r;
        
        relCam=[rdiff'*xCam; rdiff'*yCam; rdiff'*zCam];
        relOpt = matrix*relCam;
        
        %Account for camera interior parameters
        relImg = K*(relOpt);
        
        if (relImg(3)>0)
            
            intruderProj = relImg/abs(relImg(3));        
            boxCx=intruderProj(1);
            boxCy=intruderProj(2); 
            
            if ((boxCx>0)&&(boxCx<image_w)&&(boxCy>0)&&(boxCy<image_h))  
               
                whiteImage = 255 * ones(image_h, image_w, 'uint8');
                fig=imshow(whiteImage);
                title('Simulated camera image and simulated intruder detection');
                hold on
                plot(boxCx,boxCy,'g+', 'MarkerSize', 30);
                hold off      
                pause(0.1) %in seconds
		delete(fig);
            else
                disp('Intruder is not in the FOV of your simulated camera');
            end
            
        end         

         
end




