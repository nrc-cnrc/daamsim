function [rr]= calculateTotalRRwSeeAndAvoid(inSpeedVect,inRrVect,inRrVectSAA,daaSpec)
%Calculates cumulative RR factoring in See and Avoid


myBins = [5:10:295];

%These values determined based on curve interpolation 
myNormDist = [   0.001405667692905
   0.001405667692905
   0.001405667692905
   0.003987547931244
   0.009674291907117
   0.026095698944210
   0.051832146169262
   0.090691356469433
   0.132072989243198
   0.141405408062189
   0.135374611044511
   0.107715315615124
   0.084529105141218
   0.063002617885127
   0.045538631401297
   0.033246626287672
   0.024041374204839
   0.016159996583123
   0.010047998179908
   0.007391314924999
   0.004326616198941
   0.002667935337829
   0.002126145203992
   0.001746768646695
   0.001405667692905
   0.001405667692905
                   0
                   0
                   0
                   0]';
               
%% If you want to Plot...uncomment this
%Plot results
% figure
% bar(myBins,myNormDist);
% xlabel('Airspeed (kts)')
% ylabel('Prob Density')
% title('Airspace Distribution by Speed (10 kt bins)')
% grid;


%Build the pass/fail #'s
% Memory Allocation
passHeight = zeros(1,length(inSpeedVect));
failHeight = zeros(1,length(inSpeedVect));
saaHeight = zeros(1,length(inSpeedVect)); % this is the height of the see and avoid portion

for i=1:length(inSpeedVect)
    binDist = interp1(myBins,myNormDist,inSpeedVect(i),'nearest');  % This is the overall prob dis of the current speed bin
 
    failHeightNoSee = binDist*inRrVect(i);   % Not counting see and avoid
    passHeightNoSee = binDist - failHeightNoSee;
    
    failHeight(i) = binDist*inRrVectSAA(i);
    saaHeight(i) = binDist - failHeight(i) - passHeightNoSee;
    passHeight(i) = binDist - failHeightNoSee;

end % for



passCumSum = cumsum(passHeight);
failCumSum = cumsum(failHeight);
saaCumSum = cumsum(saaHeight);
rr=failCumSum(end);





