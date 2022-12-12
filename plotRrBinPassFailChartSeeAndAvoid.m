function plotRrBinPassFailChartSeeAndAvoid(inSpeedVect,inRrVect,inRrVectSAA,daaSpec)
%Plots a stacked bar chart of the Pass/Fail RR normalized so the sum of the bars == the distribution of that traffic speed in the airsapce


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


%%
figure
subplot(2,1,1)  %Plot of each bin pass/fail
bar(inSpeedVect, [failHeight; passHeight; saaHeight]','stacked');
colormap(hot); % Maybe tweak these
myTitleLine1 = sprintf('RR %d kts ownship, %d deg Bank', daaSpec.ownSpeed_kts,daaSpec.maxBank_deg);
myTitleLine2 = sprintf('Including See & Avoid');
title({myTitleLine1 myTitleLine2});
xlabel('Intruder Speed (kts)');
ylabel('Normalized Probability');
legend('Fail','Pass:Detect','Pass:See');
grid on;

%Now let's do a cumulative stacked bar chart
subplot(2,1,2)
passCumSum = cumsum(passHeight);
failCumSum = cumsum(failHeight);
saaCumSum = cumsum(saaHeight);
bar(inSpeedVect, [failCumSum; passCumSum; saaCumSum]','stacked');
colormap(hot); % Maybe tweak these
xlabel('Intruder Speed (kts)');
ylabel('Cumulative Probability')
grid on
hold on
plot([inSpeedVect(1) inSpeedVect(end)],[failCumSum(end) failCumSum(end)],'--k'); % PLot the RR Line

text(inSpeedVect(1),failCumSum(end)+0.1,['RR=' num2str(failCumSum(end),2)]);
hold off





