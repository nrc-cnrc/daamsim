%Author: Cyrus Minwalla
%Date: Aug 21, 2014
%Organization: FRL NRC Canada

%Purpose: Load an SRT file in the right format depending on the calling
%function
% Copyright (c) 2022 National Research Council Canada


function srtStream = LoadSRTFile(filename, functionString)
    functor = str2func(functionString);
    srtStream = functor(filename);
    %srtStream = cell2mat(srtStream);    
end