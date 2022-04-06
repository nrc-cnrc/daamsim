function [ out ] = SRT_PARSE( file, DBG )
%SRT_PARSE: extract aircraft data from srt file
%   Detailed explanation goes here

    %Open File for Parsing
    fid = fopen(file, 'rb');
    
    %Set Not Debug if No Debug Variabl is Passed
    if ~exist('DBG');
        DBG = 0;
    end
    
    %File Does Not Exist?
    if(fid < 0)
        fprintf(2, 'ERROR: The File "%s" Does Not Exist!\n\n', file);
        return;
    end
    
    out = cell(0,0);
    
    %Initial State
    STATE = 0;
    
    %Main Loop Parses File.
    line = '';
    tst = [1,1];
    
    while STATE ~= -1
        %Set Loop Condition
        tst = [];
        
        %Read Lines Until Line With Writing is Found End if EOF
        while length(tst) == 0
            line = fgets(fid);
            if feof(fid)
                STATE = -1;
                break;
            end
            tst = regexp(line, '[a-zA-Z0-9]');
        end
        
        %Parse Current Line
        switch STATE
            case -1
                break;
            case 0
                %Make Sure This is a Frame Number
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                if isnan(str2double(line))
                    fprintf(2, 'ERROR: Incorrect Format of Input File %s\n\n', file);
                    return;
                end
                
                %Create Entry for new Frame
                out = [out; cell(1,1)];
                out{end} = struct('host_gpstime', 0, 'cam_gpstime', 0, 'frame_gpstime', 0, 'index', 0, ...
					'elapsed_time',...
                    0, 'host_roll', 0, 'host_pitch', 0, 'host_hdg', 0, 'cam_roll', 0,...
                    'cam_pitch', 0, 'cam_hdg', 0, 'host_alt_gps_ft', 0, 'host_alt_agl_ft', 0,...
					'host_lat', 0, 'host_lon', 0, 'cam_lat', 0, 'cam_lon', 0, 'cam_alt_gps_ft', 0,...
                    'cam_p', 0, 'cam_q', 0, 'cam_r', 0, 'cam_acc_x', 0,'cam_acc_y', 0, 'cam_acc_z',0 );
				nums = regexp(line, '[-]?\d\d*+\.?\d*');                
				out{end} = struct('frameIndex', str2double(strtok(line(nums(1):end), ' ')), 'gps_time', 0);
                STATE = 1;
            case 1
                %Ignore Time Stamp (For Now)
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                STATE = 2;
            case 2
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                nums = regexp(line, '[-]?\d\d*+\.?\d*');
                out{end}.host_gpstime = str2double(strtok(line(nums(1):end), ' ')); %num
				out{end}.cam_gpstime = str2double(strtok(line(nums(2):end), ' ')); %num
				out{end}.frame_gpstime = str2double(strtok(line(nums(3):end), ' ')); %sec
                STATE = 3;
            case 3
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                nums = regexp(line, '[-]?\d\d*+\.?\d*');
                out{end}.index = str2double(strtok(line(nums(1):end), ' ')); %deg
                STATE = 4;
			case 4
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                nums = regexp(line, '[-]?\d\d*+\.?\d*');
                out{end}.host_roll = str2double(strtok(line(nums(1):end), ' ')); %deg
                out{end}.host_pitch = str2double(strtok(line(nums(2):end), ' ')); %deg
                out{end}.host_hdg = str2double(strtok(line(nums(3):end), ' ')); %ft
                STATE = 5;
            case 5
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                nums = regexp(line, '[-]?\d\d*+\.?\d*');
                out{end}.host_lat = str2double(strtok(line(nums(1):end), ' ')); %deg
                out{end}.host_lon = str2double(strtok(line(nums(2):end), ' ')); %deg
                out{end}.host_alt_gps_ft = str2double(strtok(line(nums(3):end), ' ')); %deg
				out{end}.host_alt_agl_ft = str2double(strtok(line(nums(4):end), ' ')); %deg
                STATE = 6;
			case 6
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                nums = regexp(line, '[-]?\d\d*+\.?\d*');
                out{end}.cam_roll = str2double(strtok(line(nums(1):end), ' ')); %deg
                out{end}.cam_pitch = str2double(strtok(line(nums(2):end), ' ')); %deg
                out{end}.cam_hdg = str2double(strtok(line(nums(3):end), ' ')); %ft
                STATE = 7;
			case 7
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                nums = regexp(line, '[-]?\d\d*+\.?\d*');
                out{end}.cam_lat = str2double(strtok(line(nums(1):end), ' ')); %deg
                out{end}.cam_lon = str2double(strtok(line(nums(2):end), ' ')); %deg
                out{end}.cam_alt_gps_ft = str2double(strtok(line(nums(3):end), ' ')); %deg
				STATE = 8;
			case 8
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                nums = regexp(line, '[-]?\d\d*+\.?\d*');
                out{end}.cam_P = str2double(strtok(line(nums(1):end), ' ')); %deg
                out{end}.cam_Q = str2double(strtok(line(nums(2):end), ' ')); %deg
                out{end}.cam_R = str2double(strtok(line(nums(3):end), ' ')); %deg
				STATE = 9;
			case 9
                if DBG
                    fprintf(1, 'CASE %d: %s', STATE, line);
                end
                nums = regexp(line, '[-]?\d\d*+\.?\d*');
                out{end}.cam_acc_x = str2double(strtok(line(nums(1):end), ' ')); %deg
                out{end}.cam_acc_y = str2double(strtok(line(nums(2):end), ' ')); %deg
                out{end}.cam_acc_z = str2double(strtok(line(nums(3):end), ' ')); %deg
				STATE = 0;
%             case 5
%                 if DBG
%                     fprintf(1, 'CASE %d: %s', STATE, line);
%                 end
%                 nums = regexp(line, '[-]?\d\d*+\.?\d*');
%                 out{end}.altitude_gps = str2double(strtok(line(nums(1):end), ' ')); %ft
%                 out{end}.altitude_agl = str2double(strtok(line(nums(2):end), ' ')); %ft
%                 STATE = 0;
            otherwise
        end
        
    end
end

