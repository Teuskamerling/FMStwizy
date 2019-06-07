%%***************************************************************************************
%% file         sfcn_controller_error_deleteerror_mcb.m
%% brief        Block mask initialization function.
%%
%%---------------------------------------------------------------------------------------
%%                          C O P Y R I G H T
%%---------------------------------------------------------------------------------------
%%  Copyright 2019 (c) by HAN Automotive     http://www.han.nl     All rights reserved
%%
%%---------------------------------------------------------------------------------------
%%                            L I C E N S E
%%---------------------------------------------------------------------------------------
%% Permission is hereby granted, free of charge, to any person obtaining a copy of this
%% software and associated documentation files (the "Software"), to deal in the Software
%% without restriction, including without limitation the rights to use, copy, modify, merge,
%% publish, distribute, sublicense, and/or sell copies of the Software, and to permit
%% persons to whom the Software is furnished to do so, subject to the following conditions:
%%
%% The above copyright notice and this permission notice shall be included in all copies or
%% substantial portions of the Software.
%%
%% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
%% INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
%% PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
%% FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
%% OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
%% DEALINGS IN THE SOFTWARE.
%%
%%***************************************************************************************
function [] = sfcn_controller_error_deleteerror_mcb(useInPorts, errorID, para)

% Before we are able to change the block, we have to de-activate the link
% of this block to the library.
% So check if the link status is active:
if(~strcmp(get_param(gcb,'LinkStatus'),'inactive'))
    % De-activate the library link
	set_param(gcb,'LinkStatus','inactive')
end

% Get current number of inputs ('gcb' is a system defined var that points
% to the tree structure of the current block)
currentInputNumber = length (find_system (gcb, 'LookUnderMasks', 'on', 'BlockType', 'Inport'));

% Check if InPorts must be used
% add port
if useInPorts
    if currentInputNumber ~= 2 
        for counter = 1:2
            % Get the position of the ground block
            thisPosition = get_param ([gcb, '/Gnd', num2str(counter)], 'Position');
            % Remove the ground block
            delete_block ([gcb, '/Gnd',num2str(counter)]);
            % Add a input block
            add_block ('built-in/Inport', [gcb, '/In', num2str(counter)]);
            % Alligh the new input block to be placed at the position of the
            % removed ground
            set_param ([gcb, '/In', num2str(counter)], 'Position', thisPosition);
            % disable error code and error parameter
            set_param(gcb,'MaskEnables',{'on','off','off'});
        end
    end
	
% remove port
else
	if currentInputNumber ~= 0
		for counter = 1:2
			% Get the position of the input block
			thisPosition = get_param ([gcb, '/In', num2str(counter)], 'Position');
			% Remove the input block
			delete_block([gcb, '/In', num2str(counter)]);
			% Add a ground block
			add_block('built-in/Ground', [gcb, '/Gnd', num2str(counter)]);
			% Alligh the new grund block to be placed at the position of the
			% removed input
			set_param([gcb, '/Gnd', num2str(counter)], 'Position', thisPosition);
			% enable error code and error parameter
			set_param(gcb,'MaskEnables',{'on','on','on'});
		end
	end
end




% create resource keywords to be reserved in resource database
modelRTWFields = struct('useInPorts', int2str(useInPorts), 'errorID', int2str(errorID), 'para', int2str(para));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_controller_error_deleteerror_mcb.m ********