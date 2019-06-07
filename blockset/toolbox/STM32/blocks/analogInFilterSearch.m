%%***************************************************************************************
%% file         analogInFilterSearch.m
%% brief        Searches for analog input blocks with filtering enabled.
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
%% Search for the number of Analog input where filtering is turned on
function analogInFilterSearch(modelName)
% declare variable to hold number of filtered channels
numberOfFilteredChannels = 0;
% build an array with all the blocks that have a Tag starting with HANcoder_TARGET_
blockArray = find_system(modelName, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'Tag', 'HANcoder_TARGET_STM32_');
    % only perform check if at least 1 or more HANcoder Target blocks were used
    if (length(blockArray) > 0)
	% iterate through all blocks to make sure the tags match to the first one
        for i = 1 : length(blockArray)
            % Get the HANcoder struct from the UserData
			HANcoderStruct = get_param(blockArray{i}, 'UserData');
			% Check if it is an Analog input
            if (~isempty(HANcoderStruct) && strcmp(HANcoderStruct.BlockType, 'AnalogInput') == 1)
				% Check if filtering is turned on in the block
				if (strcmp(get_param(blockArray{i},'filtered'),'on')==1)
					numberOfFilteredChannels = numberOfFilteredChannels+1;
				end
            end
        end %end of for loop
	end
%% Add number of filtered channels to SYS_config.h
% Open SYS_config.h file, in append mode
file = fopen('SYS_Config.h', 'a');
    if (file == -1)
         error('### failed to open SYS_Config.h'); 
    end
	%Add the define with the number of filtered channels
	fprintf('### Adding the number analog channels with filtering: %d to SYS_config.h\n', numberOfFilteredChannels);
    fprintf(file, '#define NUMFILTEREDCHANNELS            %d\n', numberOfFilteredChannels);
	fclose(file);
end	
