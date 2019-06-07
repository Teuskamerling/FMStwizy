%%***************************************************************************************
%% file         sfcn_custom_version_config_mcb.m
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
function [] = sfcn_custom_version_config_mcb(softID, softNUM)

if numel(softID) == 0 && numel(softNUM) == 0
    message  = 'Warning: You are using the custom ID block, but did not fill in an ID or software version number, compiling now would show an error.';
	errordlg(message, 'Block error');
end

% print result

    customID = char(softID); 
    customID = strcat(customID,'_v', softNUM);
	
    formatOut = 'ddmmyy_HHMMSS';
    daten = datestr(now, formatOut);
    stationIdString = strcat(bdroot, '_', customID, '_', daten);
    %%fprintf('### ID will be formatted as following (final version may differ due to build time): %s\n', stationIdString);

% calculate stationIdString length
stationIdLength = numel(stationIdString);	
	
% check if the entire name is not larger than 255 characters
	if stationIdLength > 255
        message = 'Error: Station ID is larger than 255 characters. Use a shorter station ID, please';
        % Abort and display pop-up window with error message.
		errordlg(message, 'Block error');   
    end	
	
	
assignin('base', 'kXcpStationId', stationIdString);


	

% create resource keywords to be reserved in resource database
modelRTWFields = struct('stationIdString', stationIdString, 'stationIdLength', int2str(stationIdLength));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);

%%******************************* end of sfcn_custom_version_config_mcb.m **********************



