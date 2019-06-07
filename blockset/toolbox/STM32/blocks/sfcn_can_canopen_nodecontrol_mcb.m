%%***************************************************************************************
%% file         sfcn_can_canopen_sdoread_mcb.m
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
function [] = sfcn_can_canopen_nodecontrol_mcb(canBus, nodeId, state, waitingTime, tsamp)

%Our data type parameters:
% int8
% uint8
% int16
% uint16
% int32
% uint32
% boolean

%Check node ID
if (nodeId > 64)
	errordlg('The node ID has to be between 0x000 and 0x03F');
	error('The node ID has to be between 0x000 and 0x03F');
end

switch (state)
	case ('Pre-Operational')
	state = 1;
	case ('Operational')
	state = 2;
	case ('Stop')
	state = 3;
	case ('Reset')	
	state = 4;
	case ('Reset Communication')
	state = 5;
end

% get the steptime from the model
steptime_ms = uint32(str2num(get_param(bdroot, 'FixedStep')) * 1000);

% Create resource keywords to be reserved in resource database
modelRTWFields = struct('canBus', int2str(canBus-1),...
    'nodeId', int2str(nodeId), 'state', int2str(state), 'waitingTime', int2str(waitingTime),...
	'tsamp', int2str(tsamp), 'baseSampleTime', int2str(steptime_ms));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'Tag', 'HANcoder_TARGET_.');
set_param(HANcoder_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_can_canopen_nodecontroll_mcb.m *************************


