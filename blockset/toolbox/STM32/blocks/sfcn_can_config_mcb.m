%%***************************************************************************************
%% file         sfcn_can_config_mcb.m
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
function [] = sfcn_can_config_mcb(txBufSize, evBufSize, canBus1, baudRate1, rxMask1, rxCode1, rxMode1, canBus2, baudRate2, rxMask2, rxCode2, rxMode2)

% construct the mask string
if (rxMask1 < 0) || (rxMask1 > hex2dec('1fffffff'))
    fprintf ('Error: Invalid filter mask specified in block "CAN Config" for CAN1. It must be in the 0x00000000-0x1fffffff range.\n');
    rxMask1Str = '0x0000000000';
else
    rxMask1Str = sprintf('0x%08X', rxMask1);
end;

% construct the code string
if (rxCode1 < 0) || (rxCode1 > hex2dec('1fffffff'))
    fprintf ('Error: Invalid filter code specified in block "CAN Config" for CAN1. It must be in the 0x00000000-0x1fffffff range.\n');
    rxCode1Str = '0x0000000000';
else
    rxCode1Str = sprintf('0x%08X', rxCode1);
end;

% construct the mask string
if (rxMask2 < 0) || (rxMask2 > hex2dec('1fffffff'))
    fprintf ('Error: Invalid filter mask specified in block "CAN Config" for CAN2. It must be in the 0x00000000-0x1fffffff range.\n');
    rxMask2Str = '0x0000000000';
else
    rxMask2Str = sprintf('0x%08X', rxMask2);
end;

% construct the code string
if (rxCode2 < 0) || (rxCode2 > hex2dec('1fffffff'))
    fprintf ('Error: Invalid filter code specified in block "CAN Config" for CAN2. It must be in the 0x00000000-0x1fffffff range.\n');
    rxCode2Str = '0x0000000000';
else
    rxCode2Str = sprintf('0x%08X', rxCode2);
end;

% array with mode strings
rxModeStrings = {'CAN_FILTER_MODE_STDID_ONLY', 'CAN_FILTER_MODE_EXTID_ONLY', 'CAN_FILTER_MODE_MIXED_ID'};
% construct the mode string
rxMode1Str = rxModeStrings{rxMode1};
rxMode2Str = rxModeStrings{rxMode2};

% array with baudrate strings
baudrateStrings = {'125000', '250000', '500000', '1000000'};
% construct the baudrate string
baudRate1Str = baudrateStrings{baudRate1};
baudRate2Str = baudrateStrings{baudRate2};



% create resource keywords to be reserved in resource database
modelRTWFields = struct('txBufSize', int2str(txBufSize), 'evBufSize', int2str(evBufSize), 'canBus1', int2str(canBus1), 'baudRate1', baudRate1Str, 'rxMask1', rxMask1Str, 'rxCode1', rxCode1Str, 'rxMode1', rxMode1Str, 'canBus2', int2str(canBus2), 'baudRate2', baudRate2Str, 'rxMask2', rxMask2Str, 'rxCode2', rxCode2Str, 'rxMode2', rxMode2Str);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);

%%******************************* end of sfcn_can_config_mcb.m **************************


