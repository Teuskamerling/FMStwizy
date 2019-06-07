%%***************************************************************************************
%% file         sfcn_can_send_mcb.m
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
function [] = sfcn_can_send_mcb(ip1, ip2, ip3, ip4, port, inputNumber, dataType, byteOrder)

%Our data type parameters:
% int8
% uint8
% int16
% uint16
% int32
% uint32
% boolean

% calculate the data length (target specific)
dataLength = 0;
switch(dataType)
    case {1,2,7}
        % data length of 1 byte
        dataLength = inputNumber * 1;
    case {3,4}
        % data length of 2 bytes
        dataLength = inputNumber * 2;
    case {5,6}
        % data length of 4 bytes
        dataLength = inputNumber * 4;
end



% The number of inputs needs to match the amount of inputs the user did set

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

% Now we update the number of inputs to match the given amount

% If we have to many inputs:
if inputNumber < currentInputNumber
    % Open the default library (invisible) for use
    load_system('built-in')
    % Loop trough all inputs that are to much
    for counterRemove = inputNumber+1:currentInputNumber
        % Get the position of the input block
        thisPosition = get_param ([gcb, '/In', num2str(counterRemove)], 'Position');
        % Remove the input block
        delete_block ([gcb, '/In', num2str(counterRemove)])
        % Add a ground block
        add_block ('built-in/Ground', [gcb, '/Gnd', num2str(counterRemove)])
        % Alligh the new ground block to be placed at the position of the
        % removed input
        set_param ([gcb, '/Gnd', num2str(counterRemove)], 'Position', thisPosition)
    end
% Or if we have to few inputs:
elseif inputNumber > currentInputNumber
    % Open the default library (invisible) for use
    load_system('built-in')
    % Loop trough all Grounds that needs to be inputs
    for counterAdd = currentInputNumber+1:inputNumber
        % Get the position of the ground block
        thisPosition = get_param ([gcb, '/Gnd', num2str(counterAdd)], 'Position');
        % Remove the ground block
        delete_block ([gcb, '/Gnd', num2str(counterAdd)])
        % Add a input block
        add_block ('built-in/Inport', [gcb, '/In', num2str(counterAdd)])
        % Alligh the new input block to be placed at the position of the
        % removed ground
        set_param ([gcb, '/In', num2str(counterAdd)], 'Position', thisPosition)
    end
end

% NOTE: I had some problems with this dynamic input system, because the
% S-Function updates at a later time. The problem with that, is that you
% cannot draw a line to the S-Function if you just added a new port. And on
% top of that, the line is not drawn, resulting in an error when you attemp 
% to remove the line when you lower the amounts of ports.
% The solution to this problem is to have the S-Function set his amount of
% input ports static to 32, and add Grounds in the model to close those 
% inputs. That way you do not have the S-Function to update its amount of 
% ports.


% The inputs are in line with the user input now.

% (ip1, ip2, ip3, ip4, port, inputNumber, dataType, byteOrder)


% Create resource keywords to be reserved in resource database
modelRTWFields = struct('ip1', int2str(ip1), 'ip2', int2str(ip2), 'ip3', int2str(ip3), 'ip4', int2str(ip4),...
                        'port', int2str(port), 'inputNumber', int2str(inputNumber), 'dataType', int2str(dataType),...
                        'byteOrder', int2str(byteOrder), 'dataLength', int2str(dataLength));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_can_send_mcb.m ****************************

