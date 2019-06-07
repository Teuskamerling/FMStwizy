%%***************************************************************************************
%% file         HANcoderPinChecks.m
%% brief        Function to check whether a pin is already used in another HANcoder block
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
function HANcoderPinChecks(pinName,Target)
% Check pin name
% Check if the pin is part of the arduino headers
expression = 'CON[1234]\s-\s[AD]\d*';
matchStr = regexp(pinName,expression,'match');
if isempty(matchStr) % pin is not one of the arduino pinouts
    expression = 'CON\sP[DEFG]\s-\s\d*';
    matchStr = regexp(pinName,expression,'match');
end
if isempty(matchStr) % check if it is a pin from the UEXT
    expression = 'UEXT\s-\s\d*';
    matchStr = regexp(pinName,expression,'match');
end
if isempty(matchStr) % check if it is a pin from the EXT
    expression = 'EXT\s-\sD\d*';
    matchStr = regexp(pinName,expression,'match');
end
if isempty(matchStr)
    pinToCheck = pinName; % pinName is the name of the pin
else
    pinToCheck = matchStr{1}; % found with regexp
end

% Find all HANcoder blocks
HANcoderBlocks = find_system(bdroot,'RegExp', 'on', 'LookUnderMasks', 'All', 'Tag', 'HANcoder_TARGET_.');
[truefalse,index]=ismember(gcb,HANcoderBlocks); % Find the block which initiated this check
HANcoderBlocks{index} = []; % Delete it from the cell array
HANcoderBlocks = HANcoderBlocks(~cellfun(@isempty,HANcoderBlocks)); % Remove the empty cell
for index=1:1:length(HANcoderBlocks) % Loop through the HANcoder blocks
    HANcoderStruct = get_param(HANcoderBlocks{index},'UserData');
    BlockType = HANcoderStruct.BlockType;
    switch BlockType
        case 'BaseSampleTimeConfig'
        case 'CANconfig'
            MaskValues = get_param(HANcoderBlocks{index},'MaskValues');
            if(strcmp(Target,'E407'))
                if (strcmp('on',MaskValues{3})&&(strcmp(pinToCheck,'CON PD - 3')||strcmp(pinToCheck,'CON PD - 4')))
                    message = sprintf(['Error pin: "',pinToCheck, ' " is already used in ',HANcoderBlocks{index},' CAN1. \nSelect a different pin or remove the other block.']);
                    errordlg(message, 'Block error');
                end
                if (strcmp('on',MaskValues{8})&&(strcmp(pinToCheck,'CON3 - D1')||strcmp(pinToCheck,'CON4 - D11')))
                    message = sprintf(['Error pin: "',pinToCheck, '" is already used in ',HANcoderBlocks{index},' CAN2. \nSelect a different pin or remove the other block.']);
                    errordlg(message, 'Block error');
                end
            elseif(strcmp(Target,'Olimexino'))
                 if (strcmp('on',MaskValues{1})&&(strcmp(pinToCheck,'CON4 - D14')||strcmp(pinToCheck,'EXT - D24')))
                    message = sprintf(['Error pin: "',pinToCheck, '" is already used in ',HANcoderBlocks{index},' CAN2. \nSelect a different pin or remove the other block.']);
                    errordlg(message, 'Block error');
                end
            end
        case 'CustomIDconfig'
        case 'FileLoggerClose'
        case 'FileLoggerSignal'
        case 'FileLoggerInit'
        case 'EraseEEPROM'
        case 'QuadratureEncoderGet'
            QuadratureEncoderCheck(HANcoderBlocks{index},pinToCheck, Target);
        case 'SaveEEPROM'
        case 'SPImasterInit'
            MaskValues = get_param(HANcoderBlocks{index},'MaskValues');
            if (strcmp(MaskValues{1},'SPI1: D13 & D12 & D11')&& (strcmp(pinToCheck,'CON4 - D11')||strcmp(pinToCheck,'CON4 - D12')||strcmp(pinToCheck,'CON4 - D13')))
                message = sprintf(['Error pin: "',pinToCheck, '" is already used in ',HANcoderBlocks{index},' SPI1. \nSelect a different pin or remove the other block.']);
                errordlg(message, 'Block error');
            end
            if (strcmp(MaskValues{1},'SPI2: UEXT 9 & 7 & 8')&& (strcmp(pinToCheck,'UEXT - 7'))) %UEXT 8 & 9 are not used for anything else
                message = sprintf(['Error pin: "',pinToCheck, '" is already used in ',HANcoderBlocks{index},' SPI2. \nSelect a different pin or remove the other block.']);
                errordlg(message, 'Block error');
            end
            if (strcmp(MaskValues{1},'SPI2: D32 & D33 & D34')&& (strcmp(pinToCheck,'EXT - D32')||strcmp(pinToCheck,'EXT - D33')||strcmp(pinToCheck,'EXT - D34')))
                message = sprintf(['Error pin: "',pinToCheck, '" is already used in ',HANcoderBlocks{index},' SPI2. \nSelect a different pin or remove the other block.']);
                errordlg(message, 'Block error');
            end
        case 'SPImasterTransfer'
            MaskValues = get_param(HANcoderBlocks{index},'MaskValues');
            if (strcmp(MaskValues{2},'on')&& strcmp(pinToCheck,MaskValues{3})) %If slave select is enabled and the pin matches
                message = sprintf(['Error pin: "',pinToCheck, '" is already used in ',HANcoderBlocks{index},' Slave Select. \nSelect a different pin or remove the other block.']);
                errordlg(message, 'Block error');
            end
        case 'UARTinit'
            UARTcheck(HANcoderBlocks{index},pinToCheck, Target);
        case 'UARTreceive'
            UARTcheck(HANcoderBlocks{index},pinToCheck, Target);
        case 'UARTreceiveBufferStatus'
            UARTcheck(HANcoderBlocks{index},pinToCheck, Target);
        case 'UARTsend'
            UARTcheck(HANcoderBlocks{index},pinToCheck, Target);
        case 'XCPoverTCPconfig'
        case 'XCPoverUSBconfig'
        otherwise
            MaskValues = get_param(HANcoderBlocks{index},'MaskValues');
            if ~isempty(MaskValues) % Prevent errors when there are no MaskValues
                PinInUse = MaskValues{1}; % Pin should always be the first choice in a block
                if strfind(PinInUse,pinToCheck)
                    message = sprintf(['Error pin: "',pinToCheck, ' " is already used in ',HANcoderBlocks{index},'. \nSelect a different pin or remove the duplicate block.']);
                    errordlg(message, 'Block error');
                    break;
                end
            end
    end % end of switch
end % end of for loop
end % end of function HANcoderPinChecks()

function UARTcheck(block,pin, Target)
MaskValues = get_param(block,'MaskValues');
% Checks for the Olimexino
if strcmp(Target, 'Olimexino')
    % Olmixino Settings: 'UART1: D7 & D8', 'UART2: D1 & D0', 'UART3: D29 & D30'
    if (strcmp(MaskValues{1},'UART1: D7 & D8') && (strcmp('CON3 - D7', pin) || strcmp('CON4 - D8', pin))) || ...
       (strcmp(MaskValues{1},'UART2: D1 & D0') && (strcmp('CON3 - D1', pin) || strcmp('CON3 - D0', pin))) || ...
       (strcmp(MaskValues{1},'UART3: D29 & D30') && (strcmp('EXT - D29', pin) || strcmp('EXT - D30', pin)))
        % Display error dialog
        message = sprintf(['Error pin: "',pin, '" is already used in ',block,'. \nSelect a different pin or remove the other block.']);
        errordlg(message, 'Block error');
    end
% Checks for the E407 
elseif strcmp(Target, 'E407')
       % E407 settings: 'TIM1: CON PE - 12 & 14 (5V Tolerant)','TIM3: CON4 - D12 & D11 (5V Tolerant)','TIM4: CON PD - 15 & 16 (5V Tolerant)','TIM8: UEXT - 3 & 4 (5V Tolerant)'
    if (strcmp(MaskValues{1},'UART1: D7 & D8') && (strcmp('CON PE - 12', pin) || strcmp('CON PE - 14', pin))) || ...
       (strcmp(MaskValues{1},'UART1: D7 & D8') && (strcmp('CON4 - D12', pin) || strcmp('CON4 - D11', pin))) || ...
       (strcmp(MaskValues{1},'UART1: D7 & D8') && (strcmp('CON PD - 15', pin) || strcmp('CON PD - 16', pin))) || ...
       (strcmp(MaskValues{1},'UART1: D7 & D8') && (strcmp('UEXT - 3', pin) || strcmp('UEXT - 4', pin)))
        % Display error dialog
        message = sprintf(['Error pin: "',pin, '" is already used in ',block,'. \nSelect a different pin or remove the other block.']);
        errordlg(message, 'Block error');
    end
end
end % end of function UARTcheck()


function QuadratureEncoderCheck(block, pin, Target)
MaskValues = get_param(block,'MaskValues');
% Checks for the Olimexino
if strcmp(Target, 'Olimexino')
    % Olmixino Settings: 'TIM1: D6 & D7 - 5V Tolerant', 'TIM2: D2 & D3', 'TIM3: D35 & D36 - 5V Tolerant', 'TIM4: D5 & D9 - 5V Tolerant)'
    if (strcmp(MaskValues{1},'TIM1: D6 & D7 - 5V Tolerant') && (strcmp('CON3 - D6', pin) || strcmp('CON3 - D7', pin))) || ...
       (strcmp(MaskValues{1},'TIM2: D2 & D3') && (strcmp('CON3 - D2', pin) || strcmp('CON3 - D3', pin))) || ...
       (strcmp(MaskValues{1},'TIM3: D35 & D36 - 5V Tolerant') && (strcmp('EXT - D35', pin) || strcmp('EXT - D36', pin))) || ...
       (strcmp(MaskValues{1},'TIM4: D5 & D9 - 5V Tolerant)') && (strcmp('CON3 - D5', pin) || strcmp('CON4 - D9', pin)))
        % Display error dialog
        message = sprintf(['Error pin: "',pin, '" is already used in ',block,'. \nSelect a different pin or remove the other block.']);
        errordlg(message, 'Block error');
    end
% Checks for the E407 
elseif strcmp(Target, 'E407')
       % E407 settings: 'TIM1: CON PE - 12 & 14 (5V Tolerant)','TIM3: CON4 - D12 & D11 (5V Tolerant)','TIM4: CON PD - 15 & 16 (5V Tolerant)','TIM8: UEXT - 3 & 4 (5V Tolerant)'
    if (strcmp(MaskValues{1},'TIM1: CON PE - 12 & 14 (5V Tolerant)') && (strcmp('CON PE - 12', pin) || strcmp('CON PE - 14', pin))) || ...
       (strcmp(MaskValues{1},'TIM3: CON4 - D12 & D11 (5V Tolerant)') && (strcmp('CON4 - D12', pin) || strcmp('CON4 - D11', pin))) || ...
       (strcmp(MaskValues{1},'TIM4: CON PD - 15 & 16 (5V Tolerant)') && (strcmp('CON PD - 15', pin) || strcmp('CON PD - 16', pin))) || ...
       (strcmp(MaskValues{1},'TIM8: UEXT - 3 & 4 (5V Tolerant)') && (strcmp('UEXT - 3', pin) || strcmp('UEXT - 4', pin)))
        % Display error dialog
        message = sprintf(['Error pin: "',pin, '" is already used in ',block,'. \nSelect a different pin or remove the other block.']);
        errordlg(message, 'Block error');
    end
end
end % end of function QuadratureEncoderCheck()



%% NOTES
% use strfind(StringToLookIn, StringToFind) to find a pin name



