%%***************************************************************************************
%% file         sfcn_uart_init_mcb.m
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
function [uartChannelInfoStr] = sfcn_uart_init_mcb(hardwareID, channel, baudrate, databits, parity, stopbits, txbufsize, rxbufsize, useStatusOut)

% =============================== Generic ===============================
% Before we are able to change the block, we have to de-activate the link
% of this block to the library.
% So check if the link status is active:
if(~strcmp(get_param(gcb,'LinkStatus'),'inactive'))
    % De-activate the library link
	set_param(gcb,'LinkStatus','inactive')
end

% Get current number of outputs ('gcb' is a system defined var that points
% to the tree structure of the current block)
currentOutputNumber = length (find_system (gcb, 'LookUnderMasks', 'on', 'BlockType', 'Outport'));

% Check if OutPorts must be used
% add port
if useStatusOut
  if currentOutputNumber ~= 1
    % Get the position of the terminate block
    thisPosition = get_param ([gcb, '/Term1'], 'Position');
    % Remove the terminate block
    delete_block ([gcb, '/Term1']);
    % Add a output block
    add_block ('built-in/Outport', [gcb, '/Out1']);
    % Alligh the new output block to be placed at the position of the
    % removed terminate
    set_param ([gcb, '/Out1'], 'Position', thisPosition);
  end
% remove port
else
	if currentOutputNumber ~= 0
    % Get the position of the output block
    thisPosition = get_param ([gcb, '/Out1'], 'Position');
    % Remove the output block
    delete_block ([gcb, '/Out1']);
    % Add a terminate block
    add_block ('built-in/Terminator', [gcb, '/Term1']);
    % Alligh the new terminate block to be placed at the position of the
    % removed output
    set_param ([gcb, '/Term1'], 'Position', thisPosition);
	end
end

% array with baudrate strings
baudratesStrings = { '300',  ...\
                     '600',  ...\
                     '1200',  ...\
                     '2400',  ...\
                     '4800',  ...\
                     '9600',  ...\
                     '19200',  ...\
                     '38400',  ...\
                     '57600',  ...\
                     '115200' };
                     
% construct the baudrate string
baudrateStr = baudratesStrings{baudrate};

% array with databits strings
databitsStrings = { 'DATA_BITS_8',  ...\
                    'DATA_BITS_9' };

% construct the databits string
databitsStr = databitsStrings{databits};

% array with parity strings
parityStrings = { 'PARITY_NONE',  ...\
                  'PARITY_EVEN',  ...\
                  'PARITY_ODD' };

% construct the parity string
parityStr = parityStrings{parity};

% array with stopbits strings
stopbitsStrings = { 'STOP_BITS_1',  ...\
                    'STOP_BITS_2' };

% construct the stopbits string
stopbitsStr = stopbitsStrings{stopbits};

% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with channel strings
  channelStrings = { 'UART_CHANNEL1_PA9_PA10',  ...\
                     'UART_CHANNEL2_PA2_PA3',  ...\
                     'UART_CHANNEL3_PB10_PB11' };
                   
  % construct the channel string
  channelStr = channelStrings{channel};

  % array with channel infos
  uartChannelInfoStrings = { 'UART1: D7 & D8',  ...\
                             'UART2: D1 & D0',  ...\
                             'UART3: D29 & D30'};

  % construct the moduleID info string
  uartChannelInfoStr = uartChannelInfoStrings{channel};
% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with channel strings
  channelStrings = { 'UART_CHANNEL1_PB6_PB7',  ...\
                     'UART_CHANNEL2_PD5_PD6',  ...\
                     'UART_CHANNEL3_PD8_PD9',  ...\
                     'UART_CHANNEL6_PC6_PC7' };
                   
  % construct the channel string
  channelStr = channelStrings{channel};

  % array with channel infos
  uartChannelInfoStrings = { 'UART1: CON3 - D1 & D0',  ...\
                             'UART2: CON PD - 8 & 9',  ...\
                             'UART3: CON PD - 11 & 12',  ...\
                             'UART6: UEXT-3 & 4'};

  % construct the moduleID info string
  uartChannelInfoStr = uartChannelInfoStrings{channel};
% =============================== STM32P405 ===============================
elseif (hardwareID == 3) 
  % array with channel strings
  channelStrings = { 'UART_CHANNEL1_PA9_PA10',  ...\
                     'UART_CHANNEL2_PA2_PA3' };
                   
  % construct the channel string
  channelStr = channelStrings{channel};

  % array with channel infos
  uartChannelInfoStrings = { 'UART1: Con. UEXT-3 & Con. UEXT-4 (PA9 & PA10)',  ...\
                             'UART2: SUB D9 (PA2 & PA3)'};

  % construct the moduleID info string
  uartChannelInfoStr = uartChannelInfoStrings{channel};
end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('channel', channelStr, 'baudrate', baudrateStr, 'databits', databitsStr, 'parity', parityStr, 'stopbits', stopbitsStr, 'txbufsize', int2str(txbufsize), 'rxbufsize', int2str(rxbufsize), 'useStatusOut', int2str(useStatusOut));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);

%%******************************* end of sfcn_uart_init_mcb.m **************************


