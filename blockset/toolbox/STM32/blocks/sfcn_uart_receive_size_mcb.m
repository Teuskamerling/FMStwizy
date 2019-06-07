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
function [uartChannelInfoStr] = sfcn_uart_init_mcb(hardwareID, channel)

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
modelRTWFields = struct('channel', channelStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);

%%******************************* end of sfcn_uart_init_mcb.m **************************


