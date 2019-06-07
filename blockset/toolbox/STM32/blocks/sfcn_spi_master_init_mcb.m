%%***************************************************************************************
%% file         sfcn_spi_master_init_mcb.m
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
function [spiChannelInfoStr] = sfcn_spi_master_init_mcb(hardwareID, channel, speed, polarity, phase)

% =============================== Generic ===============================
% array with polarity strings
polarityStrings = { 'POLARITY_LOW',  ...\
                    'POLARITY_HIGH' };

% construct the polarity string
polarityStr = polarityStrings{polarity};

% array with phase strings
phaseStrings = { 'PHASE_1ST_EDGE',  ...\
                 'PHASE_2ND_EDGE' };

% construct the phase string
phaseStr = phaseStrings{phase};


% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with channel strings
  channelStrings = { 'SPI_MASTER_CHANNEL1_PA5_PA6_PA7',  ...\
                     'SPI_MASTER_CHANNEL2_PB13_PB14_PB15' };
                   
  % construct the channel string
  channelStr = channelStrings{channel};

  % array with communication speed strings
  speedStrings = { 'COM_SPEED_36000_KHZ',  ...\
                   'COM_SPEED_18000_KHZ',  ...\
                   'COM_SPEED_9000_KHZ',  ...\
                   'COM_SPEED_4500_KHZ',  ...\
                   'COM_SPEED_2250_KHZ',  ...\
                   'COM_SPEED_1125_KHZ',  ...\
                   'COM_SPEED_563_KHZ',  ...\
                   'COM_SPEED_281_KHZ' };
                       
  % construct the communication speed string
  speedStr = speedStrings{speed};

  % array with channel infos
  spiChannelInfoStrings = { 'SPI1: D13 & D12 & D11',  ...\
                            'SPI2: D32 & D33 & D34'};

  % construct the moduleID info string
  spiChannelInfoStr = spiChannelInfoStrings{channel};
% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with channel strings
  channelStrings = { 'SPI_MASTER_CHANNEL1_PA5_PA6_PB5',  ...\
                     'SPI_MASTER_CHANNEL2_PB10_PC2_PC3' };
                  
  % construct the channel string
  channelStr = channelStrings{channel};

  % array with communication speed strings
  speedStrings = { 'COM_SPEED_21000_KHZ',  ...\
                   'COM_SPEED_10500_KHZ',  ...\
                   'COM_SPEED_5250_KHZ',  ...\
                   'COM_SPEED_2625_KHZ',  ...\
                   'COM_SPEED_1313_KHZ',  ...\
                   'COM_SPEED_656_KHZ',  ...\
                   'COM_SPEED_328_KHZ',  ...\
                   'COM_SPEED_164_KHZ' };

                 
  % construct the communication speed string
  speedStr = speedStrings{speed};

  % array with channel infos
  spiChannelInfoStrings = { 'SPI1: D13 & D12 & D11',  ...\
                            'SPI2: UEXT 9 & 7 & 8'};

  % construct the moduleID info string
  spiChannelInfoStr = spiChannelInfoStrings{channel};
% =============================== STM32P405 ===============================
elseif (hardwareID == 3) 
  % array with channel strings
  channelStrings = { 'SPI_MASTER_CHANNEL1_PA5_PA6_PA7',  ...\
                     'SPI_MASTER_CHANNEL2_PB13_PB14_PB15' };
                  
  % construct the channel string
  channelStr = channelStrings{channel};

  % array with communication speed strings
  speedStrings = { 'COM_SPEED_21000_KHZ',  ...\
                   'COM_SPEED_10500_KHZ',  ...\
                   'COM_SPEED_5250_KHZ',  ...\
                   'COM_SPEED_2625_KHZ',  ...\
                   'COM_SPEED_1313_KHZ',  ...\
                   'COM_SPEED_656_KHZ',  ...\
                   'COM_SPEED_328_KHZ',  ...\
                   'COM_SPEED_164_KHZ' };

                 
  % construct the communication speed string
  speedStr = speedStrings{speed};

  % array with channel infos
  spiChannelInfoStrings = { 'SPI1: UEXT 9 & 7 & 8 (PA5 & PA6 & PA7)',  ...\
                            'SPI2: PB13 & PB14 & PB15'};

  % construct the moduleID info string
  spiChannelInfoStr = spiChannelInfoStrings{channel};
end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('channel', channelStr, 'speed', speedStr, 'polarity', polarityStr, 'phase', phaseStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);

%%******************************* end of sfcn_uart_init_mcb.m **************************


