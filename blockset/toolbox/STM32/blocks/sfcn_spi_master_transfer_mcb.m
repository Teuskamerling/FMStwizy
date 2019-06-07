%%***************************************************************************************
%% file         sfcn_spi_master_transfer_mcb.m
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
function [spiChannelInfoStr] = sfcn_spi_master_transfer_mcb(hardwareID, channel, useSS, SSpin, SSenable, SSdisable, delayCycles)


% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with channel strings
  channelStrings = { 'SPI_MASTER_CHANNEL1_PA5_PA6_PA7',  ...\
                     'SPI_MASTER_CHANNEL2_PB13_PB14_PB15' };
                   
  % construct the channel string
  channelStr = channelStrings{channel};

  % array with pinIDs
  pinIDStrings = { 'DIGOUT_PORTC_PIN0',  ...\
                   'DIGOUT_PORTC_PIN1',  ...\
                   'DIGOUT_PORTC_PIN2',  ...\
                   'DIGOUT_PORTC_PIN3',  ...\
                   'DIGOUT_PORTC_PIN4',  ...\
                   'DIGOUT_PORTC_PIN5',  ...\
                   'DIGOUT_PORTA_PIN3',  ...\
                   'DIGOUT_PORTA_PIN2',  ...\
                   'DIGOUT_PORTA_PIN0',  ...\
                   'DIGOUT_PORTA_PIN1',  ...\
                   'DIGOUT_PORTB_PIN5',  ...\
                   'DIGOUT_PORTB_PIN6',  ...\
                   'DIGOUT_PORTA_PIN8',  ...\
                   'DIGOUT_PORTA_PIN9',  ...\
                   'DIGOUT_PORTA_PIN10', ...\
                   'DIGOUT_PORTB_PIN7',  ...\
                   'DIGOUT_PORTA_PIN4',  ...\
                   'DIGOUT_PORTA_PIN7',  ...\
                   'DIGOUT_PORTA_PIN6',  ...\
                   'DIGOUT_PORTA_PIN5',  ...\
                   'DIGOUT_PORTD_PIN2',  ...\
                   'DIGOUT_PORTC_PIN10', ...\
                   'DIGOUT_PORTB_PIN12', ...\
                   'DIGOUT_PORTB_PIN13', ...\
                   'DIGOUT_PORTC_PIN6',  ...\
                   'DIGOUT_PORTC_PIN7',  ...\
                   'DIGOUT_PORTC_PIN8'};
  % construct the pinID string
  pinIDStr = pinIDStrings{SSpin};



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

  % array with pinIDs
  pinIDStrings = { 'DIGOUT_PORTC_PIN0',  ...\
                   'DIGOUT_PORTF_PIN6',  ...\
                   'DIGOUT_PORTF_PIN7',  ...\
                   'DIGOUT_PORTF_PIN8',  ...\
                   'DIGOUT_PORTF_PIN9',  ...\
                   'DIGOUT_PORTF_PIN10',  ...\
                   'DIGOUT_PORTB_PIN7',  ...\
                   'DIGOUT_PORTB_PIN6',  ...\
                   'DIGOUT_PORTE_PIN2',  ...\
                   'DIGOUT_PORTE_PIN4',  ...\
                   'DIGOUT_PORTE_PIN5',  ...\
                   'DIGOUT_PORTE_PIN6',  ...\
                   'DIGOUT_PORTG_PIN7',  ...\
                   'DIGOUT_PORTG_PIN8',  ...\
                   'DIGOUT_PORTG_PIN12',  ...\
                   'DIGOUT_PORTG_PIN15',  ...\
                   'DIGOUT_PORTA_PIN4',  ...\
                   'DIGOUT_PORTB_PIN5',  ...\
                   'DIGOUT_PORTA_PIN6',  ...\
                   'DIGOUT_PORTA_PIN5',  ...\
                   'DIGOUT_PORTD_PIN0',  ...\
                   'DIGOUT_PORTD_PIN1',  ...\
                   'DIGOUT_PORTD_PIN2',  ...\
                   'DIGOUT_PORTD_PIN3',  ...\
                   'DIGOUT_PORTD_PIN4',  ...\
                   'DIGOUT_PORTD_PIN5',  ...\
                   'DIGOUT_PORTD_PIN6',  ...\
                   'DIGOUT_PORTD_PIN7',  ...\
                   'DIGOUT_PORTD_PIN8',  ...\
                   'DIGOUT_PORTD_PIN9',  ...\
                   'DIGOUT_PORTD_PIN10',  ...\
                   'DIGOUT_PORTD_PIN11',  ...\
                   'DIGOUT_PORTD_PIN12',  ...\
                   'DIGOUT_PORTD_PIN13',  ...\
                   'DIGOUT_PORTD_PIN14',  ...\
                   'DIGOUT_PORTD_PIN15',  ...\
                   'DIGOUT_PORTE_PIN0',  ...\
                   'DIGOUT_PORTE_PIN1',  ...\
                   'DIGOUT_PORTE_PIN3',  ...\
                   'DIGOUT_PORTE_PIN7',  ...\
                   'DIGOUT_PORTE_PIN8',  ...\
                   'DIGOUT_PORTE_PIN9',  ...\
                   'DIGOUT_PORTE_PIN10',  ...\
                   'DIGOUT_PORTE_PIN11',  ...\
                   'DIGOUT_PORTE_PIN12',  ...\
                   'DIGOUT_PORTE_PIN13',  ...\
                   'DIGOUT_PORTE_PIN14',  ...\
                   'DIGOUT_PORTE_PIN15',  ...\
                   'DIGOUT_PORTF_PIN0',  ...\
                   'DIGOUT_PORTF_PIN1',  ...\
                   'DIGOUT_PORTF_PIN2',  ...\
                   'DIGOUT_PORTF_PIN3',  ...\
                   'DIGOUT_PORTF_PIN4',  ...\
                   'DIGOUT_PORTF_PIN5',  ...\
                   'DIGOUT_PORTF_PIN11',  ...\
                   'DIGOUT_PORTF_PIN12',  ...\
                   'DIGOUT_PORTF_PIN13',  ...\
                   'DIGOUT_PORTF_PIN14',  ...\
                   'DIGOUT_PORTF_PIN15',  ...\
                   'DIGOUT_PORTG_PIN0',  ...\
                   'DIGOUT_PORTG_PIN1',  ...\
                   'DIGOUT_PORTG_PIN2',  ...\
                   'DIGOUT_PORTG_PIN3',  ...\
                   'DIGOUT_PORTG_PIN4',  ...\
                   'DIGOUT_PORTG_PIN5',  ...\
                   'DIGOUT_PORTG_PIN6',  ...\
                   'DIGOUT_PORTG_PIN9',  ...\
                   'DIGOUT_PORTG_PIN10',  ...\
                   'DIGOUT_PORTG_PIN11',  ...\
                   'DIGOUT_PORTG_PIN13',  ...\
                   'DIGOUT_PORTG_PIN14'};
  % construct the pinID string
  pinIDStr = pinIDStrings{SSpin};

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

  % array with pinIDs
  pinIDStrings = { 'DIGOUT_PORTA_PIN1',  ...\
                   'DIGOUT_PORTA_PIN8',  ...\
                   'DIGOUT_PORTB_PIN0',  ...\
                   'DIGOUT_PORTB_PIN1',  ...\
                   'DIGOUT_PORTB_PIN2',  ...\
                   'DIGOUT_PORTB_PIN5',  ...\
                   'DIGOUT_PORTB_PIN8',  ...\
                   'DIGOUT_PORTB_PIN9',  ...\
                   'DIGOUT_PORTB_PIN10',  ...\
                   'DIGOUT_PORTB_PIN11',  ...\
                   'DIGOUT_PORTB_PIN12',  ...\
                   'DIGOUT_PORTB_PIN13',  ...\
                   'DIGOUT_PORTB_PIN14',  ...\
                   'DIGOUT_PORTB_PIN15',  ...\
                   'DIGOUT_PORTC_PIN0',  ...\
                   'DIGOUT_PORTC_PIN1',  ...\
                   'DIGOUT_PORTC_PIN2',  ...\
                   'DIGOUT_PORTC_PIN3',  ...\
                   'DIGOUT_PORTC_PIN4',  ...\
                   'DIGOUT_PORTC_PIN5',  ...\
                   'DIGOUT_PORTC_PIN6',  ...\
                   'DIGOUT_PORTC_PIN7',  ...\
                   'DIGOUT_PORTC_PIN8',  ...\
                   'DIGOUT_PORTC_PIN9',  ...\
                   'DIGOUT_PORTC_PIN10',  ...\
                   'DIGOUT_PORTC_PIN11',  ...\
                   'DIGOUT_PORTC_PIN12',  ...\
                   'DIGOUT_PORTC_PIN13',  ...\
                   'DIGOUT_PORTD_PIN2'};
  % construct the pinID string
  pinIDStr = pinIDStrings{SSpin};

  % array with channel infos
  spiChannelInfoStrings = { 'SPI1: UEXT 9 & 7 & 8 (PA5 & PA6 & PA7)',  ...\
                            'SPI2: PB13 & PB14 & PB15'};

  % construct the moduleID info string
  spiChannelInfoStr = spiChannelInfoStrings{channel};

end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('channel', channelStr, 'useSS', int2str(useSS), 'SSpin', pinIDStr, 'SSenable', int2str(SSenable), 'SSdisable', int2str(SSdisable), 'delayCycles', int2str(delayCycles));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);

%%******************************* end of sfcn_spi_master_transfer_mcb.m *****************


