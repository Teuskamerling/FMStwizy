%%***************************************************************************************
%% file         sfcn_digital_in_mcb.m
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
function [pinIDInfoStr] = sfcn_digital_in_mcb(hardwareID, pinID, config)

% =============================== Generic ===============================
% array with configurations
configStrings = { 'DIGIN_CFG_FLOATING',  ...\
                  'DIGIN_CFG_PULLUP',  ...\
                  'DIGIN_CFG_PULLDOWN'};
% construct the config string
configStr = configStrings{config};

% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with pinIDs
  pinIDStrings = { 'DIGIN_PORTC_PIN0',  ...\
                   'DIGIN_PORTC_PIN1',  ...\
                   'DIGIN_PORTC_PIN2',  ...\
                   'DIGIN_PORTC_PIN3',  ...\
                   'DIGIN_PORTC_PIN4',  ...\
                   'DIGIN_PORTC_PIN5',  ...\
                   'DIGIN_PORTA_PIN3',  ...\
                   'DIGIN_PORTA_PIN2',  ...\
                   'DIGIN_PORTA_PIN0',  ...\
                   'DIGIN_PORTA_PIN1',  ...\
                   'DIGIN_PORTB_PIN5',  ...\
                   'DIGIN_PORTB_PIN6',  ...\
                   'DIGIN_PORTA_PIN8',  ...\
                   'DIGIN_PORTA_PIN9',  ...\
                   'DIGIN_PORTA_PIN10',  ...\
                   'DIGIN_PORTB_PIN7',  ...\
                   'DIGIN_PORTA_PIN4',  ...\
                   'DIGIN_PORTA_PIN7',  ...\
                   'DIGIN_PORTA_PIN6',  ...\
                   'DIGIN_PORTA_PIN5',  ...\
                   'DIGIN_PORTD_PIN2',  ...\
                   'DIGIN_PORTC_PIN10',  ...\
                   'DIGIN_PORTB_PIN10',  ...\
                   'DIGIN_PORTB_PIN11',  ...\
                   'DIGIN_PORTB_PIN12',  ...\
                   'DIGIN_PORTB_PIN13',  ...\
                   'DIGIN_PORTB_PIN14',  ...\
                   'DIGIN_PORTB_PIN15',  ...\
                   'DIGIN_PORTC_PIN6',  ...\
                   'DIGIN_PORTC_PIN7',  ...\
                   'DIGIN_PORTC_PIN8',  ...\
                   'DIGIN_PORTC_PIN9' };
                   
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};


  % array with pinID infos
  pinIDInfoStrings = {  'CON2 - A0',  ...\
						'CON2 - A1',  ...\
						'CON2 - A2',  ...\
						'CON2 - A3',  ...\
						'CON2 - A4',  ...\
						'CON2 - A5',  ...\
						'CON3 - D0',  ...\
						'CON3 - D1',  ...\
						'CON3 - D2',  ...\
						'CON3 - D3/LED2',  ...\
						'CON3 - D4',  ...\
						'CON3 - D5',  ...\
						'CON3 - D6',  ...\
						'CON3 - D7',  ...\
						'CON4 - D8',  ...\
						'CON4 - D9',  ...\
						'CON4 - D10',  ...\
						'CON4 - D11',  ...\
						'CON4 - D12',  ...\
						'CON4 - D13/LED1',  ...\
						'EXT - D25',  ...\
						'EXT - D26',  ...\
						'EXT - D29',  ...\
						'EXT - D30',  ...\
						'EXT - D31',  ...\
						'EXT - D32',  ...\
						'EXT - D33',  ...\
						'EXT - D34',  ...\
						'EXT - D35',  ...\
						'EXT - D36',  ...\
						'EXT - D37',  ...\
						'BUT'};
  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};

% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with pinIDs
  pinIDStrings = { 'DIGIN_PORTC_PIN0',  ...\
                   'DIGIN_PORTF_PIN6',  ...\
                   'DIGIN_PORTF_PIN7',  ...\
                   'DIGIN_PORTF_PIN8',  ...\
                   'DIGIN_PORTF_PIN9',  ...\
                   'DIGIN_PORTF_PIN10',  ...\
                   'DIGIN_PORTB_PIN7',  ...\
                   'DIGIN_PORTB_PIN6',  ...\
                   'DIGIN_PORTE_PIN2',  ...\
                   'DIGIN_PORTE_PIN4',  ...\
                   'DIGIN_PORTE_PIN5',  ...\
                   'DIGIN_PORTE_PIN6',  ...\
                   'DIGIN_PORTG_PIN7',  ...\
                   'DIGIN_PORTG_PIN8',  ...\
                   'DIGIN_PORTG_PIN12',  ...\
                   'DIGIN_PORTG_PIN15',  ...\
                   'DIGIN_PORTA_PIN4',  ...\
                   'DIGIN_PORTB_PIN5',  ...\
                   'DIGIN_PORTA_PIN6',  ...\
                   'DIGIN_PORTA_PIN5',  ...\
                   'DIGIN_PORTD_PIN0',  ...\
                   'DIGIN_PORTD_PIN1',  ...\
                   'DIGIN_PORTD_PIN2',  ...\
                   'DIGIN_PORTD_PIN3',  ...\
                   'DIGIN_PORTD_PIN4',  ...\
                   'DIGIN_PORTD_PIN5',  ...\
                   'DIGIN_PORTD_PIN6',  ...\
                   'DIGIN_PORTD_PIN7',  ...\
                   'DIGIN_PORTD_PIN8',  ...\
                   'DIGIN_PORTD_PIN9',  ...\
                   'DIGIN_PORTD_PIN10',  ...\
                   'DIGIN_PORTD_PIN11',  ...\
                   'DIGIN_PORTD_PIN12',  ...\
                   'DIGIN_PORTD_PIN13',  ...\
                   'DIGIN_PORTD_PIN14',  ...\
                   'DIGIN_PORTD_PIN15',  ...\
                   'DIGIN_PORTE_PIN0',  ...\
                   'DIGIN_PORTE_PIN1',  ...\
                   'DIGIN_PORTE_PIN3',  ...\
                   'DIGIN_PORTE_PIN7',  ...\
                   'DIGIN_PORTE_PIN8',  ...\
                   'DIGIN_PORTE_PIN9',  ...\
                   'DIGIN_PORTE_PIN10',  ...\
                   'DIGIN_PORTE_PIN11',  ...\
                   'DIGIN_PORTE_PIN12',  ...\
                   'DIGIN_PORTE_PIN13',  ...\
                   'DIGIN_PORTE_PIN14',  ...\
                   'DIGIN_PORTE_PIN15',  ...\
                   'DIGIN_PORTF_PIN0',  ...\
                   'DIGIN_PORTF_PIN1',  ...\
                   'DIGIN_PORTF_PIN2',  ...\
                   'DIGIN_PORTF_PIN3',  ...\
                   'DIGIN_PORTF_PIN4',  ...\
                   'DIGIN_PORTF_PIN5',  ...\
                   'DIGIN_PORTF_PIN11',  ...\
                   'DIGIN_PORTF_PIN12',  ...\
                   'DIGIN_PORTF_PIN13',  ...\
                   'DIGIN_PORTF_PIN14',  ...\
                   'DIGIN_PORTF_PIN15',  ...\
                   'DIGIN_PORTG_PIN0',  ...\
                   'DIGIN_PORTG_PIN1',  ...\
                   'DIGIN_PORTG_PIN2',  ...\
                   'DIGIN_PORTG_PIN3',  ...\
                   'DIGIN_PORTG_PIN4',  ...\
                   'DIGIN_PORTG_PIN5',  ...\
                   'DIGIN_PORTG_PIN6',  ...\
                   'DIGIN_PORTG_PIN9',  ...\
                   'DIGIN_PORTG_PIN10',  ...\
                   'DIGIN_PORTG_PIN11',  ...\
                   'DIGIN_PORTG_PIN13',  ...\
                   'DIGIN_PORTG_PIN14',  ...\
                   'DIGIN_PORTA_PIN0' };

                   
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};

    % array with pinID infos
  pinIDInfoStrings = {	'CON2 - A0', ...\
						'CON2 - A1', ...\
						'CON2 - A2', ...\
						'CON2 - A3', ...\
						'CON2 - A4', ...\
						'CON2 - A5', ...\
						'CON3 - D0', ...\
						'CON3 - D1', ...\
						'CON3 - D2', ...\
						'CON3 - D3', ...\
						'CON3 - D4', ...\
						'CON3 - D5', ...\
						'CON3 - D6', ...\
						'CON3 - D7', ...\
						'CON4 - D8', ...\
						'CON4 - D9', ...\
						'CON4 - D10', ...\
						'CON4 - D11', ...\
						'CON4 - D12', ...\
						'CON4 - D13', ...\
						'CON PD - 3', ...\
						'CON PD - 4', ...\
						'CON PD - 5', ...\
						'CON PD - 6', ...\
						'CON PD - 7', ...\
						'CON PD - 8', ...\
						'CON PD - 9', ...\
						'CON PD - 10', ...\
						'CON PD - 11', ...\
						'CON PD - 12', ...\
						'CON PD - 13', ...\
						'CON PD - 14', ...\
						'CON PD - 15', ...\
						'CON PD - 16', ...\
						'CON PD - 17', ...\
						'CON PD - 18', ...\
						'CON PE - 3', ...\
						'CON PE - 4', ...\
						'CON PE - 6', ...\
						'CON PE - 10', ...\
						'CON PE - 11', ...\
						'CON PE - 12', ...\
						'CON PE - 13', ...\
						'CON PE - 14', ...\
						'CON PE - 15', ...\
						'CON PE - 16', ...\
						'CON PE - 17', ...\
						'CON PE - 18', ...\
						'CON PF - 3', ...\
						'CON PF - 4', ...\
						'CON PF - 5', ...\
						'CON PF - 6', ...\
						'CON PF - 7', ...\
						'CON PF - 8', ...\
						'CON PF - 14', ...\
						'CON PF - 15', ...\
						'CON PF - 16', ...\
						'CON PF - 17', ...\
						'CON PF - 18', ...\
						'CON PG - 3', ...\
						'CON PG - 4', ...\
						'CON PG - 5', ...\
						'CON PG - 6', ...\
						'CON PG - 7', ...\
						'CON PG - 8', ...\
						'CON PG - 9', ...\
						'CON PG - 12', ...\
						'CON PG - 13', ...\
						'CON PG - 14', ...\
						'CON PG - 16', ...\
						'CON PG - 17', ...\
						'WKUP button'};
  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};
% =============================== STM32P405 ===============================
elseif (hardwareID == 3) 
  % array with pinIDs
  pinIDStrings = { 'DIGIN_PORTA_PIN1',  ...\
                   'DIGIN_PORTA_PIN8',  ...\
                   'DIGIN_PORTB_PIN0',  ...\
                   'DIGIN_PORTB_PIN1',  ...\
                   'DIGIN_PORTB_PIN2',  ...\
                   'DIGIN_PORTB_PIN5',  ...\
                   'DIGIN_PORTB_PIN8',  ...\
                   'DIGIN_PORTB_PIN9',  ...\
                   'DIGIN_PORTB_PIN10',  ...\
                   'DIGIN_PORTB_PIN11',  ...\
                   'DIGIN_PORTB_PIN12',  ...\
                   'DIGIN_PORTB_PIN13',  ...\
                   'DIGIN_PORTB_PIN14',  ...\
                   'DIGIN_PORTB_PIN15',  ...\
                   'DIGIN_PORTC_PIN0',  ...\
                   'DIGIN_PORTC_PIN1',  ...\
                   'DIGIN_PORTC_PIN2',  ...\
                   'DIGIN_PORTC_PIN3',  ...\
                   'DIGIN_PORTC_PIN4',  ...\
                   'DIGIN_PORTC_PIN5',  ...\
                   'DIGIN_PORTC_PIN6',  ...\
                   'DIGIN_PORTC_PIN7',  ...\
                   'DIGIN_PORTC_PIN8',  ...\
                   'DIGIN_PORTC_PIN9',  ...\
                   'DIGIN_PORTC_PIN10',  ...\
                   'DIGIN_PORTC_PIN11',  ...\
                   'DIGIN_PORTC_PIN12',  ...\
                   'DIGIN_PORTC_PIN13',  ...\
                   'DIGIN_PORTD_PIN2',  ...\
                   'DIGIN_PORTA_PIN0'};
                   
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};

    % array with pinID infos
  pinIDInfoStrings = { 'PA1',  ...\
                       'PA8',  ...\
                       'PB0',  ...\
                       'PB1',  ...\
                       'PB2',  ...\
                       'PB5',  ...\
                       'PB8',  ...\
                       'PB9',  ...\
                       'PB10',  ...\
                       'PB11',  ...\
                       'PB12',  ...\
                       'PB13',  ...\
                       'PB14',  ...\
                       'PB15',  ...\
                       'PC0',  ...\
                       'PC1',  ...\
                       'PC2',  ...\
                       'PC3',  ...\
                       'PC4',  ...\
                       'PC5',  ...\
                       'PC6',  ...\
                       'PC7',  ...\
                       'PC8',  ...\
                       'PC9',  ...\
                       'PC10',  ...\
                       'PC11',  ...\
                       'PC12',  ...\
                       'PC13',  ...\
                       'PD2',  ...\
                       'PA0 / BUTTON'};

  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};
end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('pinID', pinIDStr, 'config', configStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_digital_in_mcb.m *************************



