%%***************************************************************************************
%% file         sfcn_analog_in_mcb.m
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
function [pinIDInfoStr] = sfcn_analog_in_mcb(hardwareID, pinID, filtered)

% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with pinIDs
  pinIDStrings = { 'ANIN_PORTC_PIN0',  ...\
                   'ANIN_PORTC_PIN1',  ...\
                   'ANIN_PORTC_PIN2',  ...\
                   'ANIN_PORTC_PIN3',  ...\
                   'ANIN_PORTC_PIN4',  ...\
                   'ANIN_PORTC_PIN5',  ...\
                   'ANIN_PORTA_PIN3',  ...\
                   'ANIN_PORTA_PIN2',  ...\
                   'ANIN_PORTA_PIN0',  ...\
                   'ANIN_PORTA_PIN1',  ...\
                   'ANIN_PORTA_PIN4',  ...\
                   'ANIN_PORTA_PIN7',  ...\
                   'ANIN_PORTA_PIN6',  ...\
                   'ANIN_PORTA_PIN5',  ...\
                   'ANIN_PORTB_PIN0',  ...\
                   'ANIN_PORTB_PIN1' };
               
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};

  % array with pinID infos
  pinIDInfoStrings = { 'CON2 - A0',  ...\
                       'CON2 - A1',  ...\
                       'CON2 - A2',  ...\
                       'CON2 - A3',  ...\
                       'CON2 - A4',  ...\
                       'CON2 - A5',  ...\
                       'CON3 - D0',  ...\
                       'CON3 - D1',  ...\
                       'CON3 - D2',  ...\
                       'CON3 - D3/LED2',  ...\
                       'CON4 - D10',  ...\
                       'CON4 - D11',  ...\
                       'CON4 - D12',  ...\
                       'CON4 - D13/LED1',  ...\
                       'EXT - D27',  ...\
                       'EXT - D28' };
                       
  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};
% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with pinIDs
  pinIDStrings = { 'ANIN_PORTF_PIN3',  ...\
                   'ANIN_PORTF_PIN4',  ...\
                   'ANIN_PORTF_PIN5',  ...\
                   'ANIN_PORTF_PIN6',  ...\
                   'ANIN_PORTF_PIN7',  ...\
                   'ANIN_PORTF_PIN8',  ...\
                   'ANIN_PORTF_PIN9',  ...\
                   'ANIN_PORTF_PIN10',  ...\
                   'ANIN_PORTC_PIN0',  ...\
                   'ANIN_PORTC_PIN2' };
               
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};

  % array with pinID infos
  pinIDInfoStrings = { 'CON PF - 6',  ...\
                       'CON PF - 7',  ...\
                       'CON PF - 8',  ...\
                       'CON2 - A1',  ...\
                       'CON2 - A2',  ...\
                       'CON2 - A3',  ...\
                       'CON2 - A4',  ...\
                       'CON2 - A5',  ...\
                       'CON2 - A0',  ...\
                       'UEXT - 7' };
                       
  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};
% =============================== STM32P405 ===============================
elseif (hardwareID == 3) 
  % array with pinIDs
  pinIDStrings = { 'ANIN_PORTA_PIN1',  ...\
                   'ANIN_PORTB_PIN0',  ...\
                   'ANIN_PORTB_PIN1',  ...\
                   'ANIN_PORTC_PIN0',  ...\
                   'ANIN_PORTC_PIN1',  ...\
                   'ANIN_PORTC_PIN2',  ...\
                   'ANIN_PORTC_PIN3',  ...\
                   'ANIN_PORTC_PIN4',  ...\
                   'ANIN_PORTC_PIN5'};
                   
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};

  % array with pinID infos
  pinIDInfoStrings = { 'PA1',  ...\
                       'PB0',  ...\
                       'PB1',  ...\
                       'PC0',  ...\
                       'PC1',  ...\
                       'PC2',  ...\
                       'PC3',  ...\
                       'PC4',  ...\
                       'PC5'};
                       
  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};
end;


% create resource keywords to be reserved in resource database
modelRTWFields = struct('pinID', pinIDStr, 'filtered', int2str(filtered));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_analog_in_mcb.m *************************



