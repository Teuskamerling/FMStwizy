%%***************************************************************************************
%% file         sfcn_pwmout_setduty_mcb.m
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
function [pinIDInfoStr] = sfcn_pwmout_setduty_mcb(hardwareID, pinID)

% =============================== Generic ===============================


% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with pinIDs
  pinIDStrings = { 'PWMOUT_TIM1_PIN_PA8',  ...\
                   'PWMOUT_TIM1_PIN_PA9',  ...\
                   'PWMOUT_TIM1_PIN_PA10',  ...\
                   'PWMOUT_TIM2_PIN_PA0',  ...\
                   'PWMOUT_TIM2_PIN_PA1',  ...\
                   'PWMOUT_TIM2_PIN_PA3',  ...\
                   'PWMOUT_TIM3_PIN_PC6',  ...\
                   'PWMOUT_TIM3_PIN_PC7',  ...\
                   'PWMOUT_TIM3_PIN_PC8',  ...\
                   'PWMOUT_TIM4_PIN_PB6',  ...\
                   'PWMOUT_TIM4_PIN_PB7',  ...\
                   'PWMOUT_TIM4_PIN_PB8',  ...\
                   'PWMOUT_TIM4_PIN_PB9'};
                   
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};

    % array with pinID infos
  pinIDInfoStrings = { 'TIM1 - CON3 - D6',  ...\
                       'TIM1 - CON3 - D7',  ...\
                       'TIM1 - CON4 - D8',  ...\
                       'TIM2 - CON3 - D2',  ...\
                       'TIM2 - CON3 - D3',  ...\
                       'TIM2 - CON3 - D0',  ...\
                       'TIM3 - EXT - D35',  ...\
                       'TIM3 - EXT - D36',  ...\
                       'TIM3 - EXT - D37',  ...\
                       'TIM4 - CON3 - D5',  ...\
                       'TIM4 - CON4 - D9',  ...\
                       'TIM4 - CON4 - D14',  ...\
                       'TIM4 - EXT - D24'} ;
			                       
  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};

% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with pinIDs
  pinIDStrings = { 'PWMOUT_TIM1_PIN_PE9',  ...\
                   'PWMOUT_TIM1_PIN_PE11',  ...\
                   'PWMOUT_TIM1_PIN_PE13',  ...\
                   'PWMOUT_TIM1_PIN_PE14',  ...\
                   'PWMOUT_TIM2_PIN_PA5',  ...\
                   'PWMOUT_TIM3_PIN_PA6',  ...\
                   'PWMOUT_TIM3_PIN_PB5',  ...\
                   'PWMOUT_TIM4_PIN_PD12',  ...\
                   'PWMOUT_TIM4_PIN_PD13',  ...\
                   'PWMOUT_TIM8_PIN_PC6',  ...\
                   'PWMOUT_TIM8_PIN_PC7',  ...\
                   'PWMOUT_TIM9_PIN_PE5',  ...\
                   'PWMOUT_TIM9_PIN_PE6',  ...\
                   'PWMOUT_TIM10_PIN_PF6',  ...\
                   'PWMOUT_TIM11_PIN_PF7',  ...\
                   'PWMOUT_TIM13_PIN_PF8',  ...\
                   'PWMOUT_TIM14_PIN_PF9'};
                   
                   
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};

    % array with pinID infos
  pinIDInfoStrings = { 'CON PE - 12',  ...\
                       'CON PE - 14',  ...\
                       'CON PE - 16',  ...\
                       'CON PE - 17',  ...\
                       'CON4 - D13',  ...\
                       'CON4 - D12',  ...\
                       'CON4 - D11',  ...\
                       'CON PD - 15',  ...\
                       'CON PD - 16',  ...\
                       'UEXT - 3',  ...\
                       'UEXT - 4',  ...\
                       'CON3 - D4',  ...\
                       'CON3 - D5',  ...\
                       'CON2 - A1',  ...\
                       'CON2 - A2',  ...\
                       'CON2 - A3',  ...\
                       'CON2 - A4'};
                       
                       
  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};
% =============================== STM32P405 ===============================
elseif (hardwareID == 3) 
  % array with pinIDs
  pinIDStrings = { 'PWMOUT_TIM1_PIN_PA8',  ...\
                   'PWMOUT_TIM1_PIN_PA9',  ...\
                   'PWMOUT_TIM1_PIN_PA10',  ...\
                   'PWMOUT_TIM2_PIN_PA1',  ...\
                   'PWMOUT_TIM3_PIN_PC6',  ...\
                   'PWMOUT_TIM3_PIN_PC7',  ...\
                   'PWMOUT_TIM3_PIN_PC8',  ...\
                   'PWMOUT_TIM3_PIN_PC9',  ...\
                   'PWMOUT_TIM4_PIN_PB6',  ...\
                   'PWMOUT_TIM4_PIN_PB7',  ...\
                   'PWMOUT_TIM4_PIN_PB8',  ...\
                   'PWMOUT_TIM4_PIN_PB9'};
                   
  % construct the pinID string
  pinIDStr = pinIDStrings{pinID};

    % array with pinID infos
  pinIDInfoStrings = { 'PA8',  ...\
                       'PA9',  ...\
                       'PA10',  ...\
                       'PA1',  ...\
                       'PC6',  ...\
                       'PC7',  ...\
                       'PC8',  ...\
                       'PC9',  ...\
                       'PB6',  ...\
                       'PB7',  ...\
                       'PB8',  ...\
                       'PB9'};
                       
  % construct the pinID info string
  pinIDInfoStr = pinIDInfoStrings{pinID};
end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('pinID', pinIDStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_pwm_out_mcb.m *****************************



