%%***************************************************************************************
%% file         sfcn_timeout_getlasteventcounter_mcb.m
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
function [timeoutPinIdInfoStr] = sfcn_timeout_getlasteventcounter_mcb(hardwareID, pin)

% =============================== Generic ===============================


% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with pin strings
  pinStrings = { 'TIMEOUT_TIM1_PIN_PA8',  ...\
                 'TIMEOUT_TIM1_PIN_PA9',  ...\
                 'TIMEOUT_TIM1_PIN_PA10',  ...\
                 'TIMEOUT_TIM2_PIN_PA0',  ...\
                 'TIMEOUT_TIM2_PIN_PA1',  ...\
                 'TIMEOUT_TIM2_PIN_PA2',  ...\
                 'TIMEOUT_TIM2_PIN_PA3',  ...\
                 'TIMEOUT_TIM3_PIN_PC6',  ...\
                 'TIMEOUT_TIM3_PIN_PC7',  ...\
                 'TIMEOUT_TIM3_PIN_PC8',  ...\
                 'TIMEOUT_TIM4_PIN_PB6',  ...\
                 'TIMEOUT_TIM4_PIN_PB7',  ...\
                 'TIMEOUT_TIM4_PIN_PB8',  ...\
                 'TIMEOUT_TIM4_PIN_PB9' };
        
  % construct the pin string
  pinStr = pinStrings{pin};

  % array with pin infos
  timeoutPinIdInfoStrings = {'TIM1 CON3 - D6',  ...\
                             'TIM1 CON3 - D7',  ...\
                             'TIM1 CON4 - D8',  ...\
                             'TIM2 CON3 - D2',  ...\
                             'TIM2 CON3 - D3',  ...\
                             'TIM2 CON3 - D1',  ...\
                             'TIM2 CON3 - D0',  ...\
                             'TIM3 EXT - D35',  ...\
                             'TIM3 EXT - D36',  ...\
                             'TIM3 EXT - D37',  ...\
                             'TIM4 CON3 - D5',  ...\
                             'TIM4 CON4 - D9',  ...\
                             'TIM4 CON4 - D14',  ...\
                             'TIM4 EXT - D24'};

  % construct the pin info string
  timeoutPinIdInfoStr = timeoutPinIdInfoStrings{pin};

% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with pin strings
  pinStrings = { 'TIMEOUT_TIM1_PIN_PE9',  ...\
                 'TIMEOUT_TIM1_PIN_PE11',  ...\
                 'TIMEOUT_TIM1_PIN_PE13',  ...\
                 'TIMEOUT_TIM1_PIN_PE14',  ...\
                 'TIMEOUT_TIM2_PIN_PA5',  ...\
                 'TIMEOUT_TIM3_PIN_PA6',  ...\
                 'TIMEOUT_TIM3_PIN_PB5',  ...\
                 'TIMEOUT_TIM4_PIN_PD12',  ...\
                 'TIMEOUT_TIM4_PIN_PD13',  ...\
                 'TIMEOUT_TIM8_PIN_PC6',  ...\
                 'TIMEOUT_TIM8_PIN_PC7',  ...\
                 'TIMEOUT_TIM9_PIN_PE5',  ...\
                 'TIMEOUT_TIM9_PIN_PE6',  ...\
                 'TIMEOUT_TIM10_PIN_PF6',  ...\
                 'TIMEOUT_TIM11_PIN_PF7',  ...\
                 'TIMEOUT_TIM13_PIN_PF8',  ...\
                 'TIMEOUT_TIM14_PIN_PF9' };
        
  % construct the pin string
  pinStr = pinStrings{pin};

  % array with pin infos
  timeoutPinIdInfoStrings = {  'TIM1 - CON PE - 12',  ...\
                              'TIM1 - CON PE - 14',  ...\
                              'TIM1 - CON PE - 16',  ...\
                              'TIM1 - CON PE - 17',  ...\
                              'TIM2 - CON4 - D13',  ...\
                              'TIM3 - CON4 - D12',  ...\
                              'TIM3 - CON4 - D11',  ...\
                              'TIM4 - CON PD - 15',  ...\
                              'TIM4 - CON PD - 16',  ...\
                              'TIM8 - UEXT - 3',  ...\
                              'TIM8 - UEXT - 4',  ...\
                              'TIM9 - CON3 - D4',  ...\
                              'TIM9 - CON3 - D5',  ...\
                              'TIM10 - CON PF - 9',  ...\
                              'TIM11 - CON PF - 10',  ...\
                              'TIM13 - CON PF - 11',  ...\
                              'TIM14 - CON PF - 12'};

  % construct the pin info string
  timeoutPinIdInfoStr = timeoutPinIdInfoStrings{pin};
  
% =============================== STM32P405 ===============================
elseif (hardwareID == 3) 
  % array with pin strings
  pinStrings = { 'TIMEOUT_TIM1_PIN_PA8',  ...\
                 'TIMEOUT_TIM1_PIN_PA9',  ...\
                 'TIMEOUT_TIM1_PIN_PA10',  ...\
                 'TIMEOUT_TIM2_PIN_PA1',  ...\
                 'TIMEOUT_TIM3_PIN_PC6',  ...\
                 'TIMEOUT_TIM3_PIN_PC7',  ...\
                 'TIMEOUT_TIM3_PIN_PC8',  ...\
                 'TIMEOUT_TIM3_PIN_PC9',  ...\
                 'TIMEOUT_TIM4_PIN_PB6',  ...\
                 'TIMEOUT_TIM4_PIN_PB7',  ...\
                 'TIMEOUT_TIM4_PIN_PB8',  ...\
                 'TIMEOUT_TIM4_PIN_PB9' };
        
  % construct the pin string
  pinStr = pinStrings{pin};

  % array with pin infos
  timeoutPinIdInfoStrings = { 'TIM1 Con. PA8 (PA8)',  ...\
                              'TIM1 Con. UEXT-3 (PA9)',  ...\
                              'TIM1 Con. UEXT-4 (PA10)',  ...\
                              'TIM2 Con. PA1 (PA1)',  ...\
                              'TIM3 Con. PC6 (PC6)',  ...\
                              'TIM3 Con. PC7 (PC7)',  ...\
                              'TIM3 Con. PC8 (PC8)',  ...\
                              'TIM3 Con. PC9 (PC9)',  ...\
                              'TIM4 Con. UEXT-5 (PB6)',  ...\
                              'TIM4 Con. UEXT-6 (PB7)',  ...\
                              'TIM4 Con. PB8 (PB8)',  ...\
                              'TIM4 Con. PB9 (PB9)' };

  % construct the pin info string
  timeoutPinIdInfoStr = timeoutPinIdInfoStrings{pin};
end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('pin', pinStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_timeout_getrunningcounter_mcb.m ***********



