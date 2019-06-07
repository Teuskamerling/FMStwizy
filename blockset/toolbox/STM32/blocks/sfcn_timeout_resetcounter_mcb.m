%%***************************************************************************************
%% file         sfcn_timeout_resetcounter_mcb.m
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
function [moduleIDInfoStr] = sfcn_timeout_resetcounter_mcb(hardwareID, moduleID)

% =============================== Generic ===============================


% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with moduleIDs
  moduleIDStrings = { 'TIMEOUT_MODULE_TIM1',  ...\
                      'TIMEOUT_MODULE_TIM2',  ...\
                      'TIMEOUT_MODULE_TIM3',  ...\
                      'TIMEOUT_MODULE_TIM4'};
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};

  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1',  ...\
                          'TIM2',  ...\
                          'TIM3',  ...\
                          'TIM4'};
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};

% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with moduleIDs
  moduleIDStrings = { 'TIMEOUT_MODULE_TIM1',  ...\
                      'TIMEOUT_MODULE_TIM2',  ...\
                      'TIMEOUT_MODULE_TIM3',  ...\
                      'TIMEOUT_MODULE_TIM4',  ...\
                      'TIMEOUT_MODULE_TIM8',  ...\
                      'TIMEOUT_MODULE_TIM9',  ...\
                      'TIMEOUT_MODULE_TIM10',  ...\
                      'TIMEOUT_MODULE_TIM11',  ...\
                      'TIMEOUT_MODULE_TIM13',  ...\
                      'TIMEOUT_MODULE_TIM14'};
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};

  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1',  ...\
                          'TIM2',  ...\
                          'TIM3',  ...\
                          'TIM4',  ...\
                          'TIM8',  ...\
                          'TIM9',  ...\
                          'TIM10',  ...\
                          'TIM11',  ...\
                          'TIM13',  ...\
                          'TIM14'};
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};
  
% =============================== STM32P405 ===============================
elseif (hardwareID == 3) 
  % array with moduleIDs
  moduleIDStrings = { 'TIMEOUT_MODULE_TIM1',  ...\
                      'TIMEOUT_MODULE_TIM2',  ...\
                      'TIMEOUT_MODULE_TIM3',  ...\
                      'TIMEOUT_MODULE_TIM4'};
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};

  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1',  ...\
                          'TIM2',  ...\
                          'TIM3',  ...\
                          'TIM4'};
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};
end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('moduleID', moduleIDStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_timeout_resetcounter_mcb.m ****************



