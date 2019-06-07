%%***************************************************************************************
%% file         sfcn_pwmout_setfrequency_mcb.m
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
function [timIDInfoStr] = sfcn_pwmout_setfrequency_mcb(hardwareID, timID)

% =============================== Generic ===============================


% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with pinIDs
  timIDStrings = { 'PWMOUT_MODULE_TIM1',  ...\
                   'PWMOUT_MODULE_TIM2',  ...\
                   'PWMOUT_MODULE_TIM3',  ...\
                   'PWMOUT_MODULE_TIM4'};
                   
  % construct the pinID string
  timIDStr = timIDStrings{timID};

    % array with pinID infos
  timIDInfoStrings = { 'TIM1',  ...\
                       'TIM2',  ...\
                       'TIM3',  ...\
                       'TIM4'} ;
			                       
  % construct the pinID info string
  timIDInfoStr = timIDInfoStrings{timID};

end

% create resource keywords to be reserved in resource database
modelRTWFields = struct('timID', timIDStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_pwm_out_mcb.m *****************************



