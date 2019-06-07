%%***************************************************************************************
%% file         sfcn_servo_mcb.m
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
function [pinIDInfoStr] = sfcn_servo_mcb(hardwareID, pinID, lowerLimit, upperLimit)

% =============================== Generic ===============================


% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with pinIDs
  pinIDStrings = { 'SERVO_TIM1_PIN_PA8',  ...\
                   'SERVO_TIM1_PIN_PA9',  ...\
                   'SERVO_TIM1_PIN_PA10',  ...\
                   'SERVO_TIM2_PIN_PA0',  ...\
                   'SERVO_TIM2_PIN_PA1',  ...\
                   'SERVO_TIM2_PIN_PA3',  ...\
                   'SERVO_TIM3_PIN_PC6',  ...\
                   'SERVO_TIM3_PIN_PC7',  ...\
                   'SERVO_TIM3_PIN_PC8',  ...\
                   'SERVO_TIM4_PIN_PB6',  ...\
                   'SERVO_TIM4_PIN_PB7',  ...\
                   'SERVO_TIM4_PIN_PB8',  ...\
                   'SERVO_TIM4_PIN_PB9'};
                   
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
  
  % make sure the lower limit pulse width and the upper limit pulse width
  % values are valid.
  if (lowerLimit < 500)
      fprintf ('Warning: lowest supported pulse width is 600 microseconds. This value will be used instead.\n');
      lowerLimit = 500;
  end
  if (upperLimit > 2500)
      fprintf ('Warning: highest supported pulse width is 2400 microseconds. This value will be used instead.\n');
      upperLimit = 2500;
  end
  if(lowerLimit > upperLimit)
      warndlg('The lower limit pulse width cannot be higher than the higher limit pulse width.');
      return;
  end  

  % construct the lower limit string
  lowerLimitStr = int2str(lowerLimit);
  
  % construct the lower limit string
  upperLimitStr = int2str(upperLimit);
  
end

% create resource keywords to be reserved in resource database
modelRTWFields = struct('pinID', pinIDStr, 'lowerLimit', lowerLimitStr, 'upperLimit', upperLimitStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_pwm_out_mcb.m *****************************



