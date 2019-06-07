%%***************************************************************************************
%% file         sfcn_pwmout_init_mcb.m
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
function [moduleIDInfoStr] = sfcn_pwmout_init_mcb(hardwareID, moduleID, frequency, alignment, enableIRQoutput, ch1On, ch2On, ch3On, ch4On, ch1Pol, ch2Pol, ch3Pol, ch4Pol, ch1Inv, ch2Inv, ch3Inv, ch4Inv)

% =============================== Generic ===============================
  % array with aligments
  alignmentStrings = { 'PWMOUT_EDGE_ALIGNMENT',  ...\
                       'PWMOUT_CENTER_ALIGNMENT'};
  % construct the alignment string
  alignmentStr = alignmentStrings{alignment};

  % array with polarities
  polarityStrings = { 'PWMOUT_ACTIVE_HIGH',  ...\
                      'PWMOUT_ACTIVE_LOW'};
  % construct the polarity strings
  ch1PolStr = polarityStrings{ch1Pol};
  ch2PolStr = polarityStrings{ch2Pol};
  ch3PolStr = polarityStrings{ch3Pol};
  ch4PolStr = polarityStrings{ch4Pol};

  % array with inverted channel configuration
  invertedStrings = { 'PWMOUT_INVERTED_OUTPUT_OFF',  ...\
                      'PWMOUT_INVERTED_OUTPUT_ON'};
  % construct the inverted channel configuration strings
  ch1InvStr = invertedStrings{ch1Inv+1};
  ch2InvStr = invertedStrings{ch2Inv+1};
  ch3InvStr = invertedStrings{ch3Inv+1};
  ch4InvStr = invertedStrings{ch4Inv+1};

  % Before we are able to change the block, we have to de-activate the link
  % of this block to the library.
  % So check if the link status is active:
  if(~strcmp(get_param(gcb,'LinkStatus'),'inactive'))
      % De-activate the library link
    set_param(gcb,'LinkStatus','inactive')
  end

  % Get current number of outputs ('gcb' is a system defined var that points
  % to the tree structure of the current block)
  currentOutputNumber = length (find_system (gcb, 'LookUnderMasks', 'on', 'BlockType', 'Outport'));

  % check if IRQ output should be shown. in this case the termination block 
  % should be replaced by an output block.
  if enableIRQoutput
    if currentOutputNumber ~= 1
      % Get the position of the terminate block
      thisPosition = get_param ([gcb, '/Term1'], 'Position');
      % Remove the terminate block
      delete_block ([gcb, '/Term1']);
      % Add a output block
      add_block ('built-in/Outport', [gcb, '/Out1']);
      % Alligh the new output block to be placed at the position of the
      % removed terminate
      set_param ([gcb, '/Out1'], 'Position', thisPosition);
    end
  % no IRQ output desired, so the output block should be replaced by the
  % termination block.
  else
    if currentOutputNumber ~= 0
      % Get the position of the output block
      thisPosition = get_param ([gcb, '/Out1'], 'Position');
      % Remove the output block
      delete_block ([gcb, '/Out1']);
      % Add a terminate block
      add_block ('built-in/Terminator', [gcb, '/Term1']);
      % Alligh the new terminate block to be placed at the position of the
      % removed output
      set_param ([gcb, '/Term1'], 'Position', thisPosition);
    end
  end

% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with moduleIDs
  moduleIDStrings = { 'PWMOUT_MODULE_TIM1',  ...\
                      'PWMOUT_MODULE_TIM2',  ...\
                      'PWMOUT_MODULE_TIM3',  ...\
                      'PWMOUT_MODULE_TIM4'};
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};

  % array with pinIDs
  pinIDStrings = { 'PWMOUT_TIM1_PIN_PA8',  ...\
                   'PWMOUT_TIM1_PIN_PA9',  ...\
                   'PWMOUT_TIM1_PIN_PA10', ...\
                   'PWMOUT_TIM1_PIN_PA11', ...\
                   'PWMOUT_TIM2_PIN_PA0',  ...\
                   'PWMOUT_TIM2_PIN_PA1',  ...\
                   'PWMOUT_TIM2_PIN_PA2',  ...\
                   'PWMOUT_TIM2_PIN_PA3',  ...\
                   'PWMOUT_TIM3_PIN_PC6',  ...\
                   'PWMOUT_TIM3_PIN_PC7',  ...\
                   'PWMOUT_TIM3_PIN_PC8',  ...\
                   'PWMOUT_TIM3_PIN_PC9',  ...\
                   'PWMOUT_TIM4_PIN_PB6',  ...\
                   'PWMOUT_TIM4_PIN_PB7',  ...\
                   'PWMOUT_TIM4_PIN_PB8',  ...\
                   'PWMOUT_TIM4_PIN_PB9'};
  % construct the pinID string
  ch1IDStr = pinIDStrings{((moduleID-1)*4)+1};
  ch2IDStr = pinIDStrings{((moduleID-1)*4)+2};
  ch3IDStr = pinIDStrings{((moduleID-1)*4)+3};
  ch4IDStr = pinIDStrings{((moduleID-1)*4)+4};

  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1 D6&D7&D8&n/a',  ...\
                          'TIM2 D2&D3&n/a&D0',  ...\
                          'TIM3 D35&D36&D37&n/a',  ...\
                          'TIM4 D5&D9&D14&D24'};
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};

  % make sure the frequency is in a valid range (2Hz - 70312Hz)
  if (frequency < 2)
      fprintf ('Warning: lowest supported frequency is 2Hz. This value will be used instead.\n');
      frequency = 2;
  end;
  if (frequency > 70312)
      fprintf ('Warning: highest supported frequency is 70312Hz. This value will be used instead.\n');
      frequency = 70312;
  end;

  % the prescaler is a 16-bit unsigned value so there are a few restrictions when it comes to the frequency. calculate the 
  % actual frequency here.
  system_speed = 72000000;
  pwm_resolution = 1024;
  % compute frequency as int so everything after the decimal point is disregarded.
  prescaler = fix(system_speed / (frequency * pwm_resolution));
  % now compute the actual frequency
  frequency_actual = fix(system_speed / (prescaler * pwm_resolution));
  % construct the frequency string for RTW
  frequencyStr = int2str(frequency_actual);
  % inform the user if the exact entered frequency is not supported.
  if (frequency ~= frequency_actual)
    fprintf('Warning: desired frequency %d is not supported. %d will be used instead.\n', frequency, frequency_actual);
  end;
  % change the block parameter. note that this only works if the user clicks 'apply' and not 'ok' right away. 
  % either way the actual frequency will be used for the code generation
  set_param(gcb, 'frequency', frequencyStr);

% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with moduleIDs
  moduleIDStrings = { 'PWMOUT_MODULE_TIM1',  ...\
                      'PWMOUT_MODULE_TIM2',  ...\
                      'PWMOUT_MODULE_TIM3',  ...\
                      'PWMOUT_MODULE_TIM4',  ...\
                      'PWMOUT_MODULE_TIM8',  ...\
                      'PWMOUT_MODULE_TIM9',  ...\
                      'PWMOUT_MODULE_TIM10',  ...\
                      'PWMOUT_MODULE_TIM11',  ...\
                      'PWMOUT_MODULE_TIM13',  ...\
                      'PWMOUT_MODULE_TIM14'};
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};

  % TIM1
  if (moduleID == 1)
    ch1IDStr = 'PWMOUT_TIM1_PIN_PE9';
    ch2IDStr = 'PWMOUT_TIM1_PIN_PE11';
    ch3IDStr = 'PWMOUT_TIM1_PIN_PE13';
    ch4IDStr = 'PWMOUT_TIM1_PIN_PE14';
  % TIM2
  elseif (moduleID == 2)
    ch1IDStr = 'PWMOUT_TIM2_PIN_PA5';
    ch2IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch2On = 0;
    ch3On = 0;
    ch4On = 0;
  % TIM3
  elseif (moduleID == 3)
    ch1IDStr = 'PWMOUT_TIM3_PIN_PA6';
    ch2IDStr = 'PWMOUT_TIM3_PIN_PB5';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3On = 0;
    ch4On = 0;
  % TIM4
  elseif (moduleID == 4)
    ch1IDStr = 'PWMOUT_TIM4_PIN_PD12';
    ch2IDStr = 'PWMOUT_TIM4_PIN_PD13';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3On = 0;
    ch4On = 0;
  % TIM8
  elseif (moduleID == 5)
    ch1IDStr = 'PWMOUT_TIM8_PIN_PC6';
    ch2IDStr = 'PWMOUT_TIM8_PIN_PC7';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3On = 0;
    ch4On = 0;
  % TIM9
  elseif (moduleID == 6)
    ch1IDStr = 'PWMOUT_TIM9_PIN_PE5';
    ch2IDStr = 'PWMOUT_TIM9_PIN_PE6';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3On = 0;
    ch4On = 0;
  % TIM10
  elseif (moduleID == 7)
    ch1IDStr = 'PWMOUT_TIM10_PIN_PF6';
    ch2IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch2On = 0;
    ch3On = 0;
    ch4On = 0;
  % TIM11
  elseif (moduleID == 8)
    ch1IDStr = 'PWMOUT_TIM11_PIN_PF7';
    ch2IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch2On = 0;
    ch3On = 0;
    ch4On = 0;
  % TIM13
  elseif (moduleID == 9)
    ch1IDStr = 'PWMOUT_TIM13_PIN_PF8';
    ch2IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch2On = 0;
    ch3On = 0;
    ch4On = 0;
  % TIM14
  elseif (moduleID == 10)
    ch1IDStr = 'PWMOUT_TIM14_PIN_PF9';
    ch2IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch2On = 0;
    ch3On = 0;
    ch4On = 0;
  end;
  
  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1 PE12&PE14&PE16&PE17',  ...\
                          'TIM2 D13&n/a&n/a&n/a',  ...\
                          'TIM3 D12&PB5&n/a&n/a',  ...\
                          'TIM4 PD15&PD16&n/a&n/a',  ...\
                          'TIM8 UEXT3&UEXT4&n/a&n/a',  ...\
                          'TIM9 D4&D5&n/a&n/a',  ...\
                          'TIM10 A1&n/a&n/a&n/a',  ...\
                          'TIM11 A2&n/a&n/a&n/a',  ...\
                          'TIM13 A3&n/a&n/a&n/a',  ...\
                          'TIM14 A4&n/a&n/a&n/a'};
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};
  
  % make sure the frequency is in a valid range (3Hz - 82031Hz)
  if (frequency < 3)
      fprintf ('Warning: lowest supported frequency is 3Hz. This value will be used instead.\n');
      frequency = 3;
  end;
  if (frequency > 82031)
      fprintf ('Warning: highest supported frequency is 82031Hz. This value will be used instead.\n');
      frequency = 82031;
  end;

  % the prescaler is a 16-bit unsigned value so there are a few restrictions when it comes to the frequency. calculate the 
  % actual frequency here.
  system_speed = 84000000; % it's actually 168MHz but some timer modules can only run at half that.
  pwm_resolution = 1024;
  % compute frequency as int so everything after the decimal point is disregarded.
  prescaler = fix(system_speed / (frequency * pwm_resolution));
  % now compute the actual frequency
  frequency_actual = fix(system_speed / (prescaler * pwm_resolution));
  % construct the frequency string for RTW
  frequencyStr = int2str(frequency_actual);
  % inform the user if the exact entered frequency is not supported.
  if (frequency ~= frequency_actual)
    fprintf('Warning: desired frequency %d is not supported. %d will be used instead.\n', frequency, frequency_actual);
  end;
  % change the block parameter. note that this only works if the user clicks 'apply' and not 'ok' right away. 
  % either way the actual frequency will be used for the code generation
  set_param(gcb, 'frequency', frequencyStr);
% =============================== STM32P405 ===============================
elseif (hardwareID == 3) 
  % array with moduleIDs
  moduleIDStrings = { 'PWMOUT_MODULE_TIM1',  ...\
                      'PWMOUT_MODULE_TIM2',  ...\
                      'PWMOUT_MODULE_TIM3',  ...\
                      'PWMOUT_MODULE_TIM4'};
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};

  % TIM1
  if (moduleID == 1)
    ch1IDStr = 'PWMOUT_TIM1_PIN_PA8';
    ch2IDStr = 'PWMOUT_TIM1_PIN_PA9';
    ch3IDStr = 'PWMOUT_TIM1_PIN_PA10';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4On = 0;
  % TIM2
  elseif (moduleID == 2)
    ch1IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch2IDStr = 'PWMOUT_TIM2_PIN_PA1';
    ch3IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch4IDStr = 'CHANNEL_NOT_SUPPORTED';
    ch1On = 0;
    ch3On = 0;
    ch4On = 0;
  % TIM3
  elseif (moduleID == 3)
    ch1IDStr = 'PWMOUT_TIM3_PIN_PC6';
    ch2IDStr = 'PWMOUT_TIM3_PIN_PC7';
    ch3IDStr = 'PWMOUT_TIM3_PIN_PC8';
    ch4IDStr = 'PWMOUT_TIM3_PIN_PC9';
  % TIM4
  elseif (moduleID == 4)
    ch1IDStr = 'PWMOUT_TIM4_PIN_PB6';
    ch2IDStr = 'PWMOUT_TIM4_PIN_PB7';
    ch3IDStr = 'PWMOUT_TIM4_PIN_PB8';
    ch4IDStr = 'PWMOUT_TIM4_PIN_PB9';
  end;
  
  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1 PA8&PA9&PA10&n/a',  ...\
                          'TIM2 n/a&PA1&n/a&n/a',  ...\
                          'TIM3 PC6&PC7&PC8&PC9',  ...\
                          'TIM4 PB6&PB7&PB8&PB9'};
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};
  
  % make sure the frequency is in a valid range (3Hz - 82031Hz)
  if (frequency < 3)
      fprintf ('Warning: lowest supported frequency is 3Hz. This value will be used instead.\n');
      frequency = 3;
  end;
  if (frequency > 82031)
      fprintf ('Warning: highest supported frequency is 82031Hz. This value will be used instead.\n');
      frequency = 82031;
  end;

  % the prescaler is a 16-bit unsigned value so there are a few restrictions when it comes to the frequency. calculate the 
  % actual frequency here.
  system_speed = 84000000; % it's actually 168MHz but some timer modules can only run at half that.
  pwm_resolution = 1024;
  % compute frequency as int so everything after the decimal point is disregarded.
  prescaler = fix(system_speed / (frequency * pwm_resolution));
  % now compute the actual frequency
  frequency_actual = fix(system_speed / (prescaler * pwm_resolution));
  % construct the frequency string for RTW
  frequencyStr = int2str(frequency_actual);
  % inform the user if the exact entered frequency is not supported.
  if (frequency ~= frequency_actual)
    fprintf('Warning: desired frequency %d is not supported. %d will be used instead.\n', frequency, frequency_actual);
  end;
  % change the block parameter. note that this only works if the user clicks 'apply' and not 'ok' right away. 
  % either way the actual frequency will be used for the code generation
  set_param(gcb, 'frequency', frequencyStr);
end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('moduleID', moduleIDStr, 'frequency', frequencyStr, 'alignment', alignmentStr, 'ch1ID', ch1IDStr, 'ch2ID', ch2IDStr, 'ch3ID', ch3IDStr, 'ch4ID', ch4IDStr, 'ch1On', int2str(ch1On), 'ch2On', int2str(ch2On), 'ch3On', int2str(ch3On), 'ch4On', int2str(ch4On), 'ch1Pol', ch1PolStr, 'ch2Pol', ch2PolStr, 'ch3Pol', ch3PolStr, 'ch4Pol', ch4PolStr, 'ch1Inv', ch1InvStr, 'ch2Inv', ch2InvStr, 'ch3Inv', ch3InvStr, 'ch4Inv', ch4InvStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_pwmout_init_mcb.m *************************



