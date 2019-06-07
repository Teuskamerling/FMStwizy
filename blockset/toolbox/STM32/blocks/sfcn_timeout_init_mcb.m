%%***************************************************************************************
%% file         sfcn_timeout_init_mcb.m
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
function [moduleIDInfoStr] = sfcn_timeout_init_mcb(hardwareID, moduleID, frequency, enableIRQoutput)

% =============================== Generic ===============================
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

  % make sure the frequency is in a valid range (1099Hz - 72MHz). this is because the prescaler is
  % an 16-bit unsigned value. 1099 is max for a prescaler of 65536 and TIM module operating at
  % full system_speed (72MHz)
  if (frequency < 1099)
      fprintf ('Warning: lowest supported frequency is 1099Hz. This value will be used instead.\n');
      frequency = 1099;
  end;
  if (frequency > 72000000)
      fprintf ('Warning: highest supported frequency is 72MHz. This value will be used instead.\n');
      frequency = 72000000;
  end;

  system_speed = 72000000;
  % compute frequency as int so everything after the decimal point is disregarded.
  prescaler = fix(system_speed / frequency);
  % now compute the actual frequency
  frequency_actual = fix(system_speed / prescaler);
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
  
  % make sure the frequency is in a valid range (2564Hz - 84MHz). this is because the prescaler is
  % an 16-bit unsigned value. The 84MHz is max because some TIM modules operate at system_speed/2.
  % 2564 is max for a prescaler of 65536 and TIM module operating at full system_speed (168MHz)
  if (frequency < 2564)
      fprintf ('Warning: lowest supported frequency is 2564Hz. This value will be used instead.\n');
      frequency = 2564;
  end;
  if (frequency > 84000000)
      fprintf ('Warning: highest supported frequency is 84MHz. This value will be used instead.\n');
      frequency = 84000000;
  end;

  system_speed = 84000000; % it's actually 168MHz but some timer modules can only run at half that.
  % compute frequency as int so everything after the decimal point is disregarded.
  prescaler = fix(system_speed / frequency);
  % now compute the actual frequency
  frequency_actual = fix(system_speed / prescaler);
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

  % make sure the frequency is in a valid range (2564Hz - 84MHz). this is because the prescaler is
  % an 16-bit unsigned value. The 84MHz is max because some TIM modules operate at system_speed/2.
  % 2564 is max for a prescaler of 65536 and TIM module operating at full system_speed (168MHz)
  if (frequency < 2564)
      fprintf ('Warning: lowest supported frequency is 2564Hz. This value will be used instead.\n');
      frequency = 2564;
  end;
  if (frequency > 84000000)
      fprintf ('Warning: highest supported frequency is 84MHz. This value will be used instead.\n');
      frequency = 84000000;
  end;

  system_speed = 84000000; % it's actually 168MHz but some timer modules can only run at half that.
  % compute frequency as int so everything after the decimal point is disregarded.
  prescaler = fix(system_speed / frequency);
  % now compute the actual frequency
  frequency_actual = fix(system_speed / prescaler);
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
modelRTWFields = struct('moduleID', moduleIDStr, 'frequency', int2str(frequency));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_timeout_init_mcb.m ***********



