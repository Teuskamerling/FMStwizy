%%***************************************************************************************
%% file         sfcn_timein_get_chk.m
%% brief        Block parameter check function.
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
function sfcn_timein_irq_chk()
  % determine the number of timer in IRQ blocks
  num_timer_inblocks = length(find_system(bdroot, 'MaskType', 'Timer Input IRQ'));
  % check if a configured timer module for a Timer Input is not already used in a 
  % quadrature encoder module.
  num_quad_enc_blocks = length(find_system(bdroot, 'MaskType', 'Quadrature Encoder Inputs'));
  % only need to check if there is at least one of each
  if (num_quad_enc_blocks > 0) && (num_timer_inblocks > 0)
    % store blocks in array
    timer_in_blocks = find_system(bdroot, 'MaskType', 'Timer Input IRQ');
    quad_enc_blocks = find_system(bdroot, 'MaskType', 'Quadrature Encoder Inputs');
    % loop through all time in blocks
    for i = 1:num_timer_inblocks
      % obtain the pwm module parameter value
      maskValuesCurrent = get_param(timer_in_blocks(i), 'MaskValues');
      % extract the timer module, which is formated as TIMx
      timerCurrent = strtok(maskValuesCurrent{1}{1}, ':');
      % now loop through the quad encoder blocks to see if this one is already used elsewhere
      for j = 1:num_quad_enc_blocks
        % obtain the pwm module parameter value
        maskValuesCheck = get_param(quad_enc_blocks(j), 'MaskValues');
        % extract the timer module, which is formated as TIMx
        timerCheck = strtok(maskValuesCheck{1}{1}, ':');
        % check if the mask string for the output pin are the same
        if strcmp(timerCurrent, timerCheck) == 1
          message = strcat('Error: A Quadrature Encode Module block for the same timer (', timerCurrent, ') is already present in the model. Select a different timer or remove the Quadrature Encoder Module block.');
          errordlg(message, 'Block error');
          % no need to continue because error was encountered
          return;
        end
      end
    end
  end
  % check if a configured timer module for a Timer Inout is not already used in a 
  % pwm output module.
  num_pwm_out_blocks = length(find_system(bdroot, 'MaskType', 'Pulse Width Modulation Initialization'));
  % only need to check if there is at least one of each
  if (num_pwm_out_blocks > 0) && (num_timer_inblocks > 0)
    % store blocks in array
    timer_in_blocks = find_system(bdroot, 'MaskType', 'Timer Input IRQ');
    pwm_out_blocks = find_system(bdroot, 'MaskType', 'Pulse Width Modulation Initialization');
    % loop through all time in blocks
    for i = 1:num_timer_inblocks
      % obtain the pwm module parameter value
      maskValuesCurrent = get_param(timer_in_blocks(i), 'MaskValues');
      % extract the timer module, which is formated as TIMx
      timerCurrent = strtok(maskValuesCurrent{1}{1}, ':');
      % now loop through the quad encoder blocks to see if this one is already used elsewhere
      for j = 1:num_pwm_out_blocks
        % obtain the pwm module parameter value
        maskValuesCheck = get_param(pwm_out_blocks(j), 'MaskValues');
        % extract the timer module, which is formated as TIMx
        timerCheck = strtok(maskValuesCheck{1}{1}, ':');
        % check if the mask string for the output pin are the same
        if strcmp(timerCurrent, timerCheck) == 1
          message = strcat('Error: A PWM Init block for the same timer (', timerCurrent, ') is already present in the model. Select a different timer or remove the Pulse Width Modulation block.');
          errordlg(message, 'Block error');
          % no need to continue because error was encountered
          return;
        end
      end
    end
  end
  % check if a configured timer module for a Timer Input is not already used in a 
  % Output Compare Init block.
  num_timer_out_blocks = length(find_system(bdroot, 'MaskType', 'Output Compare Init'));
  % only need to check if there is at least one of each
  if (num_timer_out_blocks > 0) && (num_timer_inblocks > 0)
    % store blocks in array
    timer_in_blocks = find_system(bdroot, 'MaskType', 'Timer Input IRQ');
    timer_out_blocks = find_system(bdroot, 'MaskType', 'Output Compare Init');
    % loop through all time in blocks
    for i = 1:num_timer_inblocks
      % obtain the pwm module parameter value
      maskValuesCurrent = get_param(timer_in_blocks(i), 'MaskValues');
      % extract the timer module, which is formated as TIMx
      timerCurrent = strtok(maskValuesCurrent{1}{1}, ':');
      % now loop through the quad encoder blocks to see if this one is already used elsewhere
      for j = 1:num_timer_out_blocks
        % obtain the pwm module parameter value
        maskValuesCheck = get_param(timer_out_blocks(j), 'MaskValues');
        % extract the timer module, which is formated as TIMx
        timerCheck = strtok(maskValuesCheck{1}{1}, ':');
        % check if the mask string for the output pin are the same
        if strcmp(timerCurrent, timerCheck) == 1
          message = strcat('Error: A Output Compare Init block block for the same timer (', timerCurrent, ') is already present in the model. Select a different timer or remove the Output Compare Init block.');
          errordlg(message, 'Block error');
          % no need to continue because error was encountered
          return;
        end
      end
    end
  end
end

%%******************************* end of sfcn_timein_irq_chk.m *****************************
