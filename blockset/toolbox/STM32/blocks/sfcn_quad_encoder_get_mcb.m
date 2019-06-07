%%***************************************************************************************
%% file         sfcn_quad_encoder_get_mcb.m
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
function [moduleIDInfoStr] = sfcn_quad_encoder_get_mcb(hardwareID, moduleID, counterReset, config)


% Before we are able to change the block, we have to de-activate the link
% of this block to the library.
if(~strcmp(get_param(gcb,'LinkStatus'),'inactive'))
    % De-activate the library link
	set_param(gcb,'LinkStatus','inactive');
end

% Open the default library (invisible) for use
load_system('built-in');


% handle counter reset input. if counter reset input is not desired then it should internally
% be connected to a GND block, otherwise to an input block.
if (counterReset > 0)
  % input block needed. only add it if not already there
  if ((numel(find_system(gcb, 'LookUnderMasks','all', 'SearchDepth', 1, 'BlockType', 'Inport'))) == 0)
    % a ground block is now expected, let's find it
    if ((numel(find_system(gcb, 'LookUnderMasks','all', 'SearchDepth', 1, 'BlockType', 'Ground'))) > 0)
      % get position of the ground block
      orgPosition = get_param ([gcb, '/Gnd1'], 'Position');
      % remove the ground block
      delete_block ([gcb, '/Gnd1']);
      % add the input block
      add_block ('built-in/Inport', [gcb, '/In1']);
      % place it at the original position of the ground block, this way the line will auto connect
      set_param ([gcb, '/In1'], 'Position', orgPosition);
    end
  end
else
  % ground block needed. only add it if not already there
  if ((numel(find_system(gcb, 'LookUnderMasks','all', 'SearchDepth', 1, 'BlockType', 'Ground'))) == 0)
    % an input block is now expected, let's find it
    if ((numel(find_system(gcb, 'LookUnderMasks','all', 'SearchDepth', 1, 'BlockType', 'Inport'))) > 0)
      % get position of the input block
      orgPosition = get_param ([gcb, '/In1'], 'Position');
      % remove the input block
      delete_block ([gcb, '/In1']);
      % add the ground block
      add_block ('built-in/Ground', [gcb, '/Gnd1']);
      % place it at the original position of the input block, this way the line will auto connect
      set_param ([gcb, '/Gnd1'], 'Position', orgPosition);
    end
  end
end

% =============================== Generic ===============================

% array with configurations
configStrings = { 'QUADENC_CFG_FLOATING',  ...\
                  'QUADENC_CFG_PULLUP',  ...\
                  'QUADENC_CFG_PULLDOWN'};
% construct the config string
configStr = configStrings{config};

% =============================== Olimexino ===============================
if (hardwareID == 1)
  % array with module IDs
  moduleIDStrings = { 'QUADENC_TIM1_PA8_PA9',  ...\
                      'QUADENC_TIM2_PA0_PA1',  ...\
                      'QUADENC_TIM3_PC6_PC7',  ...\
                      'QUADENC_TIM4_PB6_PB7' };
              
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};


  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1 D6&D7',  ...\
                          'TIM2 D2&D3',  ...\
                          'TIM3 D35&D36',  ...\
                          'TIM4 D5&D9'};
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};
% =============================== STM32E407 ===============================
elseif (hardwareID == 2) 
  % array with module IDs
  moduleIDStrings = { 'QUADENC_TIM1_PE9_PE11',  ...\
                      'QUADENC_TIM3_PA6_PB5',  ...\
                      'QUADENC_TIM4_PD12_PD13',  ...\
                      'QUADENC_TIM8_PC6_PC7' };
              
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};


  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1 CON PE - 12 & 14', ...\
                          'TIM3 CON4 - D12 & D11', ...\
                          'TIM4 CON PD - 15 & 16', ...\
                          'TIM8 UEXT - 3 & 4' };
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};
% =============================== STM32P405 ===============================
elseif (hardwareID == 3)
  % array with module IDs
  moduleIDStrings = { 'QUADENC_TIM1_PA8_PA9',  ...\
                      'QUADENC_TIM3_PC6_PC7',  ...\
                      'QUADENC_TIM4_PB6_PB7' };
              
  % construct the moduleID string
  moduleIDStr = moduleIDStrings{moduleID};


  % array with moduleID infos
  moduleIDInfoStrings = { 'TIM1 (PA8&PA9)',  ...\
                          'TIM3 (PC6&PC7)',  ...\
                          'TIM4 (PB6&PB7)'};
  % construct the moduleID info string
  moduleIDInfoStr = moduleIDInfoStrings{moduleID};
end;

% create resource keywords to be reserved in resource database
modelRTWFields = struct('moduleID', moduleIDStr, 'config', configStr);

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);


%%******************************* end of sfcn_quad_encoder_get_mcb.m ********************



