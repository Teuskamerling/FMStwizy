%%***************************************************************************************
%% file         sfcn_digital_out_chk.m
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
function sfcn_digital_out_chk()
  % determine the number of digital output blocks
  num_blocks = length(find_system(bdroot, 'MaskType', 'Digital Output'));
  % only need to run check if there are more than 1 
  if num_blocks > 1
    % store blocks in array
    blocks = find_system(bdroot, 'MaskType', 'Digital Output');
    % loop through all blocks
    for i = 1:num_blocks
      % obtain the output pin parameter value
      maskValuesCurrent = get_param(blocks(i), 'MaskValues');
      outputPinCurrent = maskValuesCurrent{1}{1};
      % now loop through the blocks again to see if this one is already used elsewhere
      for j = 1:num_blocks
        if j ~= i
          % obtain the output pin parameter value
          maskValuesCheck = get_param(blocks(j), 'MaskValues');
          outputPinCheck = maskValuesCheck{1}{1};
          % check if the mask string for the output pin are the same
          if strcmp(outputPinCurrent, outputPinCheck) == 1
            message = strcat('Error: A Digital Output block for the same pin (', outputPinCurrent, ') is already present in the model. Select a different pin or remove the duplicate block.');
            errordlg(message, 'Block error');
            % no need to continue because error was encountered
            return;
          end
        end
      end
    end
  end
end

%%******************************* end of sfcn_digital_out_chk.m *************************
