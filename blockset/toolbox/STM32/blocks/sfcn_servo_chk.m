%%***************************************************************************************
%% file         sfcn_servo_chk.m
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
function sfcn_servo_chk()

  % determine the number of servo blocks
  num_blocks = length(find_system(bdroot, 'MaskType', 'Servo'));

  % only need to run check if there are more than 1 
  if num_blocks > 1
      
      % get the servo blocks in an array
      servo_blocks = find_system(bdroot, 'MaskType', 'Servo');      
      
      % loop all servo blocks
      for i = 1:num_blocks
          
          % get the selected pinID
          maskValuesCurrent = get_param(servo_blocks(i), 'MaskValues');
          pinIdCurrent = maskValuesCurrent{1}{1};
          cnt=0;
          
          % count all servo blocks that use the same pinID
          for j = 1:num_blocks
             
              maskValuesAll = get_param(servo_blocks(j), 'MaskValues');
              pinId = maskValuesAll{1}{1};
              
              if(strcmp(pinIdCurrent, pinId) == 1)
                  cnt = cnt + 1;
              end
          end
          
          % if there is more than one servo block with the same pinID,
          % generate an error
          if(cnt > 1)
              message = strcat('The pin (', pinIdCurrent, ') is already in use');
              errordlg(message, 'Block error');
              return;
          end
          
      end
  end
end

%%******************************* end of sfcn_servo_chk.m *****************************
