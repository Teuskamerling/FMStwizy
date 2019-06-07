%%***************************************************************************************
%% file         ert_filelogger_check_hook.m
%% brief        Function to check the consistency of the File Logger blocks that can be 
%%              called from the ert_make_rtw_hook.m script that runs during the RTW code 
%%              generation process.
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
function ert_filelogger_check_hook(modelName)        
  % check if there are any 'File Logger Signal' or 'File Logger Close'
  % blocks in the model.
  fileLoggerNonInitBlocks = getFileLoggerNumOfBlocks(modelName, 'File Logger Signal') + ...
                            getFileLoggerNumOfBlocks(modelName, 'File Logger Close');
  fileLoggerInitBlocks = getFileLoggerNumOfBlocks(modelName, 'File Logger Init');
  fileLoggerCloseBlocks = getFileLoggerNumOfBlocks(modelName, 'File Logger Close');
  % if there are any, then check that there is also a 'File Logger Init Block'
  if fileLoggerNonInitBlocks > 0
     if fileLoggerInitBlocks == 0
         % Build error message
         msg = sprintf(['Error: No "File Logger Init" block found while using other File Logger blocks.\n', ...
                       'Insert a "File Logger Init" block to ommit this error message.\n']);
         % Display error message in the matlab command window.
         fprintf(msg);
         % Abort and display pop-up window with error message.
         error(msg);           
     end
  end

  % check that there is not more than one 'File Logger Init' block in the
  % model.
  if fileLoggerInitBlocks > 1
     % Build error message
     msg = sprintf(['Error: Too many "File Logger Init" blocks found. Only one is allowed.\n', ...
                   'Remove the unwanted "File Logger Init" blocks to ommit this error message.\n']);
     % Display error message in the matlab command window.
     fprintf(msg);
     % Abort and display pop-up window with error message.
     error(msg);           
  end
  
  % check that there is also at least one 'File Logger Close' block present
  % in the model when a 'File Logger Init' block is present.
  if fileLoggerInitBlocks == 1
      if fileLoggerCloseBlocks == 0
         % Build error message
         msg = sprintf(['Error: No "File Logger Close" block found while "File Logger Init" block is present.\n', ...
                       'Add a "File Logger Close" block to ommit this error message.\n']);
         % Display error message in the matlab command window.
         fprintf(msg);
         % Abort and display pop-up window with error message.
         error(msg);           
      end
  end
end


% Function to find out how many blocks with a specific mask name are present in the
% model.
function [numOfBlocks] = getFileLoggerNumOfBlocks(modelName, maskName)
    % find the number of blocks
    numOfBlocks = length(find_system(modelName, 'MaskType', maskName));
end


%%******************************* end of ert_filelogger_check_hook.m ********************
