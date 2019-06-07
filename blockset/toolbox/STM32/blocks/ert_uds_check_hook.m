%%***************************************************************************************
%% file         ert_uds_check_hook.m
%% brief        Function to check the consistency of the UDS blocks that can be called 
%%              from the ert_make_rtw_hook.m script that runs during the RTW code 
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
function ert_uds_check_hook(modelName)        
    % check all UDS blocks that required a UDS init block to be present
    checkUdsInitBlock(modelName, 'UDS Get Seed');
    checkUdsInitBlock(modelName, 'UDS Set Seed');
    checkUdsInitBlock(modelName, 'UDS Get Seed Event');
    checkUdsInitBlock(modelName, 'UDS Get Key');
    checkUdsInitBlock(modelName, 'UDS Set Key Verified');
    checkUdsInitBlock(modelName, 'UDS Verify Key Event');
    % check all UDS blocks that are only allowed to have one per model
    checkUdsMultipleBlock(modelName, 'UDS Init');
    checkUdsMultipleBlock(modelName, 'UDS Get Seed Event');
    checkUdsMultipleBlock(modelName, 'UDS Verify Key Event');
end


% Function to find all UDS blocks with a specific mask name and to perform
% a check if there is a UDS init block.
function checkUdsInitBlock(modelName, maskName)
    % loop through all UDS blocks with this mask name
    num_blocks = length(find_system(modelName, 'MaskType', maskName));
    if num_blocks > 0
        % this block is used at least once, so check there is a UDS init
        % block present for this channel.
        num_init_blocks = length(find_system(modelName, 'MaskType', 'UDS Init'));
        if num_init_blocks == 0
            msg = sprintf(['Error: No "UDS Init" block found while using other UDS blocks.\n', ...
                           'Insert a "UDS Init" block to ommit this error message.\n']);
            % Display error message in the matlab command window.
            fprintf(msg);
            % Abort and display pop-up window with error message.
            error(msg);           
        end
    end
end


% Function to check if a UDS block with a specific mask name is used more than once
function checkUdsMultipleBlock(modelName, maskName)
    % loop through all UDS blocks with this mask name
    num_blocks = length(find_system(modelName, 'MaskType', maskName));
    if num_blocks > 1
        % this block is used more than once
        msg = sprintf(['Error: More than one "%s" block found. Only one "%s" block per model is allowed.'], maskName, maskName);
        % Display error message in the matlab command window.
        fprintf(msg);
        % Abort and display pop-up window with error message.
        error(msg);           
    end
end


%%******************************* end of ert_uds_check_hook.m ***************************
