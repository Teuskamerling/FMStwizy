%%***************************************************************************************
%% file         ert_canopennode_check_hook.m
%% brief        Function to check the consistency of the CANopenNode blocks that can be 
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
function ert_canopennode_check_hook(modelName)        
    % check all CANopenNode blocks that required a CANopenNode Init block to be present
    checkCANopenNodeInitBlock(modelName, 'CANopenNode OD Read');
    checkCANopenNodeInitBlock(modelName, 'CANopenNode OD Write');

    % check all CANopenNode blocks that are only allowed to have one per model
    checkCANopenNodeMultipleBlock(modelName, 'CANopenNode Init');
    
    % check that the CAN channel is initialized
    checkCANopenNodeInitHasCanInit(modelName);
end

% Function to check if a CAN channel is actually configured for the CAN channel used
% in the CANopenNode Init block.
function checkCANopenNodeInitHasCanInit(modelName)
    % only need to do this check if CANopen is actually used
    if length(find_system(modelName, 'MaskType', 'CANopenNode Init')) > 0
        % first check that there is actually a CAN config block in the system
        num_can_config_blocks = length(find_system(modelName, 'MaskType', 'CAN config'));
        if num_can_config_blocks == 0
            msg = sprintf(['Error: No "CAN config" block found while using other CANopenNode blocks.\n', ...
                           'Insert a "CAN config" block to ommit this error message.\n']);
            % Display error message in the matlab command window.
            fprintf(msg);
            % Abort and display pop-up window with error message.
            error(msg);           
        else
            % get handle to the CAN config block. ther should only be one
            can_config_blocks = find_system(modelName, 'MaskType', 'CAN config');
            can_config_block = can_config_blocks(1);
            % check for CANopenNode Init blocks
            num_init_blocks = length(find_system(modelName, 'MaskType', 'CANopenNode Init'));
            % only one init block is allowed, so only continue if that is the case
            if num_init_blocks == 1
                % get handle to the init block
                init_blocks = find_system(modelName, 'MaskType', 'CANopenNode Init');
                init_block = init_blocks(1);
                % get used CAN channel. This is formatted as 'CAN x', where x is 1..n.
                used_can_channel = strrep(get_param(init_block, 'channel'), 'CAN ', '');
                % check checkbox setting for this channel, assuming that max 5
                % channel can ever be present
                can_channel_enabled = 'off';
                if strcmp(used_can_channel, '1')
                    can_channel_enabled = get_param(can_config_block, 'canBus1');
                elseif strcmp(used_can_channel, '2')
                    can_channel_enabled = get_param(can_config_block, 'canBus2');
                elseif strcmp(used_can_channel, '3')
                    can_channel_enabled = get_param(can_config_block, 'canBus3');
                elseif strcmp(used_can_channel, '4')
                    can_channel_enabled = get_param(can_config_block, 'canBus4');
                elseif strcmp(used_can_channel, '5')
                    can_channel_enabled = get_param(can_config_block, 'canBus5');
                end
                % no verify that the channel is enabled
                if ~strcmp(can_channel_enabled, 'on')
                    msg = sprintf(['Error: The CAN channel selected in the "CANopenNode Init" block is \n', ...
                                   'not enabled in the "CAN config" block.\n']);
                    % Display error message in the matlab command window.
                    fprintf(msg);
                    % Abort and display pop-up window with error message.
                    error(msg);           
                end
            end
        end
    end
end

% Function to find all CANopenNode blocks with a specific mask name and to perform
% a check if there is a CANopenNode Init block.
function checkCANopenNodeInitBlock(modelName, maskName)
    % loop through all CANopenNode blocks with this mask name
    num_blocks = length(find_system(modelName, 'MaskType', maskName));
    if num_blocks > 0
        % this block is used at least once, so check there is a CANopenNode Init
        % block present for this channel.
        num_init_blocks = length(find_system(modelName, 'MaskType', 'CANopenNode Init'));
        if num_init_blocks == 0
            msg = sprintf(['Error: No "CANopenNode Init" block found while using other CANopenNode blocks.\n', ...
                           'Insert a "CANopenNode Init" block to ommit this error message.\n']);
            % Display error message in the matlab command window.
            fprintf(msg);
            % Abort and display pop-up window with error message.
            error(msg);           
        end
    end
end


% Function to check if a CANopenNode block with a specific mask name is used more than once
function checkCANopenNodeMultipleBlock(modelName, maskName)
    % loop through all CANopenNode blocks with this mask name
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


%%******************************* end of ert_canopennode_check_hook.m *******************
