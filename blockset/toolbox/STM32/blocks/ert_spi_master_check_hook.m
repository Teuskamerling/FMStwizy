%%***************************************************************************************
%% file         ert_spi_master_check_hook.m
%% brief        Function to check the consistency of the SPI master blocks that can be 
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
function ert_spi_master_check_hook(modelName)        
    % check all SPI master blocks that required a SPI master init block to be present
    % for the same channel.
    checkSpiMasterBlock(modelName, 'SPI master transfer');
end


% Function to find all SPI master blocks with a specific mask name and to perform
% a check if there is a SPI master init block for this channel.
function checkSpiMasterBlock(modelName, maskName)
    % loop through all SPI master blocks with this mask name and extract the channel
    num_blocks = length(find_system(modelName, 'MaskType', maskName));
    if num_blocks > 0
        % store blocks in array
        blocks = find_system(modelName, 'MaskType', maskName);
        % loop through all blocks in array
        for i = 1:num_blocks
            % obtain the obtain the channel parameter value
            maskValuesCurrent = get_param(blocks(i), 'MaskValues');
            % extract the uart channel, which is formated as SPIx
            spiCurrent = strtok(maskValuesCurrent{1}{1}, ':');
            % perform check that there is actually a SPI master init block for this channel.
            findSpiMasterInitBlock(modelName, spiCurrent);
        end
    end
end


% Function to loop through all SPI master init blocks to see if there is one for
% the same channel as specified in channelName. This is a string formatted
% like SPIx. This string is a part of the first block parameter. If no
% SPI master init block is found for the channel then an error is triggered.
function findSpiMasterInitBlock(modelName, channelName)
    % check that a SPI master init blocks is present for this channel
    num_init_blocks = length(find_system(modelName, 'MaskType', 'SPI master init'));
    if num_init_blocks > 0
        % init variable to track if we found the init block we are looking for
        init_block_found = 0;
        % now loop through all SPI master init blocks, first store blocks in array
        init_blocks = find_system(modelName, 'MaskType', 'SPI master init');
        % loop through all blocks in array
        for j = 1:num_init_blocks
            % obtain the obtain the channel parameter value
            maskValuesCheck = get_param(init_blocks(j), 'MaskValues');
            % extract the uart channel, which is formated as SPIx
            spiCheck = strtok(maskValuesCheck{1}{1}, ':');
            % check if this init block is for the same channel
            if strcmp(channelName, spiCheck) == 1
                % all ok for this channel, set flag for this
                init_block_found = 1;
            end
        end
        % check if we found the init block for the channel
        if init_block_found ~= 1
            displaySpiMasterInitBlockMissingError(channelName);
        end
    else
        displaySpiMasterInitBlockMissingError(channelName);
    end
end


% Function to display and trigger an error for a specific channel. The
% channel is specified as a string formatted like SPIx, where x is the
% channel number.
function displaySpiMasterInitBlockMissingError(channelName)
    msg = sprintf(['Error: No "SPI master init" block found for channel %s while using other SPI master blocks for this channel.\n', ...
                   'Insert a "SPI master init" block for channel %s to ommit this error message.\n'], channelName, channelName);
    % Display error message in the matlab command window.
    fprintf(msg);
    % Abort and display pop-up window with error message.
    error(msg);           
end

%%******************************* end of ert_spi_master_check_hook.m ********************
