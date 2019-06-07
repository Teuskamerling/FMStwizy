%%***************************************************************************************
%% file         UpdateVersionInLibrary.m
%% brief        Change the version in the HANcoder struct to the next version.
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

HANcoderBlocks = find_system(gcs,'RegExp', 'on', 'LookUnderMasks', 'All', 'Tag', 'HANcoder_TARGET_.');
for index=1:1:length(HANcoderBlocks)
    HANcoderStruct = get_param(HANcoderBlocks{index},'UserData');
    HANcoderStruct.BlocksetVersion = '1.1';
    set_param(HANcoderBlocks{index}, 'UserData', HANcoderStruct);
end

% Change the descriptions in the Base Sample Time blocks (Run Separate for each target!)
BaseSampleTimeConfigBlocks = find_system(gcs, 'MaskType', 'Base Sample Time config');
for BSTindex=1:1:length(BaseSampleTimeConfigBlocks)
    set_param(BaseSampleTimeConfigBlocks{BSTindex},'Description', 'HANcoder Target STM32-Olimexino blockset version 1.0'); %Don't forget to change the version here aswell!
end
% !! NOTE!! Version must also be updated in the hookfile & the version of the library itself should match the blockset version <-----
% (name:HANcoderVersion)

% For new library blocks the Forwarding Table can be used. This way you can run scripts to change the blocks in the model.