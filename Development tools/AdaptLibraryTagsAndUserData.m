%%***************************************************************************************
%% file         AdaptLibraryTagsAndUserData.m
%% brief        Change all tags in the library and add the HANcoder structs
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

blocks = find_system(gcs,'LookUnderMasks', 'All','BlockType','M-S-Function')
%Remove the old tags
for i=1:1:length(blocks)
    set_param(blocks{i},'Tag','');
end
%% Done!

%% Set new tags
masks = find_system(gcb,'LookUnderMasks', 'All', 'BlockType', 'SubSystem')
for k=1:1:length(masks)
    set_param(masks{k},'Tag', 'HANcoder_TARGET_STM32_P405')
end
%% Done!
%% Remove new tags on top level subsystems in the library
masks = find_system(gcb,'LookUnderMasks', 'All','SearchDepth', 1, 'BlockType', 'SubSystem')
for k=1:1:length(masks)
    set_param(masks{k},'Tag', '')
end
%% Done!

%% Make all blocks UserData persistent
masks = find_system(gcs,'LookUnderMasks','All','BlockType','SubSystem')
for j=1:1:length(masks)
    set_param(masks{j},'UserDataPersistent','on')
end
%% Done!

%% Adapt mcb files to search for the M-S--Function instead of the Tag
%% HANcoder.... because the Tag has been reset to the mask instead of the
%% S-function....%%Done!






