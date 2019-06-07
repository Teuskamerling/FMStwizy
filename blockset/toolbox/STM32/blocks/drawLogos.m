%%***************************************************************************************
%% file         drawLogos.m
%% brief        Draws the HANcoder logo in the blocks.
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
%% Draw logo's in the HANcoder blocks
function drawLogos
% get the position of the block
block_position = get_param(gcb,'Position');

% calculate the hight and width
block_width = block_position(3)-block_position(1);
block_hight = block_position(4)-block_position(2);

% calculate positions for logo
logo_xpos = num2str(block_width-1);
logo_xpos2 = num2str(block_width-7);
logo_xpos3 = num2str(block_width-9);
logo_xpos4 = num2str(block_width-15);

% make a string for the display
displayString = ['patch([' logo_xpos4 ' ' logo_xpos3 ' ' logo_xpos3 '],[8 14 2],[1 0 0]);patch([' logo_xpos2 ' ' logo_xpos2 ' ' logo_xpos '],[14 2 8],[0 0 0.35]);'];

% % get the mask from the current block (Only works in 2013 and later)
% block_maskobj = Simulink.Mask.get(gcb);
% 
% %get first part of the string, until delimiter. 'patch' is delimiter
% firstPartString = strsplit(block_maskobj.Display, 'patch'); %strsplit only works in 2013 and later
% 
% % update display
% block_maskobj.Display = strcat(firstPartString{1},displayString);

%Get mask text from the mask
maskText = get_param(gcb,'MaskDisplay');

%Split masktext using regexp
firstPartString = regexp(maskText, 'patch', 'split');

%Build new mask text
newMaskText = strcat(firstPartString{1},displayString);

% Set the new mask text in the mask
set_param(gcb,'MaskDisplay', newMaskText);

% Redraw mask (don't know if this is neccessary)
set_param(gcb,'MaskRunInitForIconRedraw','on');
end
