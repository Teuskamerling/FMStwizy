%%***************************************************************************************
%% file         DetectLinesAndGiveNames.m
%% brief        Automatically detects lines and gives names to them.
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
Gains=find_system(gcs,'FindAll','On','LookUnderMasks','All','FollowLinks','On','BlockType','Gain');
% Loop through signals
for index = 1:length(Gains)
    phandle = get_param(Gains(index),'PortHandles');
    line = get_param(phandle.Outport, 'Line');
    if(index < 10)
        lineName = ['SIGS_Counter_00',int2str(index)];
    elseif (index < 100)
        lineName = ['SIGS_Counter_0',int2str(index)];
    else
        lineName = ['SIGS_Counter_',int2str(index+288)];
    end
    set_param(line,'Name',lineName);
    % Create Simulink.Signal in workspace 
    assignin('base', lineName, Simulink.Signal);
    evalin('base', strcat(lineName,'.StorageClass = ''ExportedGlobal'';'));
end
clear index phandle line lineName Gains