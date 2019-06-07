%%***************************************************************************************
%% file         librarySetup.m
%% brief        Initialization that is automatically executed when a Simulink model in 
%%              the same directory is opened.
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

% First restore the path to factory default
restoredefaultpath;
clear RESTOREDEFAULTPATH_EXECUTED

% add STM32 Target blockset related directories to the MATLAB path
addpath(sprintf('%s\\blockset\\rtw\\STM32', pwd));
addpath(sprintf('%s\\blockset\\toolbox\\STM32\\blocks', pwd));
addpath(sprintf('%s\\blockset\\toolbox\\STM32\\help', pwd));
addpath(sprintf('%s\\blockset\\toolbox\\STM32\\blocks\\tlc_c', pwd));

%% load .mat file for this model
%load(strcat(bdroot,'.mat'));

checkGnuArmToolchain('suppressOutput');

warning off Simulink:SL_LoadMdlParameterizedLink;
warning off Simulink:Commands:LoadMdlParameterizedLink;

%%******************************* end of librarySetup.m **************************************
