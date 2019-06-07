%%***************************************************************************************
%% file         checkGnuArmToolchain.m
%% brief        Automatic check to see if the compiler is installed.
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
%% Check to see if compiler is installed by checking the path
function checkGnuArmToolchain(command)
[status,cmdResponse] = system('arm-none-eabi-gcc');
if isempty(strfind(cmdResponse, 'compilation'))
    [status,EnvironmentVariable] = system('echo %PATH%');
    if isempty(strfind(EnvironmentVariable, 'GNU Tools ARM Embedded'))
        fprintf(['Could not find path to ''GNU ARM Embedded Toolchain'' ',...
            'in the windows\nenvironment variables, please check if the tools\n',...
            'are properly installed and the path is set correctly.\n',...
            'GNU ARM Embedded Toolchain can be downloaded here: <a href="https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads">GNU ARM Tools</a>.\n'])
    else
        disp('### Path to GNU ARM Embedded Toolchain found in windows environment variables...')
        disp('  but the tools are inresponsive. Consider reinstalling the toolchain.')
        disp('  It can be downloaded here: <a href="https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads">GNU ARM Tools</a>.')
    end
else
	if ~strcmp(command,'suppressOutput')
		disp('### GNU ARM toolchain installed correctly')
	end
end