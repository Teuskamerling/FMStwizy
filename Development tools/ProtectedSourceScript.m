%%***************************************************************************************
%% file         ProtectedSourceSript.m
%% brief        This script automatically removes all .c files which need to be protected
% 				and converts the .m files to .p files so the source of the project is 
% 				protected
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

% ! Make sure the current folder is the Target folder, where the model is% located

% check if the library has been built:
if (exist('blockset\rtw\STM32\lib\HANcoder_Olimexino_STM32_RTL.a', 'file') == 2 ...
   && exist('blockset\rtw\STM32\lib\HANcoder_STM32_E407_RTL.a', 'file') == 2 ...
   && exist('blockset\rtw\STM32\lib\HANcoder_STM32_P405_RTL.a', 'file') == 2)
    disp('HANcoder libraries found: proceeding...')
    
    % Removal of .c files from the src/app folder
    [status, message, id] = rmdir('blockset\rtw\STM32\src', 's');
    if status == 0
        disp(message);
    end
    
    % Conversion of .m files to .p files(.p = protected .m file) 
    % blocks folder
    pcode('blockset\toolbox\STM32\blocks','-inplace');
    
    % Delete original m-files from blocks folder except for some m-files
    blocksFolder= 'blockset\toolbox\STM32\blocks\';
    % Get list of files with file extension .m
    mfiles = dir([blocksFolder '*.m']);
    % Make a cell array with the names of the mfiles
    mfileNames ={mfiles.name};
    % Find slblocks.m in this cell array
    slblocksFile = find(strcmp(mfileNames,'slblocks.m'));
    % replace slblocksFile with nothing
    mfileNames{slblocksFile}=[];
    % Find FlashSTM32Target.m in this cell array
    FlashFile = find(strcmp(mfileNames,'FlashSTM32Target.m'));
    % replace slblocksFile with nothing
    mfileNames{FlashFile}=[];
    % Find AddASAP2Elements.m in this cell array
    AddASAP2File = find(strcmp(mfileNames,'AddASAP2Elements.m'));
    % replace slblocksFile with nothing
    mfileNames{AddASAP2File}=[];
    % Find AddASAP2Elements.m in this cell array
    checkGnuArmFile = find(strcmp(mfileNames,'checkGnuArmToolchain.m'));
    % replace slblocksFile with nothing
    mfileNames{checkGnuArmFile}=[];

    % Loop through all (other) mfiles and delete them
    for i=1:numel(mfileNames);
        delete([blocksFolder mfileNames{i}]);
    end
    % Delete slblocks.p, AddASAP2Elements.p, FlashSTM32Target.p & checkGnuArmToolchain.p 
	% (the m-files are still present)
    delete('blockset\toolbox\STM32\blocks\slblocks.p');
    delete('blockset\toolbox\STM32\blocks\AddASAP2Elements.p');
    delete('blockset\toolbox\STM32\blocks\FlashSTM32Target.p');
	delete('blockset\toolbox\STM32\blocks\checkGnuArmToolchain.p');
    % Delete the c-files used to create the mexw32/64 files
    delete('blockset\toolbox\STM32\blocks\sfcn_pwmout_init.c');
    delete('blockset\toolbox\STM32\blocks\sfcn_timein_irq.c');
    delete('blockset\toolbox\STM32\blocks\sfcn_timeout_event_irq.c');
    delete('blockset\toolbox\STM32\blocks\sfcn_timeout_init.c');
    
    clear all
    % Give message
    disp('Please check manually if all source files are deleted');
else
    % Display warning
    disp('Not all libraries found: Stopping procedure');
end
