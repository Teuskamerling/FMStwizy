%%***************************************************************************************
%% file         ert_make_rtw_hook.m
%% brief        Hook file with functionality that is invoked during the build process
%%              of an embedded coder real-time target.
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
function ert_make_rtw_hook(hookMethod,modelName,rtwroot,templateMakefile,buildOpts,buildArgs)
% ERT_MAKE_RTW_HOOK - This is the standard ERT hook file for the RTW build
% process (make_rtw), and implements automatic configuration of the
% models configuration parameters.  When the buildArgs option is specified
% as 'optimized_fixed_point=1' or 'optimized_floating_point=1', the model
% is configured automatically for optimized code generation.
%
% This hook file (i.e., file that implements various RTW callbacks) is
% called by RTW for system target file ert.tlc.  The file leverages
% strategic points of the RTW process.  A brief synopsis of the callback
% API is as follows:
%
% ert_make_rtw_hook(hookMethod, modelName, rtwroot, templateMakefile,
%                   buildOpts, buildArgs)
%
% hookMethod:
%   Specifies the stage of the RTW build process.  Possible values are
%   entry, before_tlc, after_tlc, before_make, after_make and exit, etc.
%
% modelName:
%   Name of model.  Valid for all stages.
%
% rtwroot:
%   Reserved.
%
% templateMakefile:
%   Name of template makefile.  Valid for stages 'before_make' and 'exit'.
%
% buildOpts:
%   Valid for stages 'before_make' and 'exit', a MATLAB structure
%   containing fields
%
%   modules:
%     Char array specifying list of generated C files: model.c, model_data.c,
%     etc.
%
%   codeFormat:
%     Char array containing code format: 'RealTime', 'RealTimeMalloc',
%     'Embedded-C', and 'S-Function'
%
%   noninlinedSFcns:
%     Cell array specifying list of non-inlined S-Functions.
%
%   compilerEnvVal:
%     String specifying compiler environment variable value, e.g.,
%     D:\Applications\Microsoft Visual
%
% buildArgs:
%   Char array containing the argument to make_rtw.  When pressing the build
%   button through the Configuration Parameter Dialog, buildArgs is taken
%   verbatim from whatever follows make_rtw in the make command edit field.
%   From MATLAB, it's whatever is passed into make_rtw.  For example, its
%   'optimized_fixed_point=1' for make_rtw('optimized_fixed_point=1').
%
%   This file implements these buildArgs:
%     optimized_fixed_point=1
%     optimized_floating_point=1
%
% You are encouraged to add other configuration options, and extend the
% various callbacks to fully integrate ERT into your environment.

  switch hookMethod
   case 'error'
    % Called if an error occurs anywhere during the build.  If no error occurs
    % during the build, then this hook will not be called.  Valid arguments
    % at this stage are hookMethod and modelName. This enables cleaning up
    % any static or global data used by this hook file.
    disp(['### Real-Time Workshop build procedure for model: ''' modelName...
          ''' aborted due to an error.']);
    
   case 'entry'
    % Called at start of code generation process (before anything happens.)
    % Valid arguments at this stage are hookMethod, modelName, and buildArgs.

    % The following warning must be given if the evaluation version is
    % used, comment for licensed use.
    %uiwait(warndlg('This version is for evaluation purposes only','HANcoder','modal'));
    % The following warning is given for the student version, comment for
    % licensed use.
    %warndlg(sprintf('You are using an educational version of HANcoder\n\nCommercial usage is not allowed in any way\n\nContact hancoder@han.nl for more information'),'HANcoder','modal');
 	
	% Check if the code generation is started from the correct path
	model_path = get_param(bdroot, 'FileName');
	model_path = regexprep(model_path, strcat('\\',modelName,'.slx'),'');% Do for both slx as
	model_path = regexprep(model_path, strcat('\\',modelName,'.mdl'),'');% for mdl files
	
	if(~(strcmp(pwd,model_path)))
		errorMessage = strcat('The current folder is incorrect, please', ...
		' set the current folder to:', model_path);
		error(errorMessage);
    end
    
	% Commented out because Simulink checks this itself, ticket #26
    %% Check if the Simulink Cache Folder and Simulink Code Generation
    %% folder are empty. If not the code generation won't work
    %if ~strcmp(Simulink.fileGenControl('get','CacheFolder'),pwd) || ~strcmp(Simulink.fileGenControl('get','CodeGenFolder'),pwd) 
    %    errorMessage = (['The Simulink Cache folder is not empty. In Simulink go to File->', ...
    %        'Simulink Preferences and then the tab General to remove the cache folder']);
    %    error(errorMessage);
    %end
    
    % Call function to add all variables which are not declared by the
    % user to the workspace as Simulink.Signals/Parameters. This way the
    % missing signals and parameters will become visible in HANtune
    AddASAP2Elements(modelName);
	
    % Search for all file logger signal blocks and create index and first line string  
	% Must be done before tlc
	disp('### Scanning for file logger blocks');
	sfcn_filelogger_scan();
	
	% In MATLAB 2014b & 2015a the buffer passed into LibMdlStartCustomCode 
	% does not appear in the generated code. To fix this Mathworks released
	% a patch. This patch needs to be run before starting the code-generation
	% process 
	% NOTE: This problem seems solved when reinstalling a newly downloaded
	% MATLAB?? from the Mathworks site, the code hereunder gives errors in this
	% 'new' versions
	%if(strcmp(version('-release'),'2014b') || strcmp(version('-release'),'2015a')) 
	%	disableInitializationCodeInIR
	%end

	
    % The following checks make sure that the top level layout of the
    % HANcoder target is correct. If not the build process is canceled
    errorMessage = HANcoderChecks('Hookfile',modelName);
    if (errorMessage == 0)
        % do nothing
    else
        %stop execution of the build process
        error(errorMessage);
    end
    % The following check makes sure nobody tempered with the
    % HANcoderChecks.m file by requesting a secret key
    if (071503416317 == HANcoderChecks('HookfileSecretKey'))
        % do nothing
    else
		%stop execution of the build process
        error('HANcoder error: Original HANcoderChecks m/p-file not found');
    end
    
    fprintf(['\n### Starting Real-Time Workshop build procedure for ', ...
                  'model: %s\n'],modelName);                
    fprintf('### Checking for the use of HANcoder STM32 Target blocks...');
              
    useHANcoderTarget = findHANcoderTarget(modelName);
    
    switch useHANcoderTarget
        case 0
            fprintf('none found, continuing.\n');
        case 1
            fprintf('config block(s) found, checking HANcoder Target blocks for errors and compiling.\n');
    end
      
    if useHANcoderTarget ~= 0
        option = LocalParseArgList(buildArgs);
    
        if ~strcmp(option,'none')
            ert_unspecified_hardware(modelName);
            cs = getActiveConfigSet(modelName);
            cscopy = cs.copy;
            ert_auto_configuration(modelName,option);
            locReportDifference(cscopy, cs);
        end
    end

    
   case 'before_tlc'
    % Called just prior to invoking TLC Compiler (actual code generation.)
    % Valid arguments at this stage are hookMethod, modelName, and
    % buildArgs

    useHANcoderTarget = findHANcoderTarget(modelName);
    
    if useHANcoderTarget ~= 0
        %---[Read and display blockset type and version]--------------------------------
        % Display HANcoder Target blockset type and version information if present
        CheckHANcoderTargetVersionInfo(modelName);
        % Check if all HANcoder blocks have the same version in the
        % UserData struct:
        BlocksFound = CheckHANcoderBlocksetVersion(modelName);
        if(BlocksFound > 0)
		msg = sprintf([int2str(BlocksFound),' Blocks with a different version of the HANcoder blockset are found\n',...
            'Check the Diagnostics Viewer to see which blocks need to be replaced\n']);
            % Abort and display pop-up window with error message.
            error(msg);
		%uiwait(warndlg(sprintf([int2str(BlocksFound),' Blocks with a different version of the HANcoder blockset are found\n',...
        %    'Check the build output (View diagnostics) to see which blocks need to be replaced']),'HANcoder warning'));
        end
        %---[End of Read and display blockset type and version]-------------------------


        %---[Prevent mixing blocks from different HANcoder Target blocksets]----------------
        fprintf('### Checking to prevent blocks from different blockset to be mixed...');
        if (CheckHANcoderTargetBlocksetMixing(modelName) ~= 0)
            fprintf('done\n');
        else
            msg = sprintf(['Error: Blocks from different HANcoder Target blocksets are mixed.\n', ...
                           'Make sure only blocks from one blockset are used.\n']);
            % Display error message
            fprintf(msg);
            % Abort and display pop-up window with error message.
            error(msg);
        end
        %---[End of Prevent mixing blocks from different HANcoder Target blockset]-----------

        %---[Base Sample Time config]---------------------------------------------------
        % This is a check to see if the 'Base Sample Time config' block is
        % used. This is necessary because the 'Base Sample Time config' block 
        % should ALWAYS be used once in every model. If not there will be given
        % an error during compiling. 
        disp('### Checking if the "Base Sample Time config" block is used');
        
        % Search for Digital Input Group block(s).
       sysInitBlock = find_system(modelName, 'MaskType', 'Base Sample Time config');
                        
        if ( isempty(sysInitBlock))
            msg = sprintf(['Error: No "Base Sample Time config" block found.\n', ...
                           'Insert a "Base Sample Time config" block to resolve this error.\n']);
            % Display error message
            fprintf(msg);
            % Abort and display pop-up window with error message.
            error(msg);
        end
        
        %---[End of Base Sample Time config]--------------------------------------------
        
		%---[Custom ID config]---------------------------------------------------
        % This is a check to see if the 'Custom ID config' block is
        % used. This is necessary because the 'Custom ID config' block 
        % should ALWAYS be used once in every model. If not there will be given
        % an error during compiling. 
        disp('### Checking if the "Custom ID config" block is used');
        
        % Search for Digital Input Group block(s).
        sysInitBlock = find_system(modelName, 'MaskType', 'Custom ID config');
                        
        if ( isempty(sysInitBlock))
            msg = sprintf(['Error: No "Custom ID config" block found.\n', ...
                           'Insert a "Custom ID config" block to resolve this error.\n']);
            % Display error message
            fprintf(msg);
            % Abort and display pop-up window with error message.
            error(msg);
        end
        %---[End of Custom ID config]--------------------------------------------
        
     
        %---[Multiple config]---------------------------------------------------
        % Searches for multiple use of config blocks 
        disp('### Checking for multiple use of configuration blocks');
        
        % CAN Config
        if (numel(find_system(modelName, 'MaskType', 'CAN config')) > 1)
             msg = sprintf('Error: Use of multiple ''CAN Config'' blocks.\n');
             % Display error message
             fprintf(msg);
             % Abort and display pop-up window with error message.
             error(msg); 
        % XCP on CAN Config
        elseif (numel(find_system(modelName, 'MaskType', 'XCP on CAN config')) > 1)
             msg = sprintf('Error: Use of multiple ''XCP on CAN config'' blocks.\n');
             % Display error message
             fprintf(msg);
             % Abort and display pop-up window with error message.
             error(msg);
        % XCP on USB Config
        elseif (numel(find_system(modelName, 'MaskType', 'XCP on USB config')) > 1)
             msg = sprintf('Error: Use of multiple ''XCP on USB config'' blocks.\n');
             % Display error message
             fprintf(msg);
             % Abort and display pop-up window with error message.
             error(msg);
        % XCP on TCP/IP Config
        elseif (numel(find_system(modelName, 'MaskType', 'XCP on TCP/IP config')) > 1)
             msg = sprintf('Error: Use of multiple ''XCP on TCP/IP config'' blocks.\n');
             % Display error message in the MATLAB?? command window.
             fprintf(msg);
             % Abort and display pop-up window with error message.
             error(msg);
        % Ethernet Config
        elseif (numel(find_system(modelName, 'MaskType', 'Ethernet config')) > 1)
             msg = sprintf('Error: Use of multiple ''Ethernet config'' blocks.\n');
             % Display error message in the MATLAB?? command window.
             fprintf(msg);
             % Abort and display pop-up window with error message.
             error(msg);
        % Base Sample Time config
        elseif (numel(find_system(modelName, 'MaskType', 'Base Sample Time config')) > 1)
             msg = sprintf('Error: Use of multiple ''Base Sample Time config'' blocks.\n');
             % Display error message in the MATLAB?? command window.
             fprintf(msg);
             % Abort and display pop-up window with error message.
             error(msg);
        end 
        %---[End of Multiple config]-------------------------------------------
        
        
        %---[CAN Config]---------------------------------------------------
        % Search for CAN bus block usage without a CAN configuration block.
        % This will give all sorts of uninitialized function errors during
        % compiling. Warn the user if CAN blocks are used without a 
        % "CAN config" block. (by Florian Eggers)
        % Use receive/send/XCP/low-level commands without enabling selected CAN channel,
        % check for double CAN IDs (CAN send & receive only parameter) and using 
        % reserved CAN IDs if XCP/low-level commands is used on the CAN channel 
        disp('### Checking CAN configuration consistency');
        
        % Search for CAN block(s).
        CC = find_system(modelName, 'MaskType', 'CAN config');
        CR = find_system(modelName, 'MaskType', 'CAN receive');
        CS = find_system(modelName, 'MaskType', 'CAN send');
        CX = find_system(modelName, 'MaskType', 'XCP on CAN config');

        %---[Check for CAN/USB muxing on Olimexino]--------------------------------------
        % build an array with all the blocks that have a Tag starting with HANcoder_TARGET_OLIMEXINO_STM32
        blockArray = find_system(modelName, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'Tag', 'HANcoder_TARGET_OLIMEXINO_STM32');
        % only perform check for the Olimexino
        if (length(blockArray) > 0)
          % check if CAN is used
          if (~isempty(CC))
            UC = find_system(modelName, 'MaskType', 'XCP on USB config');
            if (~isempty(UC))
              msg = sprintf(['Error: USB and CAN blocks detected. The STM32F103 has a hardware limitation.\n', ...
                             'restricting the use of USB and CAN at the same time. Remove either the USB or CAN\n', ...
                             'blocks to omit this error message.\n']);
              % Display error message in the MATLAB?? command window.
              fprintf(msg);
              % Abort and display pop-up window with error message.
              error(msg);           
            end
          end
        end
        %---[End of Check for CAN/USB muxing on Olimexino]-------------------------------
        
        
        % Check use of CAN without CAN config
        if ( isempty(CC) && ( ~isempty(CR) || ~isempty(CS) || ~isempty(CX) ))
            msg = sprintf(['Error: No "CAN config" block found while using other CAN bus blocks.\n', ...
                           'Insert a "CAN config" block and configure the correct CAN bus settings ', ...
                           'to omit this error message.\n']);
            % Display error message in the MATLAB?? command window.
            fprintf(msg);
            % Abort and display pop-up window with error message.
            error(msg);           
        else            
            for i = 1:2
                % Check use of CAN send channel X without enabling it
                if (~isempty(find_system(CS, 'BlockDialogParams', ['CAN ', int2str(i)])) && ~isempty(find_system(CC, ['canBus', int2str(i)], 'off')))
                    msg = sprintf(['Error: Use of CAN bus ', int2str(i), ' is disabled, while (a) CAN send block(s) ' ...
                                   'are found using this bus.\n', ...
                                   'Enable CAN bus ', int2str(i), ' in the "CAN config" block to omit this error message.\n']);
                    % Display error message in the MATLAB?? command window.
                    fprintf(msg);
                    % Abort and display pop-up window with error message.
                    error(msg);
                end
                % Check use of CAN receive channel X without enabling it
                if (~isempty(find_system(CR, 'BlockDialogParams', ['CAN ', int2str(i)])) && ~isempty(find_system(CC, ['canBus', int2str(i)], 'off')))
                    msg = sprintf(['Error: Use of CAN bus ', int2str(i), ' is disabled, while (a) CAN receive block(s) ' ...
                                   'are found using this bus.\n', ...
                                   'Enable CAN bus ', int2str(i), ' in the "CAN config" block to omit this error message.\n']);
                    % Display error message in the MATLAB?? command window.
                    fprintf(msg);
                    % Abort and display pop-up window with error message.
                    error(msg);
                end
                % Check use of XCP on CAN channel X without enabling it
                if (~isempty(find_system(CX, 'BlockDialogParams', ['CAN ', int2str(i)])) && ~isempty(find_system(CC, ['canBus', int2str(i)], 'off')))
                    msg = sprintf(['Error: Use of CAN bus ', int2str(i), ' is disabled, while XCP is set on this bus.\n', ...
                                   'Enable CAN bus ', int2str(i), ' in the "CAN config" block to omit this error message.\n']);
                    % Display error message in the MATLAB?? command window.
                    fprintf(msg);
                    % Abort and display pop-up window with error message.
                    error(msg);
                end
            end
        end
        %---[End of CAN Config]--------------------------------------------

        
        %---[UART Config]--------------------------------------------------
        disp('### Checking UART configuration consistency');
        % Search for UART block usage without a UART init block.
        ert_uart_check_hook(modelName);        
        %---[End of UART Config]-------------------------------------------
        
        %---[SPI Config]---------------------------------------------------
        disp('### Checking SPI configuration consistency');
        % Search for SPI master block usage without a SPI master init block.
        ert_spi_master_check_hook(modelName);        
        %---[End of SPI Config]--------------------------------------------

        %---[UDS Config]---------------------------------------------------
        disp('### Checking UDS configuration consistency');
        % Search for UDS block usage without a UDS init block.
        ert_uds_check_hook(modelName);        
        %---[End of UDS Config]--------------------------------------------
        
        %---[UDP Config]---------------------------------------------------
        disp('### Checking UDP configuration consistency');
        % Perform checks for the UDP related blocks
        ert_udp_check_hook(modelName);        
        %---[End of UDP Config]--------------------------------------------
        
        %---[CANopenNode Config]-------------------------------------------
        disp('### Checking CANopenNode configuration consistency');
        % Check that the init block is only used once and that the associated
        % CAN channel is initialized.
        ert_canopennode_check_hook(modelName);        
        %---[End of CANopenNode Config]------------------------------------

        %---[File Logger Config]-------------------------------------------
        disp('### Checking File Logger configuration consistency');
        % Check correct usage of File Logger blocks.
        ert_filelogger_check_hook(modelName);        
        %---[End of File Logger Config]------------------------------------
        
        
        %---[Ethernet Config]----------------------------------------------
        % Search for Ethernet block usage without an Ethernet configuration block.
        disp('### Checking Ethernet configuration consistency');
        % Search for Ethernet block(s).
        EC = find_system(modelName, 'MaskType', 'Ethernet config');
        EX = find_system(modelName, 'MaskType', 'XCP on TCP/IP config','MaskType', 'FTP server config');
        % Check use of Ethernet without Ethernet config
        if ( isempty(EC) && (~isempty(EX)) )
            msg = sprintf(['Error: No "Ethernet config" block found while using other Ethernet blocks.\n', ...
                           'Insert a "Ethernet config" block and configure the correct Ethernet modules settings ', ...
                           'to omit this error message.\n']);
            % Display error message in the MATLAB?? command window.
            fprintf(msg);
            % Abort and display pop-up window with error message.
            error(msg);           
        end
        %---[End of Ethernet Config]---------------------------------------
    end
        
        
   case 'after_tlc'
    % Called just after to invoking TLC Compiler (actual code generation.)
    % Valid arguments at this stage are hookMethod, modelName, and
    % buildArgs
		%add software version to the SYS_config.h file.
    fprintf('\n### Adding software id to SYS_config.h...\n');
    file = fopen('SYS_config.h', 'a');
    if file == -1{
         error('### failed to open SYS_config.h'); 
         }
    end
    stationID = evalin('base', 'kXcpStationId');
    if numel(stationID) > 255
        msg = sprintf('Error: Station ID is larger than 255 characters. Use a shorter station ID!\n');
             % Display error message in the MATLAB?? command window.
             fprintf(msg);
             % Abort and display pop-up window with error message.
             error(msg); 
    end

    fprintf(file, '#define kXcpStationIdString            "%s"\n', stationID);
    fprintf(file, '#define kXcpStationIdLength            %d\n', numel(stationID));
    
    fclose(file);
    fprintf('### Done adding!...\n');
	
	analogInFilterSearch(modelName);
    
    %--Create Include guard for SYS_config.h--%
    %file = fopen('SYS_config.h','r');
    %buffer = fread(file,
    %fprintf(file,'#ifndef SYS_CONFIG_H\n');
    %fprintf(file,'#define SYS_CONFIG_H\n');
    %fclose(file);
    %file = fopen('SYS_config.h','a');
    %fprintf(file,'#endif');
    %fclose(file);

   case 'before_make'
    % Called after code generation is complete, and just prior to kicking
    % off make process (assuming code generation only is not selected.)  All
    % arguments are valid at this stage.
    % Deleting the old object files and map, bin, elf and srec
    delete('*.obj')

   case 'after_make'
    % Called after make process is complete. All arguments are valid at 
    % this stage.
	
    % Adding the memory addresses to the ASAP2 file
	fprintf('### Post processing ASAP2 file\n');
	ASAP2file = sprintf('%s.a2l', modelName);
	MAPfile = sprintf('..\\%s.map',modelName);
    % Search for Base Sample Time config block
    BaseSampleTimeConfigBlock = find_system(modelName, 'MaskType', 'Base Sample Time config');
    % Get the HANcoderstruct from the block
    HANcoderstruct = get_param(BaseSampleTimeConfigBlock,'UserData');
    % Extract the device name from the struct
    Target = HANcoderstruct{1}.Device;
    % Read the kXcpStationId from the workspace
    stationID = evalin('base', 'kXcpStationId');
    % Read XCP CAN CRO, DTO and baud-rate values
    [XCP_Cro, XCP_Dto, XCP_Baudrate] = getXCP_CANsettings(modelName);
    % Get XCP TCP IP address and Port
    [XCP_TCP_Address, XCP_TCP_Port] = getXCP_TCPsettings(modelName);
    % Post process the ASAP2file
	ASAP2Post(ASAP2file, MAPfile, Target, stationID, XCP_Cro, XCP_Dto, XCP_Baudrate,XCP_TCP_Port, XCP_TCP_Address);
    
    % Delete object files so they will be rebuilt at new build command
    delete('*.obj')
    % Moving this file so the user won't see it in the model directory
    movefile(['../',modelName,'.map'],[modelName,'.map'], 'f') 
    movefile(['../',modelName,'.bin'],[modelName,'.bin'], 'f') 
    movefile(['../',modelName,'.elf'],[modelName,'.elf'], 'f')
    movefile(['../',modelName,'.dump'],[modelName,'.dump'], 'f')
    % Moving the A2L file to the user directory
    movefile([modelName,'.a2l'],['../',modelName,'.a2l'])
    
    % Option to automatically flash the controller
    FlashSTM32Target(modelName);
    
    
    
   case 'exit'
    % Called at the end of the RTW build process.  All arguments are valid
    % at this stage.
    disp(['### Successful completion of Real-Time Workshop build ',...
          'procedure for model: ', modelName]);
    
  end


% Simple parse function to find:
%   optimized_fixed_point=1
%   optimized_floating_point=1
function option = LocalParseArgList(args)
  
  if findstr(args,'optimized_fixed_point=1')
    option = 'optimized_fixed_point';
  elseif findstr(args,'optimized_floating_point=1')
    option = 'optimized_floating_point';
  else
    option = 'none';
  end
end % end of function LocalParseArgList()

% local function: report difference between the configuration set settings
% before and after running auto-configuration script.
function locReportDifference(cs1, cs2)
    [iseq, diffs] = slprivate('diff_config_sets', cs1, cs2, 'string');
    if ~iseq
        msg = sprintf(['You are using an auto-configuring target (help keyword: auto-configuring).  The auto-configuration ', ...
                       'script associated with this target detected incompatible settings ', ...
                       'and automatically updated the following parameters:\n', diffs, '\n']);
        summary = 'Auto-configuration script has modified model settings';
        rtwprivate('rtw_disp_info', get_param(cs2.getModel, 'Name'), summary, msg);
    end
end % end of function locReportDifference()
    
function useHANcoderTarget = findHANcoderTarget(modelName)  
    %check for the use of HANcoder Target blocks    
    blocksMissing = isempty(find_system(modelName, 'MaskType', 'CAN config'));
    blocksMissing = blocksMissing && isempty(find_system(modelName, 'MaskType', 'XCP on CAN config'));
    blocksMissing = blocksMissing && isempty(find_system(modelName, 'MaskType', 'Base Sample Time config'));
        
    % check for 1 of 3 config blocks (if all are not avaible, skip check)
    % none found, skip check
    if blocksMissing        
        useHANcoderTarget = 0;
    % found, check
    else                
        useHANcoderTarget = 1;
    end
end % end of function findHANcoderTarget()

    
% function searches for the Base Sample Time config config block, because this is a unique block
% that should always be present. The blockset type and version string it located in this block's
% description field. When the block is found, the info in the description field is written
% to the command window so the user can check what version of the blockset he is using.
function CheckHANcoderTargetVersionInfo(modelName)  
    % first try to locate the block
    versionInfoBlock = find_system(modelName, 'MaskType', 'Base Sample Time config');

    % make sure we found the block
    if ~isempty(versionInfoBlock)
        fprintf('### Checking for blockset info...');
        blocksetInfo = get_param(versionInfoBlock, 'Description');    
        if ~strcmp('', blocksetInfo)
            fprintf('using %s.\n', blocksetInfo{1});
        else
            fprintf('none found.\n');
        end
    end
end %end of function CheckHANcoderTargetVersionInfo

% function to check if blocks from different HANcoder Target blocksets are used in one model.
% returns 1 if blocks from only one blockset were used, 0 otherwise.
function okay = CheckHANcoderTargetBlocksetMixing(modelName)  
    % build an array with all the blocks that have a Tag starting with HANcoder_TARGET_
    blockArray = find_system(modelName, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'Tag', 'HANcoder_TARGET_.');
    % set default return value 
    okay = 1;
    % only perform check if at least 2 or more HANcoder Target blocks were used
    if (length(blockArray) > 1)
        % create a cell array of all the tag names
        blockTags = get_param(blockArray, 'Tag');
        % iterate through all blocks to make sure the tags match to the first one
        for i = 2 : length(blockTags)
            % check if the current tag is the same as the first one found
            if (strcmp(blockTags{1}, blockTags{i}) == 0)
                % mix from block of different HANcoder Target blocksets encountered
                okay = 0;
                break;
            end
        end
    end
end %end of function CheckHANcoderTargetBlocksetMixing()

function BlocksFound = CheckHANcoderBlocksetVersion(modelName)
%% Check the version in the model
% Version must be updated in this script!
HANcoderVersion = '1.1';
BlocksFound = 0;
try
HANcoderBlocks = find_system(modelName,'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'Tag', 'HANcoder_TARGET_.');
    for index=1:1:length(HANcoderBlocks)
        HANcoderStruct = get_param(HANcoderBlocks{index},'UserData');
        if(isempty(HANcoderStruct) || ~strcmp(HANcoderStruct.BlocksetVersion, HANcoderVersion))
            disp(sprintf(['Warning! ',HANcoderBlocks{index}, ': is not the same version as the blockset']))
            BlocksFound = BlocksFound + 1;
        end
    end
catch
    errordlg('An error with the versioncheck of the blockset has occured','HANcoder error');
end
end % end of function CheckHANcoderBlocksetVersion()

function [XCP_Cro, XCP_Dto, XCP_Baudrate] = getXCP_CANsettings(modelName)
    CANconfig = find_system(modelName, 'MaskType', 'CAN config');
    XCP_CANconfig = find_system(modelName, 'MaskType', 'XCP on CAN config');
    if (~isempty(CANconfig) && ~isempty(XCP_CANconfig)) % Only proceed if these blocks have been found
        % First get the baudrate
        if(strcmp(get_param(XCP_CANconfig,'canBus'),'CAN 1'))
            XCP_Baudrate = get_param(CANconfig,'baudRate1');
            XCP_Baudrate = eval(XCP_Baudrate{1});
        else
            XCP_Baudrate = get_param(CANconfig,'baudRate2');
            XCP_Baudrate = eval(XCP_Baudrate{1});
        end
        % Now get the croID and dtoID
        XCP_Cro = get_param(XCP_CANconfig,'croID');
        XCP_Cro = eval(XCP_Cro{1});
        XCP_Dto = get_param(XCP_CANconfig,'dtoID');
        XCP_Dto = eval(XCP_Dto{1});
    else % If nothing was found the values are set to zero
        XCP_Cro = 0;
        XCP_Dto = 0;
        XCP_Baudrate = 0;
    end
end % end of function getXCP_CANsettings()

function [XCP_TCP_Address, XCP_TCP_Port] = getXCP_TCPsettings(modelName)
    EthernetConfig = find_system(modelName, 'MaskType', 'Ethernet config');
    XCP_TCPconfig = find_system(modelName, 'MaskType', 'XCP on TCP/IP config');
    if (~isempty(EthernetConfig) && ~isempty(XCP_TCPconfig)) % Only proceed if these blocks have been found
        % First get the port
        XCP_TCP_Port = get_param(XCP_TCPconfig,'port');
        XCP_TCP_Port = eval(XCP_TCP_Port{1});
        % Next get the IP address
        XCP_TCP_Address = strcat(get_param(EthernetConfig,'ip1'),'.',get_param(EthernetConfig,'ip2'),'.',get_param(EthernetConfig,'ip3'),'.',get_param(EthernetConfig,'ip4'));
    else % If nothing was found the values are set to zero
        XCP_TCP_Port = 0;
        XCP_TCP_Address = 0;
    end
end % end of function getXCP_TCPsettings()

end %end of function ert_make_rtw_hook()

% LocalWords:  lasterr tlc hookMethod modelName buildArgs RTW disp args findstr
% LocalWords:  LocalParseArgList elseif locReportDifference cs iseq diffs diff
% LocalWords:  slprivate config msg sprintf rtwprivate rtw param getModel

    
%%******************************* end of ert_make_rtw_hook.m ****************************
