%%***************************************************************************************
%% file         sfcn_filelogger_scan.m
%% brief        Level-2 M file S-Function for finding and registering File Logger signals
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
function sfcn_filelogger_scan
FileLoggerString = 'time;'; 	% Declare start of first line string
cellArray = {''}; 				% Declare empty cell array to hold the signal names
elementIdx = -1; 				% Set Idx to minus one so the first element will be placed at 0
manualColumnDefinition = 0; 	% Variable to check if any columns are manually defined
maxSignals = 0;                 % Variable to count the number of signals to be logged

% TODO Jason: Added a simple check here to prevent an error when no File Logger blocks are being
% used. This should be improved!
if isempty(find_system(gcs,'LookUnderMasks', 'all','MaskType', 'File Logger Init'))
	return; % Don't do any checks if the File Logger Init Block isn't present
end

% Find all HANcoder blocks in the model
HANcoderBlocks = find_system(gcs,'LookUnderMasks', 'all', 'regexp', 'on','Tag','HANcoder_');
for i=1:length(HANcoderBlocks)
    % Get the HANcoder struct from the blocks
    HANcoderStruct = get_param(HANcoderBlocks{i},'UserData');
    % Check if the block is a File Logger Signal block
    if(strcmp(HANcoderStruct.BlockType,'FileLoggerSignal'))
        % Add signal count
        maxSignals = maxSignals + 1;
        % Get signal name
        signalName = get_param(HANcoderBlocks{i},'signalName');
        % Check name
        if (strcmp(signalName,'enter name'))
            warndlg(['No name specified for signal in:',HANcoderBlocks{i}]);
        end
        % If the column is not indicated by user let this script give it
        if strcmp('off', get_param(HANcoderBlocks{i},'defineColumns')) % Columns not defined manually
            elementIdx = elementIdx + 1;
            if (manualColumnDefinition > 0)
                error(sprintf(['If one column number of a log signal is defined, ',...
                    'all columns of all signals have to be defined! ',...
                    'Column is not defined in:\n"',HANcoderBlocks{i}]));
                break;
            end
        else % Columns defined manually
            % Indicate that manual definition has been found
            manualColumnDefinition = 1;
            % Else take the user defined column number
            userDefinedIdx = str2num(get_param(HANcoderBlocks{i},'columnNr'));
            elementIdx = userDefinedIdx - 1; % Minus 1 to start at 0
        end
        
        % Place the name in the appropriate cell in the cell array
        cellArray{elementIdx+1} = signalName;
        
        % Get the data type
        dataType = get_param(HANcoderBlocks{i},'dataType');
        % Change data type string
        dataTypeStr = getDataTypeString(dataType);
        % Build the RTWdata struct
        modelRTWFields = struct('elementIdx',int2str(elementIdx),'dataType',dataTypeStr);
        % Find the s-function under the mask
        SfunctionBlock = find_system(HANcoderBlocks{i}, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
        set_param(SfunctionBlock{1}, 'RTWdata', modelRTWFields);
        
    elseif strcmp(HANcoderStruct.BlockType, 'FileLoggerInit')
        % Save the location of the file logger init block
        FileLoggerInitBlock = HANcoderBlocks{i};
    end
end % End of for loop

% After the for loop construct the FileLoggerString with another for-loop
for j=1:length(cellArray)
    if isempty(cellArray{j})
        error(['Log signal has no name or a column number is missing. ',...
            'Please check all "File Logger Signal" blocks']);
        FileLoggerString = strcat(FileLoggerString, 'ERROR!;');
    else
        FileLoggerString = strcat(FileLoggerString,cellArray{j},';');
    end
end

% Now write the data to the RTWdata of the file logger init.
if ~isempty(FileLoggerInitBlock)
    filename = get_param(FileLoggerInitBlock, 'filename');
    interval = get_param(FileLoggerInitBlock, 'interval');
    fileLoggerMaxFileLength = str2num(get_param(FileLoggerInitBlock, 'fileLoggerMaxFileLength'))*1000; % to kB
    dotseparatorStr = get_param(FileLoggerInitBlock, 'dotseparator');
    % Convert string to integer
    if strcmp(dotseparatorStr, 'on')
        dotseparator = 1;
    else
        dotseparator = 0;
    end
    autostartStr = get_param(FileLoggerInitBlock, 'autostart');
    % Convert string to integer
    if strcmp(autostartStr, 'on')
        autostart = 1;
    else
        autostart = 0;
    end
    modelRTWFields = struct('filename', filename, 'firstline', FileLoggerString, ...
        'interval', interval, 'maxsignals', int2str(maxSignals), ...
        'dotseparator', int2str(dotseparator), 'autostart', int2str(autostart),...
        'fileLoggerMaxFileLength', int2str(fileLoggerMaxFileLength));
    % Find the s-function under the mask
    SfunctionBlock = find_system(FileLoggerInitBlock, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
    set_param(SfunctionBlock{1},'RTWdata',modelRTWFields);
elseif elementIdx > 0
    wrndlg('Found File Logger Signal block(s) but no File Logger Init block');
end

end % End of function


% Function to change the data type string, it is too risky to use the
% regular data type strings in the c-code.
function dataTypeStr = getDataTypeString(dataType)
if strcmp(dataType,'double')
    dataTypeStr = 'FILELOGGER_FLOAT_DOUBLE';
elseif strcmp(dataType,'single')
    dataTypeStr = 'FILELOGGER_FLOAT_SINGLE';
elseif strcmp(dataType,'int8')
    dataTypeStr = 'FILELOGGER_8BIT_SIGNED';
elseif strcmp(dataType,'uint8')
    dataTypeStr = 'FILELOGGER_8BIT_UNSIGNED';
elseif strcmp(dataType,'int16')
    dataTypeStr = 'FILELOGGER_16BIT_SIGNED';
elseif strcmp(dataType,'uint16')
    dataTypeStr = 'FILELOGGER_16BIT_UNSIGNED';
elseif strcmp(dataType,'int32')
    dataTypeStr = 'FILELOGGER_32BIT_SIGNED';
elseif strcmp(dataType,'uint32')
    dataTypeStr = 'FILELOGGER_32BIT_UNSIGNED';
elseif strcmp(dataType,'boolean')
    dataTypeStr = 'FILELOGGER_8BIT_UNSIGNED';
end
end