%%***************************************************************************************
%% file         HANcoderCHecks.m
%% brief        Check if original HANcoder software is used.
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
%% This function checks whether the sizes of the blocks and their position
%%  is correct. If not the user will be warned and the old settings will
%%  be reset.
function [checkPassed] = HANcoderChecks(command,modelName)
        
switch command
    %% Prevent moving or resizing the top level subsystem
    case 'MoveE407'
        name = 'HANcoder STM32 Target - E407-STM32 algorithm';
        desiredPosition = [84 26 684 532];
        checkPassed = ResetBlockPosition(name, desiredPosition);
        return;
            %% Prevent moving or resizing the top level subsystem
    case 'MoveOlimexino'
        name = 'HANcoder STM32 Target - Olimexino-STM32 algorithm';
        desiredPosition = [84 26 684 532];
        checkPassed = ResetBlockPosition(name, desiredPosition);
        return;
            %% Prevent moving or resizing the top level subsystem
    case 'MoveP405'
        name = 'HANcoder STM32 Target - P405-STM32 algorithm';
        desiredPosition = [84 26 684 532];
        checkPassed = ResetBlockPosition(name, desiredPosition);
        return;

    %% Prevent moving or resizing the license block
    case 'MoveLicenseBlock'
        name = 'License';
        desiredPosition = [296 624 473 684];
        checkPassed = ResetBlockPosition(name, desiredPosition);
        return;
    
    %% Warn when the user tries to change the name of the top level
    %% subsystem
    case 'NameChangeE407'
        name = 'HANcoder STM32 Target - E407-STM32 algorithm';
        checkPassed = CheckNameBlock(name);
    case 'NameChangeOlimexino'
        name = 'HANcoder STM32 Target - Olimexino-STM32 algorithm';
        checkPassed = CheckNameBlock(name);
    case 'NameChangeP405'
        name = 'HANcoder STM32 Target - P405-STM32 algorithm';
        checkPassed = CheckNameBlock(name);
    %% Warn when the user tries to change the name of the license block
    case 'NameChangeLicenseBlock'
    name = 'License';
    checkPassed = CheckNameBlock(name);

    %% Warn when the user tries to delete the block
    case 'Delete'
    % Show a message that deleting a block causes HANcoder to stop working
    showWarningDialog(horzcat('Please do not change anything in this', ...
    ' part of the model. By deleting this block HANcoder will', ...
    'no longer work. Please undo the changes.'));
    checkPassed = 0;
    return;

    %% Warn when the user tries to change something in the top level layout
    case 'Copy'
    % Show a message that the top level layout should not be changed
    showWarningDialog(horzcat('Please do not change anything in this', ...
    ' part of the model. Any change will make HANcoder ', ...
    'no longer work.'));
    checkPassed = 0;
    return;
    
    %% These checks are done by the hookfile
    case 'Hookfile'
       checkPassed = EnforceTopLevelLayout(modelName);
       return;

    %% The hookfile can request a secret key to see if the original file
    %% is used instead of a fake one.
    case 'HookfileSecretKey'
        checkPassed = 071503416317;
        return;        
        
end %end of switch statement

end %end of function CheckTopLevelLayout


%%  This function checks if the top level layout is correct. Users are not
%   allowed to change the layout
function ErrorMessage = EnforceTopLevelLayout(modelName)

% Change ErrorMessage to something like ErrorMessage

% Store all blocks in variable
blocksOnTopLevel = find_system(modelName, 'SearchDepth','1','FindAll', 'on');
% Check if not too many blocks are present (1 Annotation, 2 Subsystems, 1
% Blockdiagram)
if length(blocksOnTopLevel) > 12 % Changed to twelve for model reference in- and output (ports and lines also count)
    ErrorMessage = horzcat('Amount of blocks on the top level is wrong. ',...
    'Please restore the original top level layout');
    return
end

% Find annotation block
AnnotationBlock = find_system(modelName, 'SearchDepth','1','FindAll', 'on', 'Type', 'Annotation');
% Check if one is found
if ~isempty(AnnotationBlock)
    % Store annotation text
    AnnotationText = get_param(AnnotationBlock(1), 'Text');
    % Define the text that it should be
    AnnotationTextShouldBe = 'Read more on HANcoder';
        % Check the text of the annotation
        if ~strcmp(AnnotationText, AnnotationTextShouldBe);
            ErrorMessage = horzcat('Annotation text is wrong', ...
            'It should be: ''Read more on HANcoder''');
            return;
        end
    % Get the clickFcn text of the annotation block
    AnnotationClickFcn = get_param(AnnotationBlock(1), 'ClickFcn');
    % Define the clickFcn text that is should be
    AnnotationClickFcnShouldBe = 'web(''www.han.nl/hancoder'');';
    % Compare the two annotation texts
    if (~strcmp(AnnotationClickFcn, AnnotationClickFcnShouldBe))
        % If the texts to not match, give an error
        ErrorMessage = horzcat('The annotation clickFcn text has been', ...
            ' changed. It should contain a link to:''www.han.nl/hancoder''');
        ErrorMessage = 0;
        return;
    end
else
    % Display an error message indicating that the annotation block is
    % missing
    ErrorMessage = horzcat('No annotation block found,', ...
        'Please restore the original top level layout');
    return;
end

% Check if the license.pdf is present
if ~exist('License.pdf', 'file')
    % Display a message indicating that the license file is missing
    ErrorMessage = horzcat('License file not found. Without a',...
        ' valid license file you are not able to build.');
    return;
end

% Search for the licence block by maskType (documentation in Mask)
licenceBlock = find_system(modelName,'SearchDepth','1', 'MaskType', 'License');
% Check if one is found
if isempty(licenceBlock)
    ErrorMessage = horzcat('No license block found.', ...
        'Please restore the original top level layout');
    % No licence block found
    return;
else
    try
    % Store the openFcn text
    OpenFcnText = get_param(licenceBlock, 'OpenFcn');
    % Check if the command text is correct
    if ~strcmp(OpenFcnText, 'winopen(''License.pdf'')')
        ErrorMessage = 'Redirection to wrong license file';
        return;
    end
    % Store the mask text
    licenceMasktText = get_param(licenceBlock, 'MaskDisplay');
    % Check if the licence mask text is correct
    if ~strcmp(licenceMasktText, 'disp([''End-User License Agreement\n'',''please read before use'']);')
        ErrorMessage = horzcat('Wrong text in license block mask');
        return;
    end
    try
    % Check if it refers to the right text file
    licenceText = textread('License.pdf', '%c');
    catch
        ErrorMessage = 'Error while reading License.pdf';
    end
    % Check if the sum of the text is correct
    if sum(licenceText) ~= 1876175883
        ErrorMessage = 'Wrong text in License.pdf file';
    return;
    end
    % Catch any errors, this means the top level layout is false
    catch
    ErrorMessage = 'Error with license block';
    return;
    end
end

        
% Check if the subsystem block .../Olimexino STM32 exists by checking the
% MaskType (Documentation tab in edit mask)
topLevelBlock = find_system(modelName,'SearchDepth','1', 'MaskType', 'TopLevelSubsystem');
if isempty(topLevelBlock);
    ErrorMessage = horzcat('No TopLevelSubsystem found.', ...
        ' Please restore the original top level layout');
    return
end

%% Check which target we are dealing with
% first try to locate the block
    versionInfoBlock = find_system(modelName, 'MaskType', 'Base Sample Time config');
    % make sure we found the block
    if ~isempty(versionInfoBlock)
        blocksetInfo = get_param(versionInfoBlock, 'Description');    
    else
        % No base sample time block found
        ErrorMessage = ('Could not find base sample time config block');
    end
% convert blocksetInfo to string
blocksetInfo = char(blocksetInfo);
% check if string contains 'E407', 'Olimexino' or 'P405'
if ~isempty(findstr(blocksetInfo,'E407'))
    target = 'E407';
elseif ~isempty(findstr(blocksetInfo,'Olimexino'))
    target = 'Olimexino';
elseif ~isempty(findstr(blocksetInfo,'P405'))
    target = 'P405';
else
    disp('error 404: no target found');
    target = 'none'
    ErrorMessage = horzcat('error 404:no target found');
    return
end


%% Checking all settings in the top level by target type
% Switch statement
switch target
    %% Do checks for the E407
    case 'E407'
 % Check if the block is placed correctly
        if ~strcmp(topLevelBlock, strcat(modelName,'/HANcoder STM32 Target - E407-STM32 algorithm'))
            ErrorMessage = horzcat('No Top Level Subsystem found.', ...
                'Please restore the original top level layout');
            return
        end
        
        % Get mask text from the mask of the topLevelBlock subsystem
        maskText = get_param(topLevelBlock, 'MaskDisplay');
        maskTextShouldBe = 'image(imread(''STM32_E407_HAN.jpg''))';
        % check if the maskText is correct
        if ~strcmp(maskText, maskTextShouldBe);
            ErrorMessage = 'The mask of the Top level subsystem is incorrect';
            return
        end
        
        % Get CopyFcn, NameChangeFcn and MoveFcn callbacks
        topLevelBlockCopyFcn = get_param(topLevelBlock, 'CopyFcn');
        topLevelBlockMoveFcn = get_param(topLevelBlock, 'MoveFcn');
        topLevelBlockNameChangeFcn = get_param(topLevelBlock, 'NameChangeFcn');
        % Check if the correct callbacks are being used
        if(~strcmp('HANcoderChecks(''Copy'');',topLevelBlockCopyFcn))
            ErrorMessage = 'Wrong CopyFcn callback on top level subsystem';
            return;
        elseif (~strcmp('HANcoderChecks(''MoveE407'');',topLevelBlockMoveFcn))
                ErrorMessage = 'Wrong MoveFcn callback on top level subsystem';
                return;
        elseif (~strcmp('HANcoderChecks(''NameChangeE407'');',topLevelBlockNameChangeFcn))
                ErrorMessage = 'Wrong NameChangeFcn callback on top level subsystem';
                return;
        end

        % Check if the Olimexino_STM32_HAN.jpg exists
        if exist('STM32_E407_HAN.jpg', 'file')~= 2
            ErrorMessage = 'No top level layout image found.';
            return
        end

        % Add several "pixels" together to come to a "unique" number
        try
        [pictureInNumbers] = imread('STM32_E407_HAN.jpg');
        pictureInNumbers = double(pictureInNumbers);
        addedCode= pictureInNumbers(250,170,1) + pictureInNumbers(3,10,2)+...
            pictureInNumbers(432,232,3) + pictureInNumbers(265,128,1) + ...
            pictureInNumbers(22,531,3) + pictureInNumbers(44,364,1) + ...
            pictureInNumbers(122,426,3) + pictureInNumbers(144,234,1) + ...
            pictureInNumbers(451,197,3) + pictureInNumbers(325,233,1) + ...
            pictureInNumbers(156,267,1) + pictureInNumbers(333,435,1);
        % If it fails to add the pixels, the picture isn't original
        catch
            ErrorMessage = 'Error verifying the top level layout image';
            return
        end

        % Compare calculated number with expected result to check if the picture is
        % the same
        if addedCode == 2523 
        	ErrorMessage = 0;
        else
            ErrorMessage = horzcat('It is not allowed to change the top level', ...
                ' layout. The image does not match the original!');
        end

        return
    %% Do checks for the Olimexino
    case 'Olimexino'
        % Check if the block is placed correctly
        if ~strcmp(topLevelBlock, strcat(modelName,'/HANcoder STM32 Target - Olimexino-STM32 algorithm'))
            ErrorMessage = horzcat('No Top Level Subsystem found.', ...
                'Please restore the original top level layout');
            return
        end
        
        % Get mask text from the mask of the topLevelBlock subsystem
        maskText = get_param(topLevelBlock, 'MaskDisplay');
        maskTextShouldBe = 'image(imread(''Olimexino_STM32_HAN.jpg''))';
        % check if the maskText is correct
        if ~strcmp(maskText, maskTextShouldBe);
            ErrorMessage = 'The mask of the Top level subsystem is incorrect';
            return
        end
        
        % Get CopyFcn, NameChangeFcn and MoveFcn callbacks
        topLevelBlockCopyFcn = get_param(topLevelBlock, 'CopyFcn');
        topLevelBlockMoveFcn = get_param(topLevelBlock, 'MoveFcn');
        topLevelBlockNameChangeFcn = get_param(topLevelBlock, 'NameChangeFcn');
        % Check if the correct callbacks are being used
        if(~strcmp('HANcoderChecks(''Copy'');',topLevelBlockCopyFcn))
            ErrorMessage = 'Wrong CopyFcn callback on top level subsystem';
            return;
        elseif (~strcmp('HANcoderChecks(''MoveOlimexino'');',topLevelBlockMoveFcn))
                ErrorMessage = 'Wrong MoveFcn callback on top level subsystem';
                return;
        elseif (~strcmp('HANcoderChecks(''NameChangeOlimexino'');',topLevelBlockNameChangeFcn))
                ErrorMessage = 'Wrong NameChangeFcn callback on top level subsystem';
                return;
        end

        % Check if the Olimexino_STM32_HAN.jpg exists
        if exist('Olimexino_STM32_HAN.jpg', 'file')~= 2
            ErrorMessage = 'No top level layout image found.';
            return
        end

        % Add several "pixels" together to come to a "unique" number
        try
        [pictureInNumbers] = imread('Olimexino_STM32_HAN.jpg');
        pictureInNumbers = double(pictureInNumbers);
        addedCode= pictureInNumbers(250,170,1) + pictureInNumbers(3,10,2)+...
            pictureInNumbers(432,232,3) + pictureInNumbers(265,128,1) + ...
            pictureInNumbers(22,531,3) + pictureInNumbers(44,364,1) + ...
            pictureInNumbers(122,426,3) + pictureInNumbers(144,234,1) + ...
            pictureInNumbers(451,197,3) + pictureInNumbers(325,233,1) + ...
            pictureInNumbers(156,267,1) + pictureInNumbers(333,435,1);
        % If it fails to add the pixels, the picture isn't original
        catch
            ErrorMessage = 'Error verifying the top level layout image';
            return
        end

        % Compare calculated number with expected result to check if the picture is
        % the same
        if addedCode == 2412 
        	ErrorMessage = 0;
        else
            ErrorMessage = horzcat('It is not allowed to change the top level', ...
                ' layout. The image does not match the original!');
        end

        return
    %% Do checks for the P405
    case 'P405'
 % Check if the block is placed correctly
        if ~strcmp(topLevelBlock, strcat(modelName,'/HANcoder STM32 Target - P405-STM32 algorithm'))
            ErrorMessage = horzcat('No Top Level Subsystem found.', ...
                'Please restore the original top level layout');
            return
        end
        
        % Get mask text from the mask of the topLevelBlock subsystem
        maskText = get_param(topLevelBlock, 'MaskDisplay');
        maskTextShouldBe = 'image(imread(''STM32_P405_HAN.jpg''))';
        % check if the maskText is correct
        if ~strcmp(maskText, maskTextShouldBe);
            ErrorMessage = 'The mask of the Top level subsystem is incorrect';
            return
        end
        
        % Get CopyFcn, NameChangeFcn and MoveFcn callbacks
        topLevelBlockCopyFcn = get_param(topLevelBlock, 'CopyFcn');
        topLevelBlockMoveFcn = get_param(topLevelBlock, 'MoveFcn');
        topLevelBlockNameChangeFcn = get_param(topLevelBlock, 'NameChangeFcn');
        % Check if the correct callbacks are being used
        if(~strcmp('HANcoderChecks(''Copy'');',topLevelBlockCopyFcn))
            ErrorMessage = 'Wrong CopyFcn callback on top level subsystem';
            return;
        elseif (~strcmp('HANcoderChecks(''MoveP405'');',topLevelBlockMoveFcn))
                ErrorMessage = 'Wrong MoveFcn callback on top level subsystem';
                return;
        elseif (~strcmp('HANcoderChecks(''NameChangeP405'');',topLevelBlockNameChangeFcn))
                ErrorMessage = 'Wrong NameChangeFcn callback on top level subsystem';
                return;
        end

        % Check if the Olimexino_STM32_HAN.jpg exists
        if exist('STM32_P405_HAN.jpg', 'file')~= 2
            ErrorMessage = 'No top level layout image found.';
            return
        end

        % Add several "pixels" together to come to a "unique" number
        try
        [pictureInNumbers] = imread('STM32_P405_HAN.jpg');
        pictureInNumbers = double(pictureInNumbers);
        addedCode= pictureInNumbers(250,170,1) + pictureInNumbers(3,10,2)+...
            pictureInNumbers(432,232,3) + pictureInNumbers(265,128,1) + ...
            pictureInNumbers(22,531,3) + pictureInNumbers(44,364,1) + ...
            pictureInNumbers(122,426,3) + pictureInNumbers(144,234,1) + ...
            pictureInNumbers(451,197,3) + pictureInNumbers(325,233,1) + ...
            pictureInNumbers(156,267,1) + pictureInNumbers(333,435,1);
        % If it fails to add the pixels, the picture isn't original
        catch
            ErrorMessage = 'Error verifying the top level layout image';
            return
        end
        
        % Compare calculated number with expected result to check if the picture is
        % the same
        if addedCode == 2245
        	ErrorMessage = 0;
        else
            ErrorMessage = horzcat('It is not allowed to change the top level', ...
                ' layout. The image does not match the original!');
        end

        return
    %% No checks if the controller isn't recognized
    case 'none'
        ErrorMessage = horzcat('error in switch statement, controller not supported');
        return
    end
end %endfunction EnforceTopLevelLayout




%% This function resets the top level subsystems original position and size
function [positionReset] = ResetBlockPosition(name, desiredPosition)
% declare boolean to indicate if the top level subsystem has been found
topLevelSubsystemIsFound = false;
% Get all blocks from the first "layer" of the model
topLevelBlocks = find_system(gcs, 'SearchDepth','1','FindAll', 'on');

for i=1: length(topLevelBlocks)
    % check if the top level subsystem with name 'HANcoder STM32
    % Target...'has been found
    if(strcmp(get_param(topLevelBlocks(i),'Name'), name))
        % Check the size of the block by comparing the matrices with the
        % coordinates
        if(~isequal(get_param(topLevelBlocks(i),'Position'),desiredPosition))
            % display message to indicate resizing is not allowed
            showWarningDialog(horzcat('Please do not change anything in ',...
                ' this part of the model. \nIt is not possible to move or resize ',...
                ' this block, it will be reset to the original position', ...
                ' and size.'));
            % disable the moveFCN command so an infinite loop is avoided
            %set_param(topLevelBlocks(i),'MoveFcn',' ');
            % try
            % Reset the position and size of the top level subsystem 
            set_param(topLevelBlocks(i),'Position', desiredPosition);
            % catch Error
            % Display error message
            msgString = getReport(Error);
            disp(msgString);
            % end
            % re-enable the moveFCN command
            %set_param(topLevelBlocks(i),'MoveFcn','HANcoderChecks(''Move'');');
        end
    % Mark that the subsystem has been found
    topLevelSubsystemIsFound = true;
    %exit the for loop
    break
    end
end

% Check if the top level subsystem was found
if ~topLevelSubsystemIsFound
    % display a message to indicate the top level subsystem has not been
    % found
    showErrorDialog(horzcat('Please do not change anything in this', ...
    ' part of the model. \nThe original block;''', name, ''' was removed or', ...
    'its name was changed, HANcoder will no longer work.'));
    positionReset = 0;
    return;
end
positionReset = 1;
return;
end %end of function ResetTopLevelSubsystemPosition

function [nameCheck] = CheckNameBlock(name)
% Check if the name is correct
if(~strcmp(name,get_param(gcb,'Name')))
    showWarningDialog(horzcat('Please do not change anything in this', ...
    ' part of the model. By changing the name of this block HANcoder will', ...
    ' no longer work. The name will automatically be changed back.'));
    try
    % try to change the name of the block back to the original
    set_param(gcb,'Name', name);
    catch ME
    end % end of catch statement

end % end of the if statement
nameCheck = 0;
end % end of function CheckNameblock

function showWarningDialog(message)
uiwait(warndlg(sprintf(message),'HANcoder - warning','modal'));
end % end of function showWarningDialog()

function showErrorDialog(message)
uiwait(warndlg(sprintf(message),'HANcoder - error','modal'));
end % end of function showErrorDialog()


