%%***************************************************************************************
%% file         ert_udp_check_hook.m
%% brief        Function to check the consistency of the UDP blocks that can be called 
%%              from the ert_make_rtw_hook.m script that runs during the RTW code 
%%              generation process.
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
function ert_uds_check_hook(modelName)        
  % check all UDP blocks that depend on the Ethernet config block
  checkEthernetConfigBlockPresence(modelName, 'UDP Client Send');
  checkEthernetConfigBlockPresence(modelName, 'UDP Server Init');
  % check all UDP blocks that depend on the UDP Server Init block
  checkUdpServerInitBlockPresence(modelName, 'UDP Server Receive');
  checkUdpServerInitBlockPresence(modelName, 'UDP Server New Data Available');
  checkUdpServerInitBlockPresence(modelName, 'UDP Server Receive Size');
  % perform individual block checks
  checkUdpSeverInitBlock(modelName);
  checkUdpClientSendBlock(modelName);
end


% Function to check if a UDP block with a specific mask is present, and if so to verify
% that the Ethernet config block is also in the model.
function checkEthernetConfigBlockPresence(modelName, maskName)
  % this block requires a Ethernet config block
  num_blocks = length(find_system(modelName, 'MaskType', maskName));
  if num_blocks > 0
    % this block is used at least once
    num_config_blocks = length(find_system(modelName, 'MaskType', 'Ethernet config'));
    if num_config_blocks == 0
      msg = sprintf(['Error: No "Ethernet config" block found while using other UDP blocks.\n', ...
                     'Insert a "Ethernet config" block to ommit this error message.\n']);
      % Display error message in the matlab command window.
      fprintf(msg);
      % Abort and display pop-up window with error message.
      error(msg);           
    end
  end
end


% Function to check if a UDP block with a specific mask is present, and if so to verify
% that the UDP Server Init block is also in the model.
function checkUdpServerInitBlockPresence(modelName, maskName)
  % this block requires a Ethernet config block
  num_blocks = length(find_system(modelName, 'MaskType', maskName));
  if num_blocks > 0
    % this block is used at least once
    num_config_blocks = length(find_system(modelName, 'MaskType', 'UDP Server Init'));
    if num_config_blocks == 0
      msg = sprintf(['Error: No "UDP Server Init" block found while using other UDP Server blocks.\n', ...
                     'Insert a "UDP Server Init" block to ommit this error message.\n']);
      % Display error message in the matlab command window.
      fprintf(msg);
      % Abort and display pop-up window with error message.
      error(msg);           
    end
  end
end

% Function to do block specific checks on the UDP Server Init block. These checks make
% sure that the value specified for the port are within a valid range and
% that there is only one instance of this block present in the model.
function checkUdpSeverInitBlock(modelName)
  % first determine the number of UDP Server Init blocks
  num_blocks = length(find_system(modelName, 'MaskType', 'UDP Server Init'));
  % only continue with check if at least one is found
  if num_blocks > 0
    % first make sure that is only one instance of this block present in the model
    if num_blocks > 1
      msg = sprintf(['Error: Too many "UDP Server Init" blocks found. Only one is allowed.\n']);
      % Display error message in the matlab command window.
      fprintf(msg);
      % Abort and display pop-up window with error message.
      error(msg);           
    else
      % at this point we know for sure that there is only 1 UDP Server Init block  
      % store blocks in array
      blocks = find_system(modelName, 'MaskType', 'UDP Server Init');
      % obtain the parameter values
      maskValuesCurrent = get_param(blocks(1), 'MaskValues');
      % check the parameter 'Port'
      if (str2num(maskValuesCurrent{1}{1}) < 1) || (str2num(maskValuesCurrent{1}{1}) > 65535)
        msg = sprintf(['Error: Parameter "Port" is not within the 1..65535 range in block\n"%s".\n'], char(blocks(1)));
        % Display error message in the matlab command window.
        fprintf(msg);
        % Abort and display pop-up window with error message.
        error(msg);           
      end            
    end
  end      
end

% Function to do block specific checks on the UDP Client Send block. These checks make
% sure that the values specified for the IP address and the port are within a valid range
function checkUdpClientSendBlock(modelName)
  % loop through all UDP Client Send blocks in the model
  num_blocks = length(find_system(modelName, 'MaskType', 'UDP Client Send'));
  if num_blocks > 0
    % store blocks in array
    blocks = find_system(modelName, 'MaskType', 'UDP Client Send');
      % loop through all blocks in array
      for i = 1:num_blocks
        % obtain the parameter values
        maskValuesCurrent = get_param(blocks(i), 'MaskValues');
        % check the parameter 'IP address 1'
        if (str2num(maskValuesCurrent{1}{1}) < 0) || (str2num(maskValuesCurrent{1}{1}) > 255)
          msg = sprintf(['Error: Parameter "IP address 1" is not within the 0..255 range in block\n"%s".\n'], char(blocks(i)));
          % Display error message in the matlab command window.
          fprintf(msg);
          % Abort and display pop-up window with error message.
          error(msg);           
        end            
        % check the parameter 'IP address 2'
        if (str2num(maskValuesCurrent{1}{2}) < 0) || (str2num(maskValuesCurrent{1}{2}) > 255)
          msg = sprintf(['Error: Parameter "IP address 2" is not within the 0..255 range in block\n"%s".\n'], char(blocks(i)));
          % Display error message in the matlab command window.
          fprintf(msg);
          % Abort and display pop-up window with error message.
          error(msg);           
        end            
        % check the parameter 'IP address 3'
        if (str2num(maskValuesCurrent{1}{3}) < 0) || (str2num(maskValuesCurrent{1}{3}) > 255)
          msg = sprintf(['Error: Parameter "IP address 3" is not within the 0..255 range in block\n"%s".\n'], char(blocks(i)));
          % Display error message in the matlab command window.
          fprintf(msg);
          % Abort and display pop-up window with error message.
          error(msg);           
        end            
        % check the parameter 'IP address 4'
        if (str2num(maskValuesCurrent{1}{4}) < 0) || (str2num(maskValuesCurrent{1}{4}) > 255)
          msg = sprintf(['Error: Parameter "IP address 4" is not within the 0..255 range in block\n"%s".\n'], char(blocks(i)));
          % Display error message in the matlab command window.
          fprintf(msg);
          % Abort and display pop-up window with error message.
          error(msg);           
        end            
        % check the parameter 'Port'
        if (str2num(maskValuesCurrent{1}{5}) < 1) || (str2num(maskValuesCurrent{1}{5}) > 65535)
          msg = sprintf(['Error: Parameter "Port" is not within the 1..65535 range in block\n"%s".\n'], char(blocks(i)));
          % Display error message in the matlab command window.
          fprintf(msg);
          % Abort and display pop-up window with error message.
          error(msg);           
        end            
      end
  end
end


%%******************************* end of ert_udp_check_hook.m ***************************
