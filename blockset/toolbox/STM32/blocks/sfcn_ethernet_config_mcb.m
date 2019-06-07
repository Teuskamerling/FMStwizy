%%***************************************************************************************
%% file         sfcn_ethernet_config_mcb.m
%% brief        Block mask initialization function.
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
function [] = sfcn_ethernet_config_mcb(ip1, ip2, ip3, ip4, nm1, nm2, nm3, nm4, gw1, gw2, gw3, gw4)

% validate IP address
if (ip1 < 0) || (ip1 > 255)
  fprintf ('Error: Invalid IP address value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  ip1 = 255;
end;
if (ip2 < 0) || (ip2 > 255)
  fprintf ('Error: Invalid IP address value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  ip2 = 255;
end;
if (ip3 < 0) || (ip3 > 255)
  fprintf ('Error: Invalid IP address value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  ip3 = 255;
end;
if (ip4 < 0) || (ip4 > 255)
  fprintf ('Error: Invalid IP address value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  ip4 = 255;
end;

% validate network mask
if (nm1 < 0) || (nm1 > 255)
  fprintf ('Error: Invalid network mask value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  nm1 = 255;
end;
if (nm2 < 0) || (nm2 > 255)
  fprintf ('Error: Invalid network mask value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  nm2 = 255;
end;
if (nm3 < 0) || (nm3 > 255)
  fprintf ('Error: Invalid network mask value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  nm3 = 255;
end;
if (nm4 < 0) || (nm4 > 255)
  fprintf ('Error: Invalid network mask value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  nm4 = 255;
end;

% validate gateway address
if (gw1 < 0) || (gw1 > 255)
  fprintf ('Error: Invalid gateway address value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  gw1 = 255;
end;
if (gw2 < 0) || (gw2 > 255)
  fprintf ('Error: Invalid gateway address value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  gw2 = 255;
end;
if (gw3 < 0) || (gw3 > 255)
  fprintf ('Error: Invalid gateway address value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  gw3 = 255;
end;
if (gw4 < 0) || (gw4 > 255)
  fprintf ('Error: Invalid gateway address value specified in "Ethernet config". It must be in the 0 - 255 range.\n');
  gw4 = 255;
end;



% create resource keywords to be reserved in resource database
modelRTWFields = struct('ip1', int2str(ip1), 'ip2', int2str(ip2), 'ip3', int2str(ip3), 'ip4', int2str(ip4), 'nm1', int2str(nm1), 'nm2', int2str(nm2), 'nm3', int2str(nm3), 'nm4', int2str(nm4), 'gw1', int2str(gw1), 'gw2', int2str(gw2), 'gw3', int2str(gw3), 'gw4', int2str(gw4));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'M-S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);

%%******************************* end of sfcn_ethernet_config_mcb.m **************************


