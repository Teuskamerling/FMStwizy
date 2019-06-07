%%***************************************************************************************
%% file         sfcn_timein_get.m
%% brief        Level-2 M file S-Function for the reading data from a Timer Input 
%%              channel.
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
function sfcn_timein_get(block)
  setup(block);
%endfunction


%% Function: setup ===================================================
%% Abstract:
%%   Set up the S-function block's basic characteristics such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%% 
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
function setup(block)
  %% Register number of input and output ports
  block.NumInputPorts = 0;
  block.NumOutputPorts = 7;
  %% Override ports
  %% Frequency
  block.OutputPort(1).Dimensions = 1;
  block.OutputPort(1).DatatypeID = 7; %% uint32 is type 7, see rtwtypes.h
  block.OutputPort(1).Complexity = 'Real';
  block.OutputPort(1).SamplingMode = 'sample';
  %% Duty cycle
  block.OutputPort(2).Dimensions = 1;
  block.OutputPort(2).DatatypeID = 5; %% uint16 is type 5, see rtwtypes.h
  block.OutputPort(2).Complexity = 'Real';
  block.OutputPort(2).SamplingMode = 'sample';
  %% Edge count
  block.OutputPort(3).Dimensions = 1;
  block.OutputPort(3).DatatypeID = 7; %% uint32 is type 7, see rtwtypes.h
  block.OutputPort(3).Complexity = 'Real';
  block.OutputPort(3).SamplingMode = 'sample';
  %% Edge state
  block.OutputPort(4).Dimensions = 1;
  block.OutputPort(4).DatatypeID = 3; %% uint8 is type 3, see rtwtypes.h
  block.OutputPort(4).Complexity = 'Real';
  block.OutputPort(4).SamplingMode = 'sample';
  %% Last edge timestamp
  block.OutputPort(5).Dimensions = 1;
  block.OutputPort(5).DatatypeID = 7; %% uint32 is type 7, see rtwtypes.h
  block.OutputPort(5).Complexity = 'Real';
  block.OutputPort(5).SamplingMode = 'sample';
  %% Internal counter timestamp
  block.OutputPort(6).Dimensions = 1;
  block.OutputPort(6).DatatypeID = 7; %% uint32 is type 7, see rtwtypes.h
  block.OutputPort(6).Complexity = 'Real';
  block.OutputPort(6).SamplingMode = 'sample';
  %% Zero Hz detected
  block.OutputPort(7).Dimensions = 1;
  block.OutputPort(7).DatatypeID = 8; %% boolean is type 8, see rtwtypes.h
  block.OutputPort(7).Complexity = 'Real';
  block.OutputPort(7).SamplingMode = 'sample';
  % Number of S-Function parameters expected
  block.NumDialogPrms     = 1;
  block.SampleTimes = [block.DialogPrm(1).Data 0];
  %% -----------------------------------------------------------------
  %% Register methods called at run-time
  %% -----------------------------------------------------------------
  
  %% 
  %% Start:
  %%   Functionality    : Called in order to initialize state and work
  %%                      area values
  %%   C-Mex counterpart: mdlStart
  %%
  block.RegBlockMethod('Start', @Start);

  %% 
  %% Outputs:
  %%   Functionality    : Called to generate block outputs in
  %%                      simulation step
  %%   C-Mex counterpart: mdlOutputs
  %%
  block.RegBlockMethod('Outputs', @Outputs);

  %% 
  %% Update:
  %%   Functionality    : Called to update discrete states
  %%                      during simulation step
  %%   C-Mex counterpart: mdlUpdate
  %%
  block.RegBlockMethod('Update', @Update);
%endfunction

function Start(block)

  %% No start

%endfunction


function Outputs(block)

  %% No output
  
%endfunction


function Update(block)

  %% No update
  
%endfunction


%%******************************* end of sfcn_timein_get.m ******************************
