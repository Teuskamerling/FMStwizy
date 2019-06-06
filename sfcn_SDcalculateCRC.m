%% StreetDrone CRC calculation block
function sfcn_can_canopen_sdowrite(block)
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
  block.NumInputPorts = 8;
  block.NumOutputPorts = 1;
 
for InputCounter = 1:8
      block.InputPort(InputCounter).Dimensions = 1;
      block.InputPort(InputCounter).DatatypeID = 3; %% uint8
      block.InputPort(InputCounter).Complexity = 'Real';
      block.InputPort(InputCounter).DirectFeedthrough = true;
      block.InputPort(InputCounter).SamplingMode = 'sample';
end

  %% CRC output
  block.OutputPort(1).Dimensions = 1;
  block.OutputPort(1).DatatypeID = 3; %% uint8
  block.OutputPort(1).Complexity = 'Real';
  block.OutputPort(1).SamplingMode = 'sample';
    
  % Number of S-Function parameters expected
  % (tsamp, canBus, dataType, index, subIndex, waitingTime)
  block.NumDialogPrms     = 0;
  %block.SampleTimes = [block.DialogPrm(1).Data 0];
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
