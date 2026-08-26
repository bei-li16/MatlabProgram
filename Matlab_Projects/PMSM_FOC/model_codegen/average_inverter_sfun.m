function average_inverter_sfun(block)
%AVERAGE_INVERTER_SFUN Average two-level inverter driven by SVPWM duties.
setup(block);
end

function setup(block)
block.NumDialogPrms = 0;
block.NumInputPorts = 4;
block.NumOutputPorts = 2;
for index = 1:block.NumInputPorts
    block.InputPort(index).Dimensions = 1;
    block.InputPort(index).DatatypeID = 1; % single
    block.InputPort(index).Complexity = 'Real';
    block.InputPort(index).DirectFeedthrough = true;
end
for index = 1:block.NumOutputPorts
    block.OutputPort(index).Dimensions = 1;
    block.OutputPort(index).DatatypeID = 1; % single
    block.OutputPort(index).Complexity = 'Real';
end
block.SampleTimes = [1.0e-4 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('Outputs', @outputs);
end

function outputs(block)
dutyA = single(block.InputPort(1).Data);
dutyB = single(block.InputPort(2).Data);
dutyC = single(block.InputPort(3).Data);
vdc = single(block.InputPort(4).Data);

va = (dutyA - single(0.5)) * vdc;
vb = (dutyB - single(0.5)) * vdc;
vc = (dutyC - single(0.5)) * vdc;
commonMode = (va + vb + vc) / single(3.0);
va = va - commonMode;
vb = vb - commonMode;

block.OutputPort(1).Data = va;
block.OutputPort(2).Data = (va + single(2.0) * vb) * ...
    single(0.5773502691896258);
end
