% Solve an Input-Output Fitting problem with a Neural Network
% Script generated by NFTOOL
%
% This script assumes these variables are defined:
%
%   houseInputs - input data.
%   houseTargets - target data.
 
% Create training set
Trials = getTrials('Trials/track01_mrracer.mat');

num_state_features = size(Trials{1}.S,2);
num_action_features = size(Trials{1}.A,2);

X = zeros(0,num_state_features + num_action_features);
y = zeros(0,num_state_features);

for i = 1:numel(Trials)
    S = Trials{i}.S;
    A = Trials{i}.A;
        
    X = [X; S(1:end-1, :) A(1:end-1, :)];
    y = [y; S(2:end, :)];
end
 
% Create a Fitting Network
hiddenLayerSize = 10;
net = newff(X',y',hiddenLayerSize);

% Set up Division of Data for Training, Validation, Testing
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;
 
% Train the Network
[net,tr] = train(net,X',y');
 
% Test the Network
outputs = net(X');
errors = gsubtract(outputs,targets);
performance = perform(net,targets,outputs);
 
% View the Network
%view(net)
 
% Plots
% Uncomment these lines to enable various plots.
%figure, plotperform(tr)
%figure, plottrainstate(tr)
%figure, plotfit(targets,outputs)
%figure, plotregression(targets,outputs)
%figure, ploterrhist(errors)