clc; clear all; close all
format short
addpath( genpath( '..\MyRobot' ) );
%% initialize
delete('Robot.mat');
robotType = 'I5';
Robot = ParaCAD(robotType);
%%
% aubo_i = toolboxModel();

