% main_p1_pi2: Main script for Problem 3.1 PI2 algorithm design
%
% --
% Control for Robotics
% AER1517 Spring 2020
% Programming Exercise 3
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
% 
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistant: 
% SiQi Zhou
% siqi.zhou@robotics.utias.utoronto.ca
%
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [20.03.22, SZ]    first version

close all;
clear all;
clc;

%% General
% Add subdirectories
addpath(genpath(pwd));

% Define task
Task = Task_Design();

% Load the nominal model of the quadcopter
load('Quadrotor_Model.mat','Model');

% Define cost function
Task.cost = Cost_Design( Model.param.mQ, Task );

%% Initial controller design
% [Todo] Fill in your LQR_Design and ILQC_Design from Assignment 1 here
[Initial_Controller, Cost_LQR] = LQR_Design(Model, Task);
[ILQC_Controller, Cost] = ILQC_Design(Model,Task,Initial_Controller,@Quad_Simulator);

%% Transform controller from ILQC representation to basis function representation
% Note: The function 'BaseFcnTrans' transforms the ILQC controller into
% basis-function representation, which later serves as initialization to
% the PI2 algorithm. ILQC provides a time-indexed state feedback plus
% feedforward. Representing the initial ILQC controller in terms of a
% parameter matrix theta, that acts on the basis functions, drastically
% reduces the number of controller parameters.
ReducedController = BaseFcnTrans(ILQC_Controller,Task.n_gaussian);

%% Load the perturbed "real" quadrotor model
clear Model;
load('Quadrotor_Model_perturbed.mat','Model_perturbed'); 

%% Visualization of initial guess with policy in base function representation on perturbed system
Task.noise_insertion_method = '';
Test_sim_out = Sample_Rollout(Model_perturbed, Task, ReducedController);
fprintf('Final Quadcopter Position: xQ = %.3f, yQ = %.3f, zQ = %.3f \n',Test_sim_out.x(1:3,end));
fprintf('Final Quadcopter Velocity: xQ = % .3f, yQ = %.3f, zQ = %.3f \n',Test_sim_out.x(7:9,end));
% Visualize2(Test_sim_out,Model_perturbed.param);
 
%% Start PI Learning
% [Todo] Complete PI2_Update implementation, which is called by the
% function PIs_Learning

% Plotting
noise_array = [Task.std_noise*2, Task.std_noise, Task.std_noise/2, Task.std_noise/4];
figure;
txt = cell(length(noise_array),1);

for i = 1:length(noise_array)
    Task.std_noise = noise_array(i);
    t_cpu = cputime;
    [LearnedController,AllCost,AllController] = PIs_Learning(Model_perturbed,...
        Task, ReducedController);
    t_cpu = cputime - t_cpu;
    
    % Plotting
    plot(AllCost); hold on;
    txt{i}= sprintf('noise = %i', noise_array(i));
    
    
    fprintf('CPU time: %f \n',t_cpu);
    fprintf('The PI2 algorithm took %fs to converge \n\n',t_cpu);
end

% Plotting
hold off;
xlabel('PI2 iteration number');
ylabel('Cost');
legend(txt);
title('Quadrotor PI Learning Curve');

%% Visualization of PI2 final trajectory
Task.random=[];
Test_sim_out = Sample_Rollout(Model_perturbed, Task, LearnedController);
fprintf('Final Quadcopter Position: xQ = %.3f, yQ = %.3f, zQ = %.3f \n',Test_sim_out.x(1:3,end));
fprintf('Final Quadcopter Velocity: xQ = %.3f, yQ = %.3f, zQ = %.3f \n',Test_sim_out.x(7:9,end));
% Visualize2(Test_sim_out,Model_perturbed.param);

%% Cost plot
% figure;
% txt = cell(length(noise_array),1);
% for i = 1:length(noise_array)
%     plot(AllCost(i)); hold on;
%     txt{i}= sprintf('noise = %i', noise_array(i));
% end
% hold off;
% xlabel('PI2 iteration number');
% ylabel('Cost');
% title('Quadrotor PI Learning Curve');