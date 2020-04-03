% Task_Design: Definition of a task for a controller to execute.
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
% [20.03.22]    first version

function Task = Task_Design( )
%TASK_DESIGN Defines a task for a controller to execute
    %% ILQC part Task parameters
    % IMPORTANT: do not change these parameters

    Task = struct;

    Task.dt             = 0.02;        % sampling time period

    Task.start_time     = 0;

    Task.goal_time      = 7.5;          % Time to reach goal

    Task.start_x        = [ 0; 0; 0;   % position x,y,z
                            0; 0; 0;   % roll, pitch, yaw
                            0; 0; 0;   % velocity x,y,z
                            0; 0; 0 ]; % angular rates roll, pitch, yaw

    Task.vp1            = [ 5; -5; -5;  %the viapoint
                            0; 0; 0;         
                            0; 0; 0;
                            0; 0; 0;];         

    Task.goal_x         = [ 10; 0; 0;   % position x,y,z
                            0; 0; 0;   % roll, pitch, yaw
                            0; 0; 0;   % velocity x,y,z
                            0; 0; 0 ]; % angular rates roll, pitch, yaw


    Task.vp_time = Task.goal_time/2;   % time to pass through via-point

    Task.max_iteration  = 5;           % Maximum ILQC iterations  

    Task.input_noise_mag = 0.186;     % Adds noise on input to simulation

    Task.cost = [];                    % cost encodes performance criteria of 
                                       % how to execute the task. Filled later.    
    Task.noise_insertion_method = '';                                  

    %%  PI2 part Task parameters
    % IMPORTANT: do not change these parameters unless for answering the
    % additional questions

    Task.n_gaussian    = 15;            % Number of Guassian base functions for smoothed-time controller

    Task.num_rollouts  = 20 ;

    Task.num_reuse     = 8; 

    Task.max_iteration_learning  = 30;  % Maximum PI² iterations 

    Task.num_repeat    = 1; 

    Task.std_noise     = 0.0015;        % standard deviation of exploration noise for PI2

    Task.feedforward   = 1;          

    Task.interpolation = 'linear';      % how to interpolate the samples?

    Task.R_scale = 1e-11;               % scale the regularization term (smaller number --> weaker regularization)

    % Constant noise over the trajectory for parameter-perturbation
    num_time_steps      = (Task.goal_time-Task.start_time)/Task.dt + 1;

    Task.random         = repmat(randn(13*Task.n_gaussian,4,1,100),[1 1 num_time_steps 1]);

                            
end

