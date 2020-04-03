% Cost_Design: Definition of cost functions for reaching goal state and/or
% passing through via-point.
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

function Cost = Cost_Design( m_quad, Task)
    %COST_DESIGN Creates a cost function for LQR and for ILQC
    %   *.h*   - continuous time terminal cost
    %   *.l*   - continous time intermediate cost

    % Quadcopter system state x
    syms qxQ qyQ qzQ qph qth qps dqxQ dqyQ dqzQ dqph dqth dqps real
    x_sym = [ qxQ qyQ qzQ ...    % position x,y,z
              qph qth qps ...    % roll, pitch, yaw
              dqxQ dqyQ dqzQ ... % velocity x,y,z
              dqph dqth dqps ]'; % angular velocity roll, pitch, yaw     
    % Quadcopter input u (Forces / Torques)
    syms Fz Mx My Mz real; u_sym = [ Fz Mx My Mz ]';
    % Time variable for time-varying cost
    syms t_sym real;
    Cost = struct;

    %% Defining the cost matrices
    % LQR controller
    % Q needs to be symmetric and positive semi definite
    Cost.Q_lqr = diag([  1   1    1   ...   % penalize positions
                         3   3    3   ...   % penalize orientations
                         0.1  0.1    2   ...   % penalize linear velocities
                         1   1    1 ]);     % penalize angular velocities         
    % R needs to be positive definite
    Cost.R_lqr = 10*diag([1 1 1 1]);        % penalize control inputs

    % ILQC controller
    Cost.Qm  = Cost.Q_lqr;                  % it makes sense to redefine these
    Cost.Rm  = Cost.R_lqr;
    Cost.Qmf = 3.0*Cost.Q_lqr;

    %% Define the form of the cost function used in ILQC
    % Reference states and input the controller is supposed to track
    gravity = 9.81;
    f_hover = m_quad*gravity; % keep input close to the once necessary for hovering
    Cost.u_eq = [ f_hover ; zeros(3,1) ];
    Cost.x_eq = Task.goal_x;

    % alias for easier referencing
    x = x_sym; 
    u = u_sym;
    x_goal = Task.goal_x;

    % In this exercise, we only consider the via-point task.
    Cost.problem_type = 'via_point'; % Choose 'goal_state or 'via_point'

    switch Cost.problem_type
        case 'via_point'
            p1 = Task.vp1;      % p1 = Task.vp2 also try this one
            t1 = Task.vp_time;

            % weighting for way points
            Q_vp = zeros(size(Cost.Qm)); 
            Q_vp(1:3,1:3) = 1*Cost.Qmf(1:3,1:3); % only penalize position 

            % don't penalize position deviations, drive system with final cost
            Cost.Qm(1:3,1:3) = zeros(3); 

            % Define symbolic cost function.
            Cost.h = simplify((x-x_goal)'*Cost.Qmf*(x-x_goal));
            Cost.l = simplify(...
                (u-Cost.u_eq)'*Cost.Rm*(u-Cost.u_eq)  ...
                + (x-Cost.x_eq)'*Cost.Qm*(x-Cost.x_eq) ...
                + viapoint(t1,p1,x,t_sym,Q_vp));


          otherwise
            error('Unknown cost function mode');

    end 

    %% Cost functions for PI2 learning
    % Intermediate cost
    l_ =  Cost.l*Task.dt;
    Cost.q_fun   = matlabFunction(l_, 'Vars', [qxQ qyQ qzQ qph qth qps dqxQ dqyQ dqzQ dqph dqth dqps Fz Mx My Mz t_sym]);

    % Terminal cost
    h_ = Cost.h;
    Cost.qf_fun   = matlabFunction(h_, 'Vars', [qxQ qyQ qzQ qph qth qps dqxQ dqyQ dqzQ dqph dqth dqps]);

    Cost.x  = x;
    Cost.u  = u;
    Cost.t  = t_sym; 
end


function viapoint_cost = viapoint(vp_t,vp,x,t,Qm_vp)
    % WAYPOINT Creates cost depending on deviation of the system state from a
    % desired state vp at time t. Doesn't need to be modified. 
    % For more details see:
    % http://www.adrl.ethz.ch/archive/p_14_mlpc_quadrotor_cameraready.pdf
    %
    %    vp_t  :   time at which quadrotor should be at viapoint wp
    %    vp    :   position where quad should be at given time instance
    %    x,t   :   symbolic references to state and time
    %    Qm_vp :   The weighting matrix for deviation from the via-point

    prec = 3; % how 'punctual' does the quad have to be? The bigger the
              % number, the harder the time constraint

    viapoint_cost = (x-vp)'*Qm_vp*(x-vp) ...
                    *exp(-0.5*prec*(t-vp_t)^2) /sqrt(2*pi/prec);
end

