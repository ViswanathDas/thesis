%% APF- MPC Implementation

% In this .m file, we are to take the input from the optimal lane
% (represented in this code as a reference lane) and use that to select the
% correct road potential to use for the APF- based Model Predictive
% Control. The bicycle model is used as the system dynamics with
% longitudinal and lateral velocity and position, yaw angle and yaw rate as
% the system states and the longitudinal force and steering angle as the
% input. 

% Input to the system is the the initial HV and OV states, road data along
% with data about the road and obstacle potentials, 

% [1]E. Snapper, “Model-based Path Planning and Control for Autonomous
% Vehicles using Artificial Potential Fields,” master’s thesis, 
% January 2018.

% [2] MIA ISAKSSON PALMQVIST, "Model Predictive Control for Autonomous 
% Driving of a Truck, master’s thesis, 2016.

% [3] W. Jansen, "Lateral Path-Following Control for Automated Vehicle 
% Platoons", ,master's thesis, 2016 
 
clear all;
clc;

%% Host Data

% states are of the form [v_x X Y v_y r psi]. r is the yaw rate and psi is
% the yaw angle. v_x and v_y are longitudinal and lateral velocities and X
% Y are  coordinates.

x_h_0= [41.6 0 4.5 0 0 0]'; % state of the host vehicle. 

% U=[F_x delta]'
u_h_0= [0 0]';

%% Road Data

% decided that the global origin moved along the boundary of the right lane
% along with the HV (host vehicle).

n_lanes= 2; % number of lanes
size_lane = 3; % width of lane in meters
road_len= 1000;  % length of the road in meters

% Lateral Position of the lane boundaries (does not include the 
% outer boundaries of the road)
for i= 1:n_lanes-1
    loc_lane_bound(i) = i*size_lane; 
end

% Lateral postion of the road boundaries
loc_road_bound = [0 n_lanes*size_lane]; 

all_bound= [0 loc_lane_bound n_lanes*size_lane];

% Lateral Position of the lane centers
for i=1:1:n_lanes
    loc_lane_cent(i)=(all_bound(i)+all_bound(i+1))/2;
end

% Road Potenial Data
eta= 1000;
A_skew = 2; 
b_skew = 1e5;

const_r= [eta A_skew b_skew n_lanes all_bound loc_lane_cent];

%% Obstacle Data

% States of the Obstacle
x_o_0= [30 250 4.5 0 0 0;0.01 300 1.5 0 0 0]';
% x_o_0= [];
% Obstacle Potential Field Data
A= 1;
b= 0.01;
const_o= [A b];
 
size_veh_act= [2.5 1.5]; % Size of the vehicle
size_veh= size_veh_act;

%% Model of the Host Vehicle
% Sampling Time for discretization, T
T= 0.1;

% Data for Model

% Constants of the car (as taken from Snapper.EY) 
C_f= 98389;            % Negative front tire cornering stiffness
C_r= 198142;           % Negative rear tire cornering stiffness
L_wb= 1.7;             % Wheel base
l_r= size_veh(1,1)/2;  % Distance between rear axle and vehicle CoG
l_f= size_veh(1,1)/2;  % Distance between front axle and vehicle CoG
I_z= 2865.61;          % Vehicle yaw moment of inertia
m= 1625;               % Vehicle Mass

car_const= [C_f C_r L_wb l_r l_f I_z m];

%% MPC Formulation

% Control Horizon

N_p= 10;

% Weights for the Objective Function

% y= [v_x y v_y psi_dot psi]^T
lambda_y_APFMPC= [  50 0 0 0 0;
                    0 0 0 0 0;
                    0 0 1 0 0;
                    0 0 0 1 0; 
                    0 0 0 0 1;];

% for u= [F_x delta]^T
lambda_u_APFMPC= [  0.1       0;
                    0        1;];
                
lambda_delta_u_APFMPC= [    100   0;
                            0   10;];

lambda_delta_x_APFMPC= [    0 0 0 0 0 0;
                            0 0 0 0 0 0;
                            0 0 0 0 0 0;
                            0 0 0 0 0 0;
                            0 0 0 0 0 0;
                            0 0 0 0 0 0;];
lambda_o_APFMPC= 1;
lambda_r_APFMPC= 1;

%% Optimal Control Problem

% Minimum distance between cars should be atleast one car length
d0= size_veh(1,1);

% Detection time of the obstacle vehicle. Here assuming no detection time.
t1= 0; 

% Count used to safe the data at each step.
count= 0;

% Matrix used in the Conversion of Potential Data into State Data.
C_xy = [0 1 0 0 0 0;
        0 0 1 0 0 0];

% Here we are copying the initial data to the variables to be used inside
% the loop
x_o= x_o_0; % Obstacle Data
x_h= x_h_0; % Host vehicle State Data
u_h= u_h_0; % Host vehicle Input Data

% the range of velocities which can be taken are [0, 150 km/h], then in 
% m/s, the range is [0, 41.667]
v_x_max= 41.667;

% Maximum acceleration of the vehicles (m/s^2)
a_x_max= 3.92;

% Safety distance that is used to check of lane change is possible. 
d_safe_max= 230; % As maximum value of v_x is 41.667 m/s, we have a
% maximum safe distance (with v_x_obst= 0) as 223.95 m. Hence 230 m is used
% as the maximum safe distance as 224+(2*(l_HV + l_OV))

% Selection if the vehicle is moving towards the right or the left
rl = 'r';

while x_h(2,1)<=road_len 
    %% Set the flags for no data and lane change.
    % Find the lane on which the host vehicle is present
        for i= 1:1:n_lanes
            if x_h(3,1)>=all_bound(1,i) && x_h(3,1)<=all_bound(1,i+1)
                lane_host= i-1;
            end
        end
        if isempty(x_o)==1
            flag_ND=1;
            flag_ND_roi=1;
            flag_LC=0;
            lane_obst= [];
        elseif isempty(x_o)==0

            flag_ND=0;
            flag_LC=1;
            % Number of extra variables to be added to the system model to add the
            % MLD constraints. This can be done by first checking if the safe area
            % in the adjacent lane is empty or not and selecting a flag to indicate
            % so. Therefore we first have to check which vehicles are in the
            % adjacent lane

            % Check which lane number that the obstacles are in 
            for i= 1:1:n_lanes
                for j= 1:1:width(x_o)
                    if x_o(3,j)>=all_bound(1,i) && x_o(3,j)<=all_bound(1,i+1)
                        lane_obst(j,1)= i-1;
                    end 
                end
            end
            for i= 1:1:width(x_o)
                if abs(lane_obst(i,1)-lane_host)==1
%                     if abs(x_o(2,i)-x_h(2,1))<=d_safe_max % As maximum value of v_x is
%                         % 41.667 m/s, we have a maximum safe distance (with
%                         % v_x_obst= 0) as 223.95 m, we assume a maximum safe area
%                         % distance of 300m
                    if abs(x_h(2,1)- x_o(2,i))<=d_safe_max % As maximum value of v_x is
                        % 41.667 m/s, we have a maximum safe distance (with
                        % v_x_obst= 0) as 223.95 m, we assume a maximum safe area
                        % distance of 230 meters. Also we double this
                        % distance so that we do no change lane in case
                        % there is a slower vehicle in the adjacent lane
                        flag_LC= 0;
                    end
                end
            end
        end
        
    %% Host Vehicle Model
    
    % Model of the host vehicle for APF-MPC
    ss_d_APFMPC= car_modelAPFMPC(car_const, T, x_h, u_h);
    
    %% Obstacle Model

    ss_d_obs_APFMPC= car_model_obs(T, 'APF-MPC');
    
    %% Constraints Boundaries 
    if flag_ND==1
        distance= [];
    elseif flag_ND==0
        for i= 1:1:width(x_o)
            distance(i,1)= (x_h(1,1)^2/(2*a_x_max)) - (x_o(1,i)^2/(2*a_x_max)) +x_h(1,1)*t1 + d0;
            if distance(i,1)<16
                distance(i,1)=16;
            end
        end
    end

    % Function used to generate the limits which define the boundary
    % constraints the host vehicle.

    [a_x_max, f_ineq, b_u_ineq]= mpc_constraints2(...
        car_const, x_h, const_r, size_veh, v_x_max);
    
    %% Extended Reference Trajectory
    
    [l_ref, y_traj_APFMPC, u_traj_APFMPC] = ref_traj_st(x_h, const_r, x_o,...
        flag_LC, flag_ND, x_h_0, size_veh, distance);
        
    %% Cost Function and Optimization
    yalmip('clear');
%     d= d_o;
    x_o1= x_o;

    % Variable for the acceleration of the host vehicle over the prediction horizon
    u = sdpvar(repmat(height(u_h_0),1,N_p),repmat(1,1,N_p)); 
    % Variable for the velocity of the host vehicle over the prediction horizon
    x = sdpvar(repmat(height(x_h_0),1,N_p+1),repmat(1,1,N_p+1));  
    % Variable for the intial velocity of the host vehicle
    x_init= sdpvar(height(x_h_0),1); % 
    % Variable for the initial lane of the host vehicle
    u_init= sdpvar(height(u_h_0),1); % 
    % Assigning the initial variables defined to the respective variables
    % on the extended variable set.
    x{1,1}= x_init;
    u{1,1}= u_init;
    % Define the initial constraints and objective. 
    constraints= [];
    objective= 0;
    
    for i=1:N_p   
        %% APF Design and Approximation  

        % % Here select should be one of the three things depending of it the data
        % % being generated is road data, the lane data, the obstacle data and the
        % % road + lane data. 
        % % if road data, then select= 'road'
        % % if lane data, then select= 'lane'
        % % if obstacle data, then select= 'obst'
        % % if road + lane data, then select= 'rplusl'
        if flag_ND==1
            U_0_o= 0;
            U_1_o= [0 0];
            U_2_o= zeros(2,2);
        elseif flag_ND==0
            x_o_temp= x_o1;
            x_o_temp= ss_d_obs_APFMPC.A* x_o_temp;
            x_o1= x_o_temp;
            [U_0_o, U_1_o, U_2_o]= taylor2ndOrder(const_r, const_o,...
                x_o1, x_h, 'obst', rl, distance, size_veh, l_ref);
        end
        [U_0_r, U_1_r, U_2_r]= taylor2ndOrder(const_r, const_o,...
            x_o, x_h, 'road', rl, distance, size_veh, l_ref);
        
        %% Cost Function
        if i==1
        objective= objective + (ss_d_APFMPC.C*x{1,i+1}-y_traj_APFMPC)'*lambda_y_APFMPC*(ss_d_APFMPC.C*x{1,i+1}-y_traj_APFMPC) +...
            (u{1,i}-u_traj_APFMPC)'*lambda_u_APFMPC*(u{1,i}-u_traj_APFMPC) +...
            (u{1,i})'*lambda_delta_u_APFMPC* (u{1,i}) + (x{1,i+1}-x{1,i})'*lambda_delta_x_APFMPC* (x{1,i+1}-x{1,i}) + ...
            lambda_r_APFMPC* (U_0_r + U_1_r * C_xy * x{1,i+1} + 0.5 * (C_xy * x{1,i+1})' * U_2_r * (C_xy * x{1,i+1})) +...
            lambda_o_APFMPC* (U_0_o + U_1_o * C_xy * x{1,i+1} + 0.5 * (C_xy * x{1,i+1})' * U_2_o * (C_xy * x{1,i+1}));
        else
        objective= objective + (ss_d_APFMPC.C*x{1,i+1}-y_traj_APFMPC)'*lambda_y_APFMPC*(ss_d_APFMPC.C*x{1,i+1}-y_traj_APFMPC) +...
            (u{1,i}-u_traj_APFMPC)'*lambda_u_APFMPC*(u{1,i}-u_traj_APFMPC) +...
            (u{1,i}-u{1,i-1})'*lambda_delta_u_APFMPC* (u{1,i}-u{1,i-1}) + (x{1,i+1}-x{1,i})'*lambda_delta_x_APFMPC* (x{1,i+1}-x{1,i}) + ...
            lambda_r_APFMPC* (U_0_r + U_1_r * C_xy * x{1,i+1} + 0.5 * (C_xy * x{1,i+1})' * U_2_r * (C_xy * x{1,i+1})) +...
            lambda_o_APFMPC* (U_0_o + U_1_o * C_xy * x{1,i+1} + 0.5 * (C_xy * x{1,i+1})' * U_2_o * (C_xy * x{1,i+1}));
        end
        
        %% Constraints
        constraints= [constraints, x{1,i+1}== ss_d_APFMPC.A*x{1,i}+ ss_d_APFMPC.B*u{1,i}];
        constraints= [constraints, f_ineq(2,1) <= x{1,i+1}(1,1) <= f_ineq(1,1),...
                                   f_ineq(4,1) <= x{1,i+1}(3,1) <= f_ineq(3,1),...
                                   f_ineq(6,1) <= x{1,i+1}(4,1) <= f_ineq(5,1),...
                                   f_ineq(8,1) <= x{1,i+1}(5,1) <= f_ineq(7,1), ...
                                   f_ineq(10,1) <= x{1,i+1}(6,1) <= f_ineq(9,1),...
                                   b_u_ineq(2,1) <= u{1,i}(1,1) <= b_u_ineq(1,1),...
                                   b_u_ineq(4,1) <= u{1,i}(2,1) <= b_u_ineq(3,1)];
%         if i==1
%             constraints= [constraints, -0.2<=((u{1,i}(1,1)-u_h(1,1)))<=0.5];
%         elseif i>1
%             constraints= [constraints, -0.2<=((u{1,i}(1,1)-u{1,i-1}(1,1)))<=0.5];
%         end
%         if i==1
%             constraints= [constraints, -0.05<=(u{1,i}(2,1)-u_h(2,1))<=0.05];
%         elseif i>1
%             constraints= [constraints, -0.05<=(u{1,i}(2,1)-u{1,i-1}(2,1))<=0.05];
%         end
    end
    
    %% Optimization
    ops= sdpsettings('solver','gurobi','verbose', 2);
    optimize([constraints, x_init== x_h],objective, ops);
%     optimize(constraints,objective);
    u_h= value(u{1});
    x_h= value(x{2});
    
    display(x_h')
    display(u_h')
    % Saving the values of the states and the input
    count= count + 1;
    if flag_ND==0
        x_o= ss_d_obs_APFMPC.A* x_o;
        for i= 1:width(x_o)
            v_o_list(count,i)= x_o(1,i);
            x_o_list(count,i)= x_o(2,i);
            y_o_list(count,i)= x_o(3,i);
            X_obs_list{i,count}= x_o(:,i);
        end
    end

    a_HV_list(count,1)= u_h(1,1); 
    del_HV_list(count,1)= u_h(2,1); 
    v_x_HV_list(count,1)= x_h(1,1);
    x_HV_list(count,1)= x_h(2,1);
    y_HV_list(count,1)= x_h(3,1);
    v_y_HV_list(count,1)= x_h(4,1);
    flag_ND_list(count,1)= flag_ND;
    flag_LC_list(count,1)= flag_LC;
    u_h_list{count}= u_h;
    X_list{count}= x_h;
    Y_ref_list{count}= y_traj_APFMPC;
    u_ref_list{count}= u_traj_APFMPC;
    
    l_opt_list(count,1)= l_ref;
    
    if x_h(3,1)<=size_lane
        l_act_list(count,1)= 0;
    elseif x_h(3,1)> size_lane
        l_act_list(count,1)=1;
    end
    
    a_ref_HV_list(count,1)= u_traj_APFMPC(1,1);
    del_ref_HV_list(count,1)= u_traj_APFMPC(2,1);
    v_x_ref_HV_list(count,1)= y_traj_APFMPC(1,1);
    
    %% Plot 1    
    name= ['Fig' num2str(count)];
    R= rem(count,10);
    if count==536
        display('test')
    end

    if (R==0)
    figure(count)
    subplot(6,1,1)
    hold on;
    if flag_ND==0
        for i=1:1:width(x_o)
            disname= sprintf('Path OV %d',i);
            plot(x_o_list(:,i),y_o_list(:,i), 'Color','k', 'DisplayName',disname);
        end
    %     distance=0;
        [bound_OV1]= rect_p(size_veh, x_o, rl);
        [tria_v1]= trian(bound_OV1, distance, x_o);
        rect_plot(bound_OV1, tria_v1,'obst');
    end
    [bound_OV2]= rect_p(size_veh, x_h, rl);
    tria_v2= [];
    rect_plot(bound_OV2, tria_v2,'host');
    plotdata2(const_r)
    plot(x_HV_list,y_HV_list, 'Color','b', 'DisplayName','Path HV')
    ylim(loc_road_bound);
    xlim([0 road_len]);
%     ylim(loc_road_bound);
%     xlim([x_h(2,1)-10 x_h(2,1)+10]);
%     legend('Path of HV','Path of OV1','Path of OV2','Lane Boundaries',...
%         'Road Boundaries', 'Lane Centers', 'Initial Obstacle Position',...
%         'Final Obstacle Position');
    legend
    xlabel('Longitudinal Distance') 
    ylabel('Lateral Distance') 
    title('Basic MPC with reference trajectory given with prediction horizon, N_p =10 & T = 0.1')
    subplot(6,1,2)
    hold on;
    plot(x_HV_list,v_x_HV_list, 'Color','b', 'DisplayName',' Longitudinal HV Vel')
    plot(x_HV_list,v_x_ref_HV_list, 'r--', 'DisplayName',' Reference HV Vel')
    if flag_ND==0
        for i=1:1:width(x_o)
            disname= sprintf('OV %d Velocity ',i);
            plot(x_o_list(:,i),v_o_list(:,i), 'Color','k', 'DisplayName',disname);
        end
    end
    xlabel('Distance in the Longitudinal Direction') 
    ylabel('Longitudinal Velocity')
    title('Long Velocity')
    legend
    subplot(6,1,3)
    hold on;    
    plot(x_HV_list,flag_LC_list, 'Color','b', 'DisplayName','Flag LC')
    plot(x_HV_list,flag_ND_list,'g', 'DisplayName','Flag ND')
    xlabel('Longitudinal Distance') 
    ylabel('Flag Value')
    title('Flags for lane change and no data')
    legend
    subplot(6,1,4)
    hold on;
    plot(x_HV_list,l_act_list, 'Color','b', 'DisplayName','Actual Lane of the HV')
    plot(x_HV_list,l_opt_list,'r--', 'DisplayName','Optimal Lane')
    xlabel('Longitudinal Distance') 
    ylabel('Lane')
    title('Lane of HV')
    legend
    subplot(6,1,5)
    hold on;
    plot(x_HV_list,a_HV_list,'DisplayName','Acceleration', 'Color','b')
    plot(x_HV_list,a_ref_HV_list,'r--','DisplayName','Ref Acceleration')
%     for i= 1:1:width(x_o)
%         disname= sprintf('Negative Distance between the HV and the OV %d',i);
%         plot(X_h_list,dist_metric,'DisplayName',disname)
%     end
    xlabel('Longitudinal Distance') 
    ylabel('Acceleration')
    title('Acceleration of the HV')
    legend
    subplot(6,1,6)
    hold on;
    plot(x_HV_list,del_HV_list,'DisplayName','Steering angle', 'Color','b')
    plot(x_HV_list,del_ref_HV_list,'r--','DisplayName','Ref Steering angle')
%     if flag_ND==0
%         for i= 1:1:width(x_o)
%             disname= sprintf('Distance between the HV and the OV %d',i);
%             plot(x_HV_list,dist_metric,'DisplayName',disname)
%         end
%     end
%     xlabel('Longitudinal Distance') 
%     ylabel('Distance')
%     title('Distance Between the HV and the OV')
    xlabel('Longitudinal Distance') 
    ylabel('Steering angle')
    title('Steering Angle of the Host Vehicle')
    legend
    end
%     end

end
X_list= horzcat(X_list{:});
u_h_list= horzcat(u_h_list{:});
X_obs_list= cell2mat(X_obs_list);
Y_ref_list= cell2mat(Y_ref_list);
u_ref_list= cell2mat(u_ref_list);

%% Functions
% Prediction car model used in the MPC Problem. Use of a dynamics vehicle
% model with longitudinal velocity constant for all time. Linear model.
function ss_d= car_modelMIMPC(T, x_o, flag_LC, flag_ND)
    if flag_LC== 1% This implies that the lane change is possible
     n_MLD_var= 11;
     n= n_MLD_var*width(x_o);% Number of extra inputs to be added to the host vehicle
    % SS model (not including F_x, delta and l_HV) when flag_SF==1
%     n=1;
    elseif flag_LC==0% This implies that the lane change is not possible
     n_MLD_var= 11;
     n= n_MLD_var*width(x_o);% Number of extra inputs to be added to the host vehicle
    % SS model (not including F_x, delta and l_HV) when flag_SF==0
    end
    % Matrices of the continuous-time car model using ZOH
    A_d= [  1   0;
            0   1];

    B_d1= [ 0;
            0];
    
    B_d= [B_d1 zeros(2,n)];
    
    % Continuous-time car model
    ss_d= ss(A_d, B_d, zeros(2,2), 0, T);

end

% Prediction car model used in the MPC Problem. Use of a dynamics vehicle
% model with varying longitudinal velocity. Linearized model. See notes
function ss_d= car_modelAPFMPC(const, T, x_0, u)
%% Input the define the linearized model matrices.
    C_f= const(1);        % Negative front tire cornering stiffness
    C_r= const(2);       % Negative rear tire cornering stiffness
    l_r= const(3);         % Distance between rear axle and vehicle CoG
    l_f= const(4);         % Distance between front axle and vehicle CoG
    I_z= const(5);       % Vehicle yaw moment of inertia
    m= const(6);            % Vehicle Mass
    
    v_x = x_0(1,1);
    x = x_0(2,1);
    y = x_0(3,1);
    v_y = x_0(4,1);
    psi_dot = x_0(5,1);
    psi = x_0(6,1);
    
    Fx= u(1,1);
    delta= u(2,1);
%%
    alpha1= v_y+ l_f*psi_dot;
    alpha2= v_y- l_r*psi_dot;
    alpha3= (v_y+ l_f*psi_dot)/ v_x;
    alpha4= (v_y- l_r*psi_dot)/ v_x;
    alpha5= atand((v_y+ l_f*psi_dot)/ v_x);
    alpha6= alpha3^2+1;
    alpha7= alpha4^2+1;
    
    A11= -(2*C_f*sind(delta)*alpha1)/(m*alpha6*v_x^2);
    A12= 0; 
    A13= 0;
    A14= (2*C_f*sind(delta))/(m*alpha6*v_x)+psi_dot;
    A15= (2*C_f*l_f*sind(delta))/(m*alpha6*v_x)+v_y;
    A16= 0;
    
    A21= cosd(psi);
    A22= 0;
    A23= 0;
    A24= -sind(psi);
    A25= 0;
    A26= -v_x*sind(psi)-v_y*cosd(psi);
    
    A31= sind(psi);
    A32= 0;
    A33= 0;
    A34= cosd(psi);
    A35= 0;
    A36= v_x*cosd(psi)-v_y*sind(psi);
    
    A41= (2*C_r*alpha2)/(m*alpha7*v_x^2)+ (2*C_f*cosd(delta)*alpha1)/(m*alpha6*v_x^2)-psi_dot;
    A42= 0;
    A43= 0;
    A44= -(2*C_r)/(m*alpha7*v_x)-(2*C_f*cosd(delta))/(m*alpha6*v_x);
    A45= -(2*C_r*l_r)/(m*alpha7*v_x)-(2*C_f*l_f*cosd(delta))/(m*alpha6*v_x)-v_x;
    A46= 0;
    
    A51= (2*C_f*l_f*cosd(delta)*alpha1)/(I_z*alpha6*v_x^2)-(2*C_r*l_r*alpha2)/(I_z*alpha7*v_x^2);
    A52= 0;
    A53= 0;
    A54= (2*C_r*l_r)/(I_z*v_x*alpha7)-(2*C_f*l_f*cosd(delta))/(I_z*v_x*alpha6);
    A55= -(2*C_r*l_r^2)/(I_z*v_x*alpha7)-(2*C_f*l_f^2*cosd(delta))/(I_z*v_x*alpha6);
    A56= 0;
    
    A61= (tand(delta))/(l_f+l_r);
    A62= 0;
    A63= 0;
    A64= 0;
    A65= 0;
    A66= 0;
    
    
    B11= 1;
    B12= -(2*C_f*(sind(delta)+(delta- atand(alpha3))*cosd(delta)))/m;
    
    B21= 0;
    B22= 0;
    
    B31= 0;
    B32= 0;
    
    B41= 0;
    B42= -(2*C_f*((delta-atand(alpha3))*sind(delta)-cosd(delta)))/m;
    
    B51= 0;
    B52= -(2*C_f*l_f*((delta-atand(alpha3))*sind(delta)-cosd(delta)))/I_z;
    
    B61= 0;
    B62= (v_x*(secd(delta)^2))/(l_f+l_r);

    % Matrices of the continuous-time car model using ZOH
    A= [A11 A12 A13 A14 A15 A16;
        A21 A22 A23 A24 A25 A26;
        A31 A32 A33 A34 A35 A36;
        A41 A42 A43 A44 A45 A46;
        A51 A52 A53 A54 A55 A56;
        A61 A62 A63 A64 A65 A66;];

    B= [B11 B12;
        B21 B22;
        B31 B32;
        B41 B42;
        B51 B52;
        B61 B62;];
%% 
    C= [    1       0       0       0       0       0;
            0       0       1       0       0       0;
            0       0       0       1       0       0;
            0       0       0       0       1       0;
            0       0       0       0       0       1;];

    if v_x==0
        A= [        0	0	0	0   0   0;
                    0   1   0   0   0   0;
                    0   0   0   0   0   0;
                    0   0   0   0   0   0;
                    0   0   0   0   0   0;
                    0   0   0   0   0   0;];
        B= [    0       0;
                0       0;
                0       0;
                0       0;
                0       0;
                0       0;];
    end
        ss_c= ss(A, B, C, 0);


    % Matrices of the discretized car model using ZOH

%     A_d= expm(A_c*T);
%     fun= @(t)expm(A_c*t);
%     B_d= integral(fun,0,T,'ArrayValued',true)*B_c;
%     C_d= C_c;

%     % Discretized car model
%     ss_d= ss(A_d, B_d, C_d, 0,T);
    ss_d= c2d(ss_c, T, 'zoh');
end

% Prediction model for the obstacle vehicle (constant velocity)
function ss_d= car_model_obs(T, select)

    if isequal(select, 'MIMPC')
        A_obs= [1 0;
                0 1];
        B_obs= zeros(width(A_obs),2);
        C_obs= eye(width(A_obs));
    elseif isequal(select, 'APF-MPC')
        A_obs= [1 0 0 0 0 0;
                T 1 0 0 0 0;
                0 0 1 T 0 0;
                0 0 0 1 0 0;
                0 0 0 0 0 0;
                0 0 0 0 0 0;];
        B_obs= zeros(width(A_obs),2);
        C_obs= [eye(4) zeros(4,2)];
    end
    ss_d= ss(A_obs, B_obs, C_obs, 0, T);
        
end

% Function to Generate the extended matrices of the constraint equations
function [a_x_max, f_ineq, b_u_ineq]= mpc_constraints2(...
     car_const, x_h, const_r, size_veh, v)
%%

% The idea here is to generate the matrices for constraints. The
% constraints consists of constraints on the state, input and output. The
% states with constraints are are velocity (longitudinal and lateral), lateral position, yaw
% rate and heading angle. There are also constraints on the maximum lateral
% and longitudinal acceleration and deceleration. 
    eta= const_r(1,1);
    A_skew= const_r(1,2);
    b_skew= const_r(1,3);
    n_lanes = const_r(1,4);
    all_bound= const_r(1, 5:6+ n_lanes-1);
    loc_lane_bound= all_bound(1, 2:2+n_lanes-2);
    loc_road_bound= [all_bound(1,1) all_bound(1,end)];
    loc_lane_cent= const_r(1, 5+n_lanes+1:end);
% For the inputs the maximum and minimum value of the longitudinal force 
% is given by the maximum and minimum values of the logitudinal 
% acceleration and deceleration. The maximum and minimum steering angle
% should depend on the location of the car (lane) as well as it velocity.
v_x=x_h(1,1);
m= car_const(1,7);
% m= car2_const(1,6);
% L_wb= car2_const(1,3);

% For lateral Position, the bounds are given by the road boundaries.
Y_max= loc_road_bound(1,2);
Y_min= loc_road_bound(1,1);

% For lateral velocity, the boundaries are given by the maximum/minimim
% total velocity as well as the maximum/minimum slip angle of the vehicle.
% Assumed that the range of velocities which can be taken are [0, 150
% km/h], then in m/s, the range is [0, 41.667]
Vmax= v;
% For lateral velocity, the boundaries are given by the maximum/minimim
% total velocity as well as the maximum/minimum slip angle of the vehicle.
% Assumed that the range of velocities which can be taken are [0, 150
% km/h], then in m/s, the range is [0, 41.667]
beta_max= 3;

% For heading angle. This is a an average of the vehicle angle used for
% lane change
psi_max= 5;
psi_min= -psi_max;

% For steering angle and the longitudinal force. Maximum steering angle is
% usually given as pi/6. But
% it is  calculated here as a function of the velocity of the vehicle, the
% maximum acceleration and then wheelbase as
delta_max_rad= size_veh(1,2)/v_x;
% delta_max_rad= ((size_veh(1,2)*a_y_max)/v_x^2);
delta_max= (180/pi)*delta_max_rad;
delta_min= -delta_max;

% For yaw rate. It is dependent on the coefficient of
% friction of the road. The max is chosen as if the car is on wet roads
% with mu= 0.4;
mu= 0.4; 
g= 9.81;
a_y_max= mu * g;
% % the maximum longitudinal acceleration is also defined as it helps define
% % the maximum longitudinal force. Have to ask about this. 
a_x_max= a_y_max;
a_x_min= -a_x_max;

r_max= round(a_y_max)/v_x;
r_min= -r_max;
F_x_max= m * a_x_max;
F_x_min= m * a_x_min;

l_max= n_lanes-1;
l_min= 0;

V_max= Vmax;
V_min= 0;
v_y_max= V_max * sind(beta_max);
v_x_max= V_max * cosd(beta_max);
v_x_min= V_min * cosd(beta_max);
% v_x_min= -v_x_max;
v_y_min= -v_y_max;

f_ineq= [v_x_max v_x_min Y_max Y_min v_y_max v_y_min r_max r_min psi_max psi_min l_max l_min]';
b_u_ineq = [a_x_max a_x_min delta_max delta_min]';
end

% Extended reference trajectory used. Ideally the car should be in the
% right lane, in the centre with no heading angle, lateral velocity or yaw
% motion. Also no input to be used in the lateral direction. 
function [l_ref, y_traj_APFMPC, u_traj_APFMPC]= ref_traj_st(x_h,...
    const_r, x_o, flag_LC, flag_ND, x_h_0, size_veh, distance)
% function [y_traj u_traj]= ref_traj_st(N_p, select)

    eta= const_r(1,1);
    A_skew= const_r(1,2);
    b_skew= const_r(1,3);
    n_lanes = const_r(1,4);
    all_bound= const_r(1, 5:6+ n_lanes-1);
    loc_lane_bound= all_bound(1, 2:2+n_lanes-2);
    loc_road_bound= [all_bound(1,1) all_bound(1,end)];
    loc_lane_cent= const_r(1, 5+n_lanes+1:end);
    
    % The output reference is to keep the lateral velocity, the yaw rate and
    % the yaw angle as close to zero as possible.
    
%%

    % Find the lane on which the host vehicle is present
    for i= 1:1:n_lanes
        if x_h(3,1)>=all_bound(1,i) && x_h(3,1)<=all_bound(1,i+1)
            lane_host= i-1;
        end
    end
    
    if flag_ND==1
        flag_delta=0;
        x_o_front_SL= [];
        flag_epsilon1=0;
        flag_epsilon2=0;
    elseif flag_ND==0
        
        % Find the lanes on which the obstacle vehicles is present
        for i= 1:1:n_lanes
            for j= 1:1:width(x_o)
                if x_o(3,j)>=all_bound(1,i) && x_o(3,j)<=all_bound(1,i+1)
                    lane_obst(j,1)= i-1;
                end
            end
        end

        % Check if an obstacle is on the same lane or the adjacent lanes 
        % as the host vehicle and to save the index of such obstacle
        count0= 0;
        count1= 0;
        count11=0;
        flag_alpha1= zeros(width(x_o),1);
        flag_alpha2= zeros(width(x_o),1);
        for i= 1:1:width(x_o)
            if abs(lane_obst(i,1)-lane_host)==0% To check the total number of vehicles as the 
                % same lane as the HV
                flag_alpha1(i,1)=1; % set this flag if the corresponding OV is in 
                % the same lane as the HV
                count0= count0+1; % number of OV in the same lane and the HV
                index0(count0,1)= i; % Index of the obstacles on the same lane as
                % the host vehicle
            elseif abs(lane_obst(i,1)-lane_host)==1% To check the which OV's
                % are on adjacent lane of the HV
                count1= count1+1; % number of vehicles in the adjacent lane of the HV
                flag_alpha2(i,1)= 1;% set this flag if the corresponding OV if
                % if it on the adjacent lane of the HV
                index1(count1,1)=i;% Index of the obstacles which are on 
                % the adjacent lane of the HV
            else % To check which vehicles are farther away from the HV by more
                % than one lane.
                count11= count11+1;% total number of vehicles which are farther 
                % away from the HV by more than one lane.
                index2(count1,1)=i;% index of vehicles which are farther 
                % away from the HV by more than one lane.
            end
        end

        % Check which obstacles in the same lane as the HV are infront of and 
        % behind the HV
        count21= 0;
        count22= 0;
        if count0>=1
            for i= 1:1:count0
                if x_o(2,index0(i,1))>=x_h(2,1)
                    count21= count21+1;% number of obstacles in front of the HV
                    flag_beta1(i,1)= 1;% flag is 1 if the vehicle is infront of the HV
                    index21(count21,1)= index0(i,1); % Index of the vehicles on the same
                    % lane as the host vehicle and in front of the host vehicle
                elseif x_o(2,index0(i,1))<x_h(2,1)
                    count22= count22+1;% number of obstacles behind the HV
                    flag_beta2(i,1)= 1;% flag is 1 if the vehicle is behind the HV
                    index22(count22,1)= index0(i,1); % Index of the vehicles on the same
                    % lane as the host vehicle and behind of the host vehicle
                end
            end
        end

        % Check which obstacles in the adjacent lane as the HV are infront of and 
        % behind the HV
        count31= 0;
        count32= 0;
        if count1>=1
            for i= 1:1:count1
                if x_o(2,index1(i,1))>x_h(2,1)
                    count31= count31+1;% number of OV in the adjacent lane in front of the HV
                    flag_gamma1(i,1)= 1;% flag is 1 if the OV in the adjacent lane is 
                    % infront of the HV
                    index31(count31,1)= index1(i,1); % Index of the vehicles on the adjacent
                    % lane as the host vehicle and in front of the host vehicle
                elseif x_o(2,index1(i,1))<=x_h(2,1)
                    count32= count32+1;% number of OV in the adjacent lane behind of the HV
                    flag_gamma2(i,1)= 1;% flag is 1 if the OV in the adjacent lane is 
                    % behind of the HV
                    index32(count32,1)= index1(i,1); % Index of the vehicles on the adjacent
                    % lane as the host vehicle and behind of the host vehicle
                end
            end
        end

        % Here to find the OV which is in front of the vehicle in the same
        % lane and to find the OV in front of and behind the vehicle in the adjacent
        % lane. Found by first making a matrix of the all the vehicle data of the
        % obstacles in front of the HV and then finding the minimum value of x of
        % them and then setting them to a value. 
        if count0>=1
            if count21>=1
                flag_delta= 1; % This shows that there is a vehicle 
                % in front of the HV in the same lane
                for i=1:1:count21
                    % Here we have to find the index of the obstacle which has the
                    % least x value or in other words find the obstacle right infrnt of
                    % the HV in the same lane. 
                    x_o_scrut1{1,i}=x_o(:,index21(i,1)); 
                end
                x_o_scrut1= horzcat(x_o_scrut1{:});
                [min_x, index41]= min(x_o_scrut1(2,:));
                x_o_front_SL= x_o_scrut1(:,index41);
                index41= index21(index41,1);
            else
                flag_delta= 0; % This shows that there is no vehicle  
                % in front of the HV in the same lane
            end
        else
            flag_delta= 0; % This shows that there is no vehicle  
            % in front of the HV in the same lane
        end

        % similarly for obstacles in the adjacent lane to find the OV right infront
        % of the HV
        if count1>=1
            if count31>=1
                flag_epsilon1= 1; % This shows that there is a vehicle 
                % in front of the HV in the adjacent lane
                for i=1:1:count31
                    % Here we have to find the index of the obstacle which has the
                    % least x value or in other words find the obstacle right infrnt of
                    % the HV in the adjacent lane. 
                    x_o_scrut2{1,i}=x_o(:,index31(i,1)); 
                end
                x_o_scrut2= horzcat(x_o_scrut2{:});
                [min_x, index42]= min(x_o_scrut2(2,:));
                x_o_front_AL= x_o_scrut2(:,index42);
                index42= index31(index42,1);
            else
                flag_epsilon1= 0; % This shows that there is no vehicle  
                % in front of the HV in the adjacent lane
            end
        else
            flag_epsilon1= 0; % This shows that there is no vehicle  
            % in front of the HV in the adjacent lane
        end

        % similarly for obstacles in the adjacent lane to find the OV right behind the HV
        if count1>=1
            if count32>=1
                flag_epsilon2= 1; % This shows that there is a vehicle 
                % behind the HV in the adjacent lane
                for i=1:1:count32
                    % Here we have to find the index of the obstacle which has the
                    % least x value or in other words find the obstacle right infrnt of
                    % the HV inthe same lane. 
                    x_o_scrut3{1,i}=x_o(:,index32(i,1)); 
                end
                x_o_scrut3= horzcat(x_o_scrut3{:});
                [min_x, index43]= max(x_o_scrut3(2,:));
                x_o_rear_AL= x_o_scrut3(:,index43);
                index43= index32(index43,1);
            else
                flag_epsilon2= 0; % This shows that there is no vehicle  
                % behind the HV in the adjacent lane
            end
        else
            flag_epsilon2= 0; % This shows that there is no vehicle  
                % behind the HV in the adjacent lane
        end
    end

%%        

    vy_ref= 0;
    r_ref= 0; 
    psi_ref= 0;
    F_x_ref= 0;
    delta_ref= 0;

    if flag_ND==1
        l_ref= lane_host;
        vx_ref= x_h_0(1,1);
    elseif flag_ND==0
        if flag_delta==1 % implies that there is a vehicle infront of the host vehicle. 
            if flag_LC==0 % Implies that there is a vehicle in the adjacent lane 
                % in the safety region. Then stay in the current lane.
                l_ref= lane_host;
                vx_ref= x_h_0(1,1);
                if x_o_front_SL(2,1)- x_h(2,1)<=distance(index41)+230
                    vx_ref= x_o_front_SL(1,1);
                end
            elseif flag_LC==1 % Implies that there is no vehicle in the adjacent
                % lane in the safety region.
                vx_ref= x_h_0(1,1);
                if x_o_front_SL(2,1)- x_h(2,1)<=distance(index41)+20*size_veh(1,1)
                    if lane_host==1
                        l_ref=0;
                    elseif lane_host==0
                        l_ref=1;
                    end                             
                else
%                     vx_ref= x_h_0(1,1);
                    l_ref= lane_host;
                end
            end
        elseif flag_delta==0
            vx_ref= x_h_0(1,1);
            l_ref= lane_host;
        end
    end
    if l_ref==0
        y_ref= 1.5;
    elseif l_ref==1
        y_ref= 4.5;
    end
    
    u_traj_APFMPC(1,1)= F_x_ref;
    u_traj_APFMPC(2,1)= delta_ref;

    
    y_traj_APFMPC(1,1)= vx_ref;
    y_traj_APFMPC(2,1)= y_ref;
    y_traj_APFMPC(3,1)= vy_ref;
    y_traj_APFMPC(4,1)= r_ref;
    y_traj_APFMPC(5,1)= psi_ref;
    
 
end

% Function to Generate the extended matrices of the state space system
function [tilde_A tilde_B tilde_C tilde_A_u tilde_A_minus tilde_B_minus]= mpc_model(N_p, ss_d)

    %% Derivation of A_tilde

    for i= 1:N_p
        tilde_A{i,1}=  (ss_d.A)^i;
    end
    tilde_A= vertcat(tilde_A{:});

    %% Derivation of B_tilde

    for i= 1:N_p
        tilde_B_temp1{i,1}=  (ss_d.A^(i-1)) * ss_d.B;
    end
    tilde_B_temp1= vertcat(tilde_B_temp1{:});
    [row_tilde_B_temp1,col_tilde_B_temp1]= size(tilde_B_temp1);
    tilde_B= zeros(row_tilde_B_temp1, col_tilde_B_temp1*N_p);
    for i=1:N_p
        alpha= col_tilde_B_temp1*i-(col_tilde_B_temp1-1):col_tilde_B_temp1*i;
        tilde_B(height(ss_d.B)*(i-1)+1:row_tilde_B_temp1,alpha)= tilde_B_temp1(1:...
            (end-height(ss_d.B)*(i-1)),:);
    end

    %% Derivation of C_tilde
    tilde_C_temp = repmat({ss_d.C}, 1, N_p);  
    tilde_C = blkdiag(tilde_C_temp{:});

   %% Derivation of tilde_A_minus
   
    [row_A, col_A]= size(ss_d.A);
    for i= 1:N_p
        tilde_A_minus{i,1}=  (ss_d.A-eye(row_A))*(ss_d.A)^(i-1);
    end
    tilde_A_minus= vertcat(tilde_A_minus{:});

    %% Derivation of tilde_B_minus
    for i= 1:N_p
        tilde_B_minus_temp1{i,1}=  (ss_d.A^(i-1)) * ss_d.B;
    end
    tilde_B_minus_temp2{1,1}= tilde_B_minus_temp1{1,1};
    for i= 1:N_p-1
        tilde_B_minus_temp2{i+1,1}=  tilde_B_minus_temp1{i+1,1}-tilde_B_minus_temp1{i,1};
    end
    tilde_B_minus_temp2= vertcat(tilde_B_minus_temp2{:});
    [row_tilde_B_minus_temp2,col_tilde_B_minus_temp2]= size(tilde_B_minus_temp2);
    tilde_B_minus= zeros(row_tilde_B_minus_temp2, col_tilde_B_minus_temp2*N_p);
    for i=1:N_p
        alpha= col_tilde_B_temp1*i-(col_tilde_B_temp1-1):col_tilde_B_temp1*i;
        tilde_B_minus(height(ss_d.B)*(i-1)+1:row_tilde_B_minus_temp2,alpha)= tilde_B_minus_temp2(1:...
                    (end-height(ss_d.B)*(i-1)),:);
    end    

    %% Derivation of A_u_tilde
    
%     tilde_A_u= zeros(width(ss_d.B)*N_p,width(ss_d.B)*N_p);
%     tilde_A_u(1:width(ss_d.B),1:width(ss_d.B))=eye(width(ss_d.B));
%     te1= zeros(1,width(ss_d.B)*N_p);
%     te2= zeros(width(ss_d.B)*(N_p-1),width(ss_d.B)*N_p);
%     temp= [-1 zeros(1,width(ss_d.B)-1) -1];
%     te1(1,1:width(temp))= temp;
%     for i=1:width(ss_d.B)*(N_p-1)
%         te2(i,i:end)= te1(1:(N_p+i-1)*width(ss_d.B)- (width(temp)*(i-1)));
%     end
%     tilde_A_u(width(ss_d.B)+1:end,:)= te2;
    temp= eye(width(ss_d.B));
    temp1= zeros(width(ss_d.B));
    for i= 1:1:N_p
        for j= 1:1:N_p
    tilde_A_u{i,j}= zeros(width(ss_d.B));
        end
    end
   tilde_A_u{1,1}= temp; 
    for i= 2:1:N_p
    tilde_A_u{i,i-1}= temp;
    tilde_A_u{i,i}= -temp;
    end
   tilde_A_u= cell2mat(tilde_A_u);

end

% Function to Generate the extended matrices of the state space system
function [tilde_A tilde_B tilde_C tilde_A_u tilde_A_minus tilde_B_minus]= mpc_model1(N_p, ss_d, x_h, u, const, T)
%     u= [1000 0]';
    % Basics
    A_temp{1,1}= ss_d.A;
    B_temp{1,1}= ss_d.B;
    C_temp{1,1}= ss_d.C;
    D_temp{1,1}= ss_d.D;
    x_temp{1,1}= x_h;
    for i= 2:1:N_p
        x_temp{i,1}= A_temp{i-1,1}*x_temp{i-1,1}+B_temp{i-1,1}*u;
        ss_d_temp= car_model2(const, T, x_temp{i,1}, u);
        A_temp{i,1}= ss_d_temp.A;
        B_temp{i,1}= ss_d_temp.B;
        C_temp{i,1}= ss_d_temp.C;
        D_temp{i,1}= ss_d_temp.D;
    end
    
%%
% Derivation of A_tilde

    for i= N_p:-1:1
        tilde_A{N_p-i+1,1}= matprodtilde(i,0,A_temp, N_p);
    end
    tilde_A= vertcat(tilde_A{:});    
%%
%     Derivation of B_tilde
    for i= 1:1:N_p
        B_temp1= B_temp{i,1};
        [r_b, c_b]= size(B_temp1);
        B_zero= zeros(r_b,c_b);
        if i>1
           for k= 2:1:i
               tilde_B{k-1,i}= B_zero;
           end
           tilde_B{i,i}= B_temp1;
        elseif i==1
            k=1;
            tilde_B{i,i}= B_temp1;
        end
        for l=k+1:1:N_p
            tilde_B{l,i}= matprodtilde(N_p-l+1,i,A_temp,N_p)*B_temp1;
        end
    end
    tilde_B= cell2mat(tilde_B);
%%
    % Derivation of C_tilde
    tilde_C= blkdiag(C_temp{:,1});
%%
%   % Derivation of tilde_A_minus
    [row_A, col_A]= size(ss_d.A);
    for i= 1:N_p
        tilde_A_minus{i,1}=  eye(row_A)*(ss_d.A)^(i-1);
    end
    tilde_A_minus= vertcat(tilde_A_minus{:});
%%
% %    % Derivation of tilde_B_minus
    for i= 1:N_p
        tilde_B_minus_temp1{i,1}=  (ss_d.A^(i-1)) * ss_d.B;
    end
    tilde_B_minus_temp2{1,1}= tilde_B_minus_temp1{1,1};
    for i= 1:N_p-1
        tilde_B_minus_temp2{i+1,1}=  tilde_B_minus_temp1{i+1,1}-tilde_B_minus_temp1{i,1};
    end
    tilde_B_minus_temp2= vertcat(tilde_B_minus_temp2{:});
    [row_tilde_B_minus_temp2,col_tilde_B_minus_temp2]= size(tilde_B_minus_temp2);
    tilde_B_minus= zeros(row_tilde_B_minus_temp2, col_tilde_B_minus_temp2*N_p);
    for i=1:N_p
        if col_tilde_B_minus_temp2==1
            alpha= i;
        elseif col_tilde_B_minus_temp2==2
            alpha= (2*i-1):2*i;
        end
        tilde_B_minus(6*(i-1)+1:row_tilde_B_minus_temp2,alpha)= tilde_B_minus_temp2(1:...
            (end-6*(i-1)),:);
    end
    
%     te1= zeros(1,N_p);
%     te2= zeros(N_p-1,N_p);
%     te1(1,1:2)= [-1 1];
%     for i=1:N_p-1
%         te2(i,i:end)= te1(1:N_p-i+1);
%     end
%     tilde_A_u= zeros(N_p,N_p);
%     tilde_A_u(2:end,:)= te2;
%     tilde_A_u(1,1)=1;
%%
    % Derivation of A_u_tilde
    
    row_tilde_B_temp1= height(ss_d.B)*N_p;
    col_tilde_B_temp1= width(ss_d.B);
    tilde_A_u= zeros(width(ss_d.B)*N_p,width(ss_d.B)*N_p);
    tilde_A_u(1:width(ss_d.B),1:width(ss_d.B))=eye(width(ss_d.B));
    te1= zeros(1,width(ss_d.B)*N_p);
    te2= zeros(width(ss_d.B)*(N_p-1),width(ss_d.B)*N_p);
    if width(ss_d.B)==1
        temp= [-1 1];
    elseif width(ss_d.B)==2
        temp= [-1 0 1];
    end   
    te1(1,1:width(temp))= temp;
    for i=1:width(ss_d.B)*(N_p-1)
        te2(i,i:end)= te1(1:(N_p+i-1)*width(ss_d.B)- (width(temp)*(i-1)));
    end
    tilde_A_u(width(ss_d.B)+1:end,:)= te2;

end

% Function for extended matrices of the weights of the cost function
function [tilde_lambda_y tilde_lambda_u tilde_lambda_delta tilde_lambda_delta_x tilde_lambda_r tilde_lambda_o]...
    = mpc_weights(N_p, lambda_y, lambda_u, lambda_delta, lambda_r, lambda_o, lambda_delta_x)
tilde_lambda_y= repmat({lambda_y}, 1, N_p);  
tilde_lambda_y = blkdiag(tilde_lambda_y{:});

tilde_lambda_u= repmat({lambda_u}, 1, N_p);  
tilde_lambda_u = blkdiag(tilde_lambda_u{:});

tilde_lambda_delta= repmat({lambda_delta}, 1, N_p);  
tilde_lambda_delta = blkdiag(tilde_lambda_delta{:});

tilde_lambda_r= repmat({lambda_r}, 1, N_p);  
tilde_lambda_r = blkdiag(tilde_lambda_r{:});

tilde_lambda_o= repmat({lambda_o}, 1, N_p);  
tilde_lambda_o = blkdiag(tilde_lambda_o{:});

tilde_lambda_delta_x= repmat({lambda_delta_x}, 1, N_p);  
tilde_lambda_delta_x= blkdiag(tilde_lambda_delta_x{:});
end

% Function which generates the matrices to be used in the quadratic cost
% function which represent the taylor series approximation of the APF.
function [A1 A2 A3]= taylor2ndOrder(const_r, const_o, x_o, x_h, select, rl, distance,size_veh, l_ref)
    A= const_o(1,1);
    b= const_o(1,2);
    eta= const_r(1,1);
    a_skew= const_r(1,2);
    b_skew= const_r(1,3);
    n_lanes = const_r(1,4);
    all_bound= const_r(1, 5:6+ n_lanes-1);
    loc_lane_bound= all_bound(1, 2:2+n_lanes-2);
    loc_road_bound= [all_bound(1,1) all_bound(1,end)];
    loc_lane_cent= const_r(1, 5+n_lanes+1:end);
    
    [row, col]=size(x_o);
    
    x= x_h(2,1);
    y= x_h(3,1);
    
    opt_l= l_ref; 
    
%     if opt_l==0
%         opt_y=1.5;
%     elseif opt_l==1
%         opt_y=4.5;
%     end
    % Given the vertices of the obstacle, we can find then distance from a
    % point to all the vertices and all the boundaries, and find the minimum of
    % these values. 
    [bound_OV]= rect_p(size_veh, x_o, rl);
    [tria_v]= trian(bound_OV, distance, x_o);
    [v_alp1 v_alp2]= vertex(bound_OV, tria_v);
    [si_form]= equations(v_alp1, v_alp2, x_o);
    
    switch select   
        case 'obst'
%% Obstacle Potential             
            % write the derivative for the obstacle potential for x, y, xx,
            % xy, yx, and yy. Substitute the actual values and then get the
            % values of A1, A2, and A3
            for i= 1:1:width(x_o)
                if x_o(6,i)>0
                    flag(i)= region1(si_form(:,i), x_h);
                elseif x_o(6,i)==0
                    flag(i)= region2(si_form(:,i), x_h, v_alp1{1,i});
                elseif x_o(6,i)<0
                    flag(i)= region3(si_form(:,i), x_h);
                end
                [K(i), apf_obst(i), obst_u_x(i), obst_u_y(i),...
                    obst_u_xx(i), obst_u_yy(i), obst_u_yx(i), obst_u_xy(i)]...
                    = dist(flag(i), v_alp1{:,i}, x_h, const_o);
            end
            %%
            a1= sum(apf_obst);
            a2= sum(obst_u_x);
            a3= sum(obst_u_y);
            a4= sum(obst_u_xx);
            a5= sum(obst_u_xy);
            a6= sum(obst_u_yx);
            a7= sum(obst_u_yy);
            
            A1= a1 - a2*x - a3*y + 0.5*a4*x^2 + a5*x*y + 0.5*a7*y^2;

            A2= [ a2 - a4*x - a5*y; 
                  a3 - a5*x - a7*y]';
 
            A3= [a4 a5;
                a6 a7];
        case 'road'
%% Road Potential
            % write the derivative for the obstacle potential for x, y, xx,
            % xy, yx, and yy. Substitute the actual values and then get the
            % values of A1, A2, and A3
%             for i = 1:length(loc_road_bound)
%                 apf_road(i)= 0.5* eta * (1/(y- loc_road_bound(i)))^2;
%                 obst_ur_x(i) = 0;
%                 obst_ur_y(i) = -eta/((y- loc_road_bound(i))^3);
%                 obst_ur_xx(i) = 0;
%                 obst_ur_xy(i) = 0;
%                 obst_ur_yx(i) = 0;
%                 obst_ur_yy(i) = (3*eta)/((y-loc_road_bound(i))^4);
%                 apf_road(i)= 0.5* eta * (1/(y- loc_road_bound(i)))^4;
%                 obst_ur_x(i) = 0;
%                 obst_ur_y(i) = -(2*eta)/((y- loc_road_bound(i))^5);
%                 obst_ur_xx(i) = 0;
%                 obst_ur_xy(i) = 0;
%                 obst_ur_yx(i) = 0;
%                 obst_ur_yy(i) = (10*eta)/((y-loc_road_bound(i))^6);
            if l_ref==0
                opt_y=1.5;
                apf_road= 0.5*eta*((1/(y- loc_road_bound(1)))^2+(1/(y- loc_road_bound(2)))^2)+b_skew*(exp(a_skew*(y-opt_y))-a_skew*(y-opt_y)-1);
                obst_ur_x = 0;
                obst_ur_y = -(eta/2)*((2/(y- loc_road_bound(1)))^3+(2/(y- loc_road_bound(2)))^3)+b_skew*(a_skew*exp(a_skew*(y-opt_y))-a_skew);
                obst_ur_xx = 0;
                obst_ur_xy = 0;
                obst_ur_yx = 0;
                obst_ur_yy = (eta/2)*((6/(y- loc_road_bound(1)))^4+(6/(y- loc_road_bound(2)))^4)+b_skew*(a_skew^2*exp(a_skew*(y-opt_y)));
            elseif l_ref==1
                opt_y=4.5;
                apf_road= 0.5*eta*((1/(y- loc_road_bound(1)))^2+(1/(y- loc_road_bound(2)))^2)+b_skew*(exp(-a_skew*(y-opt_y))+a_skew*(y-opt_y)-1);
                obst_ur_x = 0;
                obst_ur_y = -(eta/2)*((2/(y- loc_road_bound(1)))^3+(2/(y- loc_road_bound(2)))^3)+b_skew*(-a_skew*exp(-a_skew*(y-opt_y))+a_skew);
                obst_ur_xx = 0;
                obst_ur_xy = 0;
                obst_ur_yx = 0;
                obst_ur_yy = (eta/2)*((6/(y- loc_road_bound(1)))^4+(6/(y- loc_road_bound(2)))^4)+b_skew*(a_skew^2*exp(-a_skew*(y-opt_y)));   
            end
%             end
            a1= apf_road;
            a2= obst_ur_x;
            a3= obst_ur_y;
            a4= obst_ur_xx;
            a5= obst_ur_xy;
            a6= obst_ur_yx;
            a7= obst_ur_yy;
            A1= a1 - a2*x - a3*y + 0.5*a4*x^2 + a5*x*y + 0.5*a7*y^2;

            A2= [ a2 - a4*x - a5*y; 
                  a3 - a5*x - a7*y]';
 
            A3= [a4 a5;
                a6 a7];
        case 'lane'
%% Lane Potential
            % write the derivative for the obstacle potential for x, y, xx,
            % xy, yx, and yy. Substitute the actual values and then get the
            % values of A1, A2, and A3
            for i= 1: length(all_bound)
                beta_lx(i)= (y-all_bound(i))/sig_lane^2;
                alpha_lx(i)= ((y- all_bound(i))^2/(2* sig_lane^2));
                apf_lane(i)= A_lane* exp(-alpha_lx(i));
                obst_ul_x(i) = 0;
%                 obst_ul_y(i) = -beta_lx(i)*apf_lane(i);
                obst_ul_y(i) = -A_lane*beta_lx(i)*exp(-alpha_lx(i));
                obst_ul_xx(i) = 0;
                obst_ul_xy(i) = 0;
                obst_ul_yx(i) = 0;
                obst_ul_yy(i) = A_lane*beta_lx(i)^2*exp(-alpha_lx(i))-(A_lane*exp(-alpha_lx(i)))/sig_lane^2;
            end
            for i= 1: length(loc_road_bound)
                beta_lx1(i)= (y-loc_road_bound(i))/sig_lane1^2;
                alpha_lx1(i)= ((y- loc_road_bound(i))^2/(2* sig_lane1^2));
                apf_lane1(i)= A_lane1* exp(-alpha_lx1(i));
                obst_ul_x1(i) = 0;
%                 obst_ul_y(i) = -beta_lx1(i)*apf_lane1(i);
                obst_ul_y1(i) = -A_lane1*beta_lx1(i)*exp(-alpha_lx1(i));
                obst_ul_xx1(i) = 0;
                obst_ul_xy1(i) = 0;
                obst_ul_yx1(i) = 0;
                obst_ul_yy1(i) = A_lane1*beta_lx1(i)^2*exp(-alpha_lx1(i))-(A_lane1*exp(-alpha_lx1(i)))/sig_lane1^2;
            end
            a1= sum(apf_lane)+sum(apf_lane1);
            a2= sum(obst_ul_x)+ sum(obst_ul_x1);
            a3= sum(obst_ul_y)+ sum(obst_ul_y1);
            a4= sum(obst_ul_xx)+ sum(obst_ul_xx1);
            a5= sum(obst_ul_xy)+ sum(obst_ul_xy1);
            a6= sum(obst_ul_yx)+ sum(obst_ul_yx1);
            a7= sum(obst_ul_yy)+ sum(obst_ul_yy1);
            
%             a1= sum(apf_lane);
%             a2= sum(obst_ul_x);
%             a3= sum(obst_ul_y);
%             a4= sum(obst_ul_xx);
%             a5= sum(obst_ul_xy);
%             a6= sum(obst_ul_yx);
%             a7= sum(obst_ul_yy);
            A1= a1 - a2*x - a3*y + 0.5*a4*x^2 + a5*x*y + 0.5*a7*y^2;

            A2= [ a2 - a4*x - a5*y; 
                  a3 - a5*x - a7*y]';
 
            A3= [a4 a5;
                a6 a7];
        case 'rplusl'
            % Road Potential
            for i = 1:length(loc_road_bound)
%                 apf_road(i)= 0.5* eta * (1/(y- loc_road_bound(i)))^2;
%                 obst_ur_x(i) = 0;
%                 obst_ur_y(i) = -eta/((y- loc_road_bound(i))^3);
%                 obst_ur_xx(i) = 0;
%                 obst_ur_xy(i) = 0;
%                 obst_ur_yx(i) = 0;
%                 obst_ur_yy(i) = (3*eta)/((y-loc_road_bound(i))^4);
                apf_road(i)= 0.5* eta * (1/(y- loc_road_bound(i)))^4;
                obst_ur_x(i) = 0;
                obst_ur_y(i) = -(2*eta)/((y- loc_road_bound(i))^5);
                obst_ur_xx(i) = 0;
                obst_ur_xy(i) = 0;
                obst_ur_yx(i) = 0;
                obst_ur_yy(i) = (10*eta)/((y-loc_road_bound(i))^6);
            end
            % Lane Potential
            for i= 1: length(all_bound)
                beta_lx(i)= (y-all_bound(i))/sig_lane^2;
                alpha_lx(i)= ((y- all_bound(i))^2/(2* sig_lane^2));
                apf_lane(i)= A_lane* exp(-alpha_lx(i));
                obst_ul_x(i) = 0;
                obst_ul_y(i) = -beta_lx(i)*apf_lane(i);
%                 obst_ul_y(i) = -A_lane*beta_lx(i)*exp(-alpha_lx(i));
                obst_ul_xx(i) = 0;
                obst_ul_xy(i) = 0;
                obst_ul_yx(i) = 0;
                obst_ul_yy(i) = A_lane*(beta_lx(i)^2)*exp(-alpha_lx(i))-(A_lane*exp(-alpha_lx(i))/sig_lane^2);
            end
            for i= 1: length(loc_road_bound)
                beta_lx1(i)= (y-loc_road_bound(i))/sig_lane1^2;
                alpha_lx1(i)= ((y- loc_road_bound(i))^2/(2* sig_lane1^2));
                apf_lane1(i)= A_lane1* exp(-alpha_lx1(i));
                obst_ul_x1(i) = 0;
%                 obst_ul_y(i) = -beta_lx(i)*apf_lane(i);
                obst_ul_y1(i) = -A_lane1*beta_lx1(i)*exp(-alpha_lx1(i));
                obst_ul_xx1(i) = 0;
                obst_ul_xy1(i) = 0;
                obst_ul_yx1(i) = 0;
                obst_ul_yy1(i) = A_lane1*beta_lx1(i)^2*exp(-alpha_lx1(i))-(A_lane1*exp(-alpha_lx1(i)))/sig_lane1^2;
            end
            a1= sum(apf_lane)+sum(apf_lane1)+sum(apf_road);
            a2= sum(obst_ul_x)+ sum(obst_ul_x1)+sum(obst_ur_x);
            a3= sum(obst_ul_y)+ sum(obst_ul_y1)+sum(obst_ur_y);
            a4= sum(obst_ul_xx)+ sum(obst_ul_xx1)+sum(obst_ur_xx);
            a5= sum(obst_ul_xy)+ sum(obst_ul_xy1)+sum(obst_ur_xy);
            a6= sum(obst_ul_yx)+ sum(obst_ul_yx1)+sum(obst_ur_yx);
            a7= sum(obst_ul_yy)+ sum(obst_ul_yy1)+sum(obst_ur_yy);
            
%             a1= sum(apf_lane)+sum(apf_road);
%             a2= sum(obst_ul_x)+sum(obst_ur_x);
%             a3= sum(obst_ul_y)+sum(obst_ur_y);
%             a4= sum(obst_ul_xx)+sum(obst_ur_xx);
%             a5= sum(obst_ul_xy)+sum(obst_ur_xy);
%             a6= sum(obst_ul_yx)+sum(obst_ur_yx);
%             a7= sum(obst_ul_yy)+sum(obst_ur_yy);
            A1= a1 - a2*x - a3*y + 0.5*a4*x^2 + a5*x*y + 0.5*a7*y^2;

            A2= [ a2 - a4*x - a5*y; 
                  a3 - a5*x - a7*y]';
 
            A3= [a4 a5;
                a6 a7];
    end
end

% Function for extended matrices of the taylor approximation of the
% Artificial Potential field
function [tilde_C_xy tilde_Ur_0 tilde_Ur_1 tilde_Ur_2 tilde_Uob_0...
    tilde_Uob_1 tilde_Uob_2]= mpc_apf(N_p, C_xy, Ur_0 ,Ur_1 ,Ur_2, Uob_0...
    ,Uob_1 ,Uob_2)

% tilde_Uob_0= Uob_0*N_p;
% 
% tilde_Uob_1= repmat({Uob_1}, 1, N_p);
% tilde_Uob_1= horzcat(tilde_Uob_1{:});
% 
% tilde_Uob_2= repmat({Uob_2}, 1, N_p);
% tilde_Uob_2 = blkdiag(tilde_Uob_2{:});

temp_tilde_Uob_0= horzcat(Uob_0{:});
tilde_Uob_0= sum(temp_tilde_Uob_0);

tilde_Uob_1= horzcat(Uob_1{:});

tilde_Uob_2 = blkdiag(Uob_2{:});

tilde_Ur_0= Ur_0*N_p;

tilde_Ur_1= repmat({Ur_1}, 1, N_p);
tilde_Ur_1= horzcat(tilde_Ur_1{:});

tilde_Ur_2= repmat({Ur_2}, 1, N_p);
tilde_Ur_2 = blkdiag(tilde_Ur_2{:});

tilde_C_xy= repmat({C_xy}, 1, N_p);
tilde_C_xy= blkdiag(tilde_C_xy{:});
end

% Function to get the quadratic cost function
function u_h = cost_func(tilde_A, tilde_B, tilde_C,tilde_A_minus, tilde_B_minus, tilde_C_xy,...
    tilde_Ur_1, tilde_Ur_2, tilde_Uob_1, tilde_Uob_2, tilde_A_u,...
    tilde_lambda_y, tilde_lambda_u, tilde_lambda_delta,tilde_lambda_delta_x, tilde_lambda_r,...
    tilde_lambda_o, N_p, x_h, tilde_y_ref, tilde_u_ref, tilde_A_ineq,...
    tilde_b_ineq, lambda_r, lambda_o, ss_d)

H1 = 2 * (tilde_C * tilde_B)' * tilde_lambda_y * (tilde_C * tilde_B);
H2 = 2 * tilde_lambda_u * eye(width(ss_d.B)*N_p);
H3 = 2 * tilde_A_u' * tilde_lambda_delta * tilde_A_u;
H4 = lambda_r * (tilde_C_xy * tilde_B)' * tilde_Ur_2 * (tilde_C_xy * tilde_B);
H5 = lambda_o * (tilde_C_xy * tilde_B)' * tilde_Uob_2 * (tilde_C_xy * tilde_B);
H6 = 2 * (tilde_B_minus)' * tilde_lambda_delta_x * (tilde_B_minus);

c1 = 2 * ((tilde_C * tilde_A * x_h) - tilde_y_ref)' * tilde_lambda_y * (tilde_C * tilde_B);
c2 = -2 * tilde_u_ref' * tilde_lambda_u;
c3 = lambda_r * (( tilde_Ur_1 + (tilde_C_xy * tilde_A * x_h)' * tilde_Ur_2) * (tilde_C_xy * tilde_B));
c4 = lambda_o * (( tilde_Uob_1 + (tilde_C_xy * tilde_A * x_h)' * tilde_Uob_2) * (tilde_C_xy * tilde_B));
c5 = -2 * (tilde_A_minus*x_h)' * tilde_lambda_delta_x * tilde_B_minus;
% H = H1 + H2 + H3 + H4 + H5;
% c = c1 + c2 + c3 + c4;
H = H1 + H2 + H3 + H4 + H5 + H6;
c = c1 + c2 + c3 + c4 + c5;
[L,p] = chol(H,'lower'); % Checking for positive definiteness of the H matrix.

% AS there are no equality constraints (can add these if there are any
% later. These were written as the solver document asks us to define them.
A_eq= zeros(0,width(ss_d.B)*N_p);
b_eq= zeros(0,1);
u_h_0= zeros(width(ss_d.B)*N_p,1);

iA0 = false(size(tilde_b_ineq)); % same size as the number of inequalities
% opt = mpcActiveSetOptions;
opt = mpcInteriorPointOptions;

% [u_h, exitflag]= mpcActiveSetSolver(H, c', tilde_A_ineq, tilde_b_ineq, A_eq, b_eq,iA0,opt);
[u_h, exitflag]= mpcInteriorPointSolver(H, c', tilde_A_ineq, tilde_b_ineq, A_eq, b_eq,u_h_0,opt);
end

% Function to get the quadratic cost function
function [H, c] = cost_func1(tilde_A, tilde_B, tilde_C,tilde_A_minus, tilde_B_minus, tilde_C_xy,...
    tilde_Ur_1, tilde_Ur_2, tilde_Uob_1, tilde_Uob_2, tilde_A_u,...
    tilde_lambda_y, tilde_lambda_u, tilde_lambda_delta,tilde_lambda_delta_x, tilde_lambda_r,...
    tilde_lambda_o, N_p, x_h, tilde_y_ref, tilde_u_ref, tilde_A_ineq,...
    tilde_b_ineq, lambda_r, lambda_o, ss_d)

H1 = (tilde_C * tilde_B)' * tilde_lambda_y * (tilde_C * tilde_B);
H2 = tilde_lambda_u * eye(width(ss_d.B)*N_p);
H3 = tilde_A_u' * tilde_lambda_delta * tilde_A_u;
H4 = 0.5* lambda_r * (tilde_C_xy * tilde_B)' * tilde_Ur_2 * (tilde_C_xy * tilde_B);
H5 = 0.5* lambda_o * (tilde_C_xy * tilde_B)' * tilde_Uob_2 * (tilde_C_xy * tilde_B);
H6 = (tilde_B_minus)' * tilde_lambda_delta_x * (tilde_B_minus);

c1 = 2 * ((tilde_C * tilde_A * x_h) - tilde_y_ref)' * tilde_lambda_y * (tilde_C * tilde_B);
c2 = -2 * tilde_u_ref' * tilde_lambda_u;
c3 = lambda_r * (( tilde_Ur_1 + (tilde_C_xy * tilde_A * x_h)' * tilde_Ur_2) * (tilde_C_xy * tilde_B));
c4 = lambda_o * (( tilde_Uob_1 + (tilde_C_xy * tilde_A * x_h)' * tilde_Uob_2) * (tilde_C_xy * tilde_B));
c5 = -2 * (tilde_A_minus*x_h)' * tilde_lambda_delta_x * tilde_B_minus;
% H = H1 + H2 + H3 + H4 + H5;
% c = c1 + c2 + c3 + c4;
H = H1 + H2 + H3 + H4 + H5 + H6;
c = c1 + c2 + c3 + c4 + c5;
% [L,p] = chol(H,'lower'); % Checking for positive definiteness of the H matrix.
% 
% % AS there are no equality constraints (can add these if there are any
% % later. These were written as the solver document asks us to define them.
% A_eq= zeros(0,width(ss_d.B)*N_p);
% b_eq= zeros(0,1);
% u_h_0= zeros(width(ss_d.B)*N_p,1);
% 
% iA0 = false(size(tilde_b_ineq)); % same size as the number of inequalities
% % opt = mpcActiveSetOptions;
% opt = mpcInteriorPointOptions;
% 
% % [u_h, exitflag]= mpcActiveSetSolver(H, c', tilde_A_ineq, tilde_b_ineq, A_eq, b_eq,iA0,opt);
% [u_h, exitflag]= mpcInteriorPointSolver(H, c', tilde_A_ineq, tilde_b_ineq, A_eq, b_eq,u_h_0,opt);
end

% Function to get the product of the A matrices based on the inputs 
function alpha= matprodtilde(i,j,A_temp, N_p)
%     ans1= A_temp{1,1};
%     ans2= vertcat(A_temp{:});
%     [r1,c1]= size(ans1);
%     [r2,c2]= size(ans2);
    alpha= A_temp{j+1,1};
    for n= j+2:1:N_p-i+1
        alpha= alpha* A_temp{n,1};
    end
end

%% Function to plot the location of the obstalces

% R1- plots it in terms of rectangles
function polyout= contourofObst(const_o, x_o)
A_o= const_o(1,1);
x_sigma= const_o(1,2);
y_sigma= const_o(1,3);
state_obst= x_o;
sigma_x= sqrt(-(x_sigma^2/(2*log(0.01/A_o))));
sigma_y= sqrt(-(y_sigma^2/(2*log(0.01/A_o))));
[row, col]=size(state_obst);
data= x_o(2:3, :);
ang= x_o(6,:);
fin_data= [data;ang];

points= tester1([x_sigma y_sigma], fin_data);

% here I have to first seperate a rectangle based on the number of
% obstacles and then each rectangle has to have a polyshape and a rotate
% and saved.
    for i= 1:col
        input= points{i};
        x{i}= input(:,1);
        y{i}= input(:,2);
        polyin(i)= polyshape(x{i}', y{i}');
        polyout(i)= rotate(polyin(i), ang(i), data(i,:));
    end
end

% R2
function points= tester1(siz, data)
    l = siz(1,1);
    b= siz(1,2);
    ang1= atand(b/l);
    ang2= 2*ang1;
    ang3= 2*atand(l/b);
    h= sqrt(l^2+b^2)/2;
    for i=1:1:width(data)
        x= data(1,i);
        y= data(2,i);
        ang= data(3,i);
        points{i}= [x+h*cosd(ang-ang1) y+h*sind(ang-ang1);
                 x+h*cosd(ang-ang1+ang2) y+h*sind(ang-ang1+ang2);
                 x+h*cosd(ang-ang1+ang2+ang3) y+h*sind(ang-ang1+ang2+ang3);
                 x+h*cosd(ang-ang1+ang2+ang3+ang2) y+h*sind(ang-ang1+ang2+ang3+ang2)];
    end
end

% E1 - In this section out1 is the output of the ellipses, out2 is the road
% boundaries and lane boundaries and out3 is the lane centers. The
% locations are given by all bound and x_o whereas the lane center
% locations are to be calculated.
function plotdata1(x_h, x_o, const_o, const_r)
    eta= const_r(1,1);
    A_lane= const_r(1,2);
    sig_lane = const_r(1,3);
    A_lane1 = const_r(1,4);
    sig_lane1 = const_r(1,5);
    n_lanes = const_r(1,6);
    alpha= 6 + (n_lanes-1);
    loc_lane_bound = const_r(1,7:alpha);
    loc_road_bound = const_r(1,alpha+1:(alpha+2));
    all_bound= const_r(1, (alpha+n_lanes+1):end);
    for i=1:1:n_lanes
        loc_lane_cent(i)=(all_bound(i)+all_bound(i+1))/2;
    end
    [row_o, col_o]= size(x_o);
    sigma_x= sqrt(-(const_o(1,2)^2/(2*log(0.01/const_o(1,1)))));
    sigma_y= sqrt(-(const_o(1,3)^2/(2*log(0.01/const_o(1,1)))));
    x_c= x_o(2,:);
    y_c= x_o(3,:);
    theta = x_o(6,:);
% Plot location of the obstacles with plots with dimensions equal to
% sigma_x and sigma_y. Vehicles in red.

for i= 1:1:col_o
        a(i)= sigma_x;
        b(i)= sigma_y;
    end
    for i= 1:1:length(x_c)
        c_x= x_c(i);
        c_y= y_c(i);
        alpha = theta(i);
        a_i= a(i);
        b_i= b(i);
        m = 1000;
        x = zeros(m,1);
        y = zeros(m,1);
        th = linspace(0,2*pi,m);
        for k = 1:m
            x(k) = a_i * cos(th(k));
            y(k) = b_i * sin(th(k));
        end
        R  = [  cosd(alpha) -sind(alpha);
                sind(alpha)  cosd(alpha)];
        rCoords = R*[x' ; y']; 
        xr = rCoords(1,:)';      
        yr = rCoords(2,:)'; 
        plot(xr+c_x,yr+c_y,'Color','m', 'DisplayName','Obstacle');
        hold on;
    end
end
function plotdata2(const_r)
    eta= const_r(1,1);
    A_skew= const_r(1,2);
    b_skew= const_r(1,3);
    n_lanes = const_r(1,4);
    all_bound= const_r(1, 5:6+ n_lanes-1);
    loc_lane_bound= all_bound(1, 2:2+n_lanes-2);
    loc_road_bound= [all_bound(1,1) all_bound(1,end)];
    loc_lane_cent= const_r(1, 5+n_lanes+1:end);  
% Plot the lines showing the road and lane boundaries and the lane centers.
% Plot road boundaries in blue, lane boundaries in cyan and lane centers in
% green

    for i=1:1:length(loc_lane_bound)
        yline(loc_lane_bound(i), 'Color','c', 'DisplayName','Lane Boundaries');
        hold on;
    end
    for i=1:1:length(loc_road_bound)
        yline(loc_road_bound(i), 'Color','b', 'DisplayName','Road Boundaries');
        hold on;
    end
    for i=1:1:n_lanes
        yline(loc_lane_cent(i), 'Color','g','LineStyle','--', 'DisplayName','Lane Centres');
        hold on;
    end  
% Add the contour of the gaussian function to the plot

% [U_RplusLplusO]= jaffa(const_r, const_o, x_o, x_h, X_var, Y_var);
% contour(X_var,Y_var,U_RplusLplusO);
% hold off;
end

% E2
function [U_RplusLplusO, U_obst_fin]= jaffa(const_r, const_o, x_o, x_h, X_var, Y_var)
    U_lane_fin= zeros(length(Y_var),length(X_var)); % Matrix defined to store the lane potential values
    U_lane_fin1= zeros(length(Y_var),length(X_var));
    U_road_fin= zeros(length(Y_var),length(X_var)); % Matrix defined to store the road potential values
    U_obst_fin= zeros(length(Y_var),length(X_var));
    A_o= const_o(1,1);
    x_sigma= const_o(1,2);
    y_sigma= const_o(1,3); 
    sigma_x= sqrt(-(x_sigma^2/(2*log(0.01/A_o))));
    sigma_y= sqrt(-(y_sigma^2/(2*log(0.01/A_o))));
    eta= const_r(1,1);
    A_lane= const_r(1,2);
    sig_lane = const_r(1,3);
    A_lane1 = const_r(1,4);
    sig_lane1 = const_r(1,5);
    n_lanes = const_r(1,6);
    alpha= 6 + (n_lanes-1);
    loc_lane_bound = const_r(1,7:alpha);
    loc_road_bound = const_r(1,alpha+1:(alpha+2));
    all_bound= const_r(1, (alpha+n_lanes+1):end);
    [row, col]=size(x_o);
    x= x_h(2,1);
    y= x_h(3,1);
    state_obst= x_o;
%% Lane Potential
for i= 1:n_lanes-1
    for j= 1:length(Y_var)
        for k= 1:length(X_var)
            U_lane(i,j,k)= A_lane* exp(-((Y_var(j)- all_bound(i))^2/(2* sig_lane^2)));
        end
    end
end
for i=1:n_lanes-1
    U_lane_temp1= U_lane(i,:,:);
    U_lane_temp2= squeeze(U_lane_temp1);
    U_lane_fin= U_lane_fin+ U_lane_temp2;
end
for i= 1:length(loc_road_bound)
    for j= 1:length(Y_var)
        for k= 1:length(X_var)
            U_lane1(i,j,k)= A_lane1* exp(-((Y_var(j)- loc_road_bound(i))^2/(2* sig_lane1^2)));
        end
    end
end
for i=1:length(loc_road_bound)
    U_lane_temp11= U_lane1(i,:,:);
    U_lane_temp21= squeeze(U_lane_temp11);
    U_lane_fin1= U_lane_fin1+ U_lane_temp21;
end
U_lane_fin= U_lane_fin+U_lane_fin1;
% surf(X_var,Y_var,U_lane_fin);
% shading interp;

%% Road Potential
for i= 1:length(loc_road_bound)
    for j= 1:length(Y_var)
        for k= 1:length(X_var)
            U_road(i,j,k)= 0.5* eta * (1/(Y_var(j)- loc_road_bound(i)))^2;
        end
    end
end

for i=1:length(loc_road_bound)
    U_road_temp1= U_road(i,:,:);
    U_road_temp2= squeeze(U_road_temp1);
    U_road_fin= U_road_fin+ U_road_temp2;
end

% surf(X_var,Y_var,U_road_fin);
% shading interp;

%% Obstacle Potential

% Rotation Parameters for each obstacle

sigma_x= sqrt(-(x_sigma^2/(2*log(0.01/A_o))));
sigma_y= sqrt(-(y_sigma^2/(2*log(0.01/A_o))));
U_obst_fin= zeros(length(Y_var),length(X_var));
[row, col]=size(state_obst);

    for i= 1:length(Y_var)
        for j=1:length(X_var)
            for k= 1:col
                a_psi(k)= ((cos(state_obst(6,k))^2)/(2*sigma_x^2))  +   ((sin(state_obst(6,k))^2/(2*sigma_y^2)));
                b_psi(k)= -((sin(2*state_obst(6,k)))/(4*sigma_x^2))  + ((sin(2*state_obst(6,k)))/(4*sigma_y^2));
                c_psi(k)= ((sin(state_obst(6,k))^2)/(2*sigma_x^2))  +   ((cos(state_obst(6,k))^2/(2*sigma_y^2))); 
                temp1obs(k)= a_psi(k)*(X_var(j)- state_obst(2,k))^2;
                temp2obs(k)= 2*b_psi(k)*(X_var(j)- state_obst(2,k))* (Y_var(i)- state_obst(3,k));
                temp3obs(k)= c_psi(k)* ((Y_var(i)- state_obst(3,k))^2);
                temp4obs(k)= -(temp1obs(k)+temp2obs(k)+temp3obs(k));
                apf_obst(k,i,j)= A_o*exp(temp4obs(k));
        end
    end
end
for i=1:col
    apf_obst_temp1= apf_obst(i,:,:);
    apf_obst_temp2= squeeze(apf_obst_temp1);
    U_obst_fin= U_obst_fin+ apf_obst_temp2;
end
% subplot(2,1,1)
% surf(X_var,Y_var,U_obst_fin);
% shading interp;
% subplot(2,1,2)
% contour(X_var,Y_var,U_obst_fin)
% hold on;

%% Road + Lane + Obstacle Potential

U_RplusLplusO = U_road_fin+ U_lane_fin + 10* U_obst_fin;
end

%% Functions (Part2)
% Function to find the vertices of the rectangle given the size of the
% rectangle, the angle and then centre. 
function [points]= rect_p(size, data, rl)
    % select1 variable is used to define if the host vehicle is
    % moving to the left or the right. Use select1= 1 for right and select1=2
    % for left
    if rl=='r'
        select= 1;
    elseif rl== 'l'
        select= 2;
    else
        display('Wrong Input')
    end
    l = size(1,1);
    b= size(1,2);
    ang1= atand(b/l);
    ang2= 2*ang1;
    ang3= 2*atand(l/b);
    h= sqrt(l^2+b^2)/2;
    for i=1:1:width(data)
        x= data(2,i);
        y= data(3,i);
        ang= (data(6,i)*180)/pi;
%         ang= data(6,i);
        if select==1
            points{i}= [x+h*cosd(ang-ang1) y+h*sind(ang-ang1);
                        x+h*cosd(ang-ang1+ang2) y+h*sind(ang-ang1+ang2);
                        x+h*cosd(ang-ang1+ang2+ang3) y+h*sind(ang-ang1+ang2+ang3);
                        x+h*cosd(ang-ang1+ang2+ang3+ang2) y+h*sind(ang-ang1+ang2+ang3+ang2)];
        elseif select==2
            points{i}= [x+h*cosd(ang-ang1+ang2+ang3+ang2) y+h*sind(ang-ang1+ang2+ang3+ang2);
                        x+h*cosd(ang-ang1+ang2+ang3) y+h*sind(ang-ang1+ang2+ang3);
                        x+h*cosd(ang-ang1+ang2) y+h*sind(ang-ang1+ang2);
                        x+h*cosd(ang-ang1) y+h*sind(ang-ang1)];
        end
    end
%     points_mat= cell2mat(points);
end

% function to plot a rectangle given its vertices
function rect_plot(vert, tria_v,select)
    if select=='obst' % when plotting an obstacle
        for i=1:1:length(vert)
            v= tria_v{i};
            vertices= vert{i};
            x_vert= vertices(:,1); 
            y_vert= vertices(:,2);
            x_vert_plot{i}= [x_vert(1:3,1);v(1,1);x_vert(4,1);x_vert(1,1)];
            y_vert_plot{i}= [y_vert(1:3,1);v(1,2);y_vert(4,1);y_vert(1,1)];
            plot(x_vert_plot{i}, y_vert_plot{i},'HandleVisibility','off');
            hold on
        end
    elseif select=='host' % when plotting the host vehicle
         for i=1:1:length(vert)
            vertices= vert{i};
            x_vert= vertices(:,1); 
            y_vert= vertices(:,2);
            x_vert_plot{i}= [x_vert;x_vert(1,1)];
            y_vert_plot{i}= [y_vert;y_vert(1,1)];
            plot(x_vert_plot{i}, y_vert_plot{i},'HandleVisibility','off');
            hold on
         end
    end
end

% Function to find the vertices on the rear of the vehicle,P1, P2, the line
% joining the centre of the line P1P2 and the rectangle centre and to find
% a point a distance d from the rear of the vehicle. 
function [tria_v]= trian(bound, distance, x_ov)
    % First find the last two points from bound to give you the rear
    % vertices.

    for i=1:1:width(bound)
        centre= [x_ov(2,i) x_ov(3,i)];
        vertices= bound{i};
        p1= vertices(3,:);
        p2= vertices(4,:);
        pMid= [(p1(1,1)+ p2(1,1))/2 (p1(1,2)+ p2(1,2))/2];
        a= centre-pMid;
        b= a/norm(a);
        v= pMid-distance(i,1)*b;
%         plot([p1(1,1) p2(1,1)], [p1(1,2) p2(1,2)], 'r');
%         hold on;
%         plot(pMid(1,1),pMid(1,2), 'ko');
%         plot([centre(1,1) pMid(1,1)],[centre(1,2) pMid(1,2)],'b')
%         plot(v(1,1),v(1,2), 'c*');
        tria_v{1,i}= v;
    end
        
end

function [v_alp1 v_alp2]= vertex(v, tria_v)
    for k=1:1:width(v)
        v_mat= v{1,k};
        tria= tria_v{1,k};
        v_mat= [v_mat(1:3,:);tria;v_mat(4,:)];
        for i=1:1:height(v_mat)
            j= i+1;
                if i==height(v_mat)
                    j=1;
                end
            v1(i,:)= v_mat(i,:);
            v2(i,:)= v_mat(j,:);
        end
        v_alp1{k}=v1;
        v_alp2{k}=v2;
    end
end

% Function to find the equations of all the 6 lines under question. The
% lines are 3 sides of the rectangle, the 2 sides of the traiangle behind
% and the line parallel to the 4th side(rear) if the rectangle but passing
% through the vertex of the traingle.
function [si_form]= equations(v_alp1, v_alp2, x_o_0)

    for i=1:1:width(v_alp1)
        X= x_o_0(2,i);
        Y= x_o_0(3,i);
        v1= v_alp1{1,i};
        v2= v_alp2{1,i};
        for j=1:1:height(v1)
            p1= v1(j,:);
            p2= v2(j,:);
            m= (p2(1,2)- p1(1,2))/ (p2(1,1)- p1(1,1));
            b= p2(1,2)- m*p2(1,1);
            si_form{j,i}= [m b];
        end
        % Line between P3 and P5
        temp1=v1(3,:);
        temp2=v1(5,:);
        m= (temp2(1,2)- temp1(1,2))/ (temp2(1,1)- temp1(1,1));
        b= temp2(1,2)- m*temp2(1,1);
        si_form{j+1,i}= [m b];
        % Equations of Line perpendicular to L3
        temp2=v1(3,:);
        slope= -1/si_form{3,i}(1,1);
        b = temp2(1,2)- slope*temp2(1,1);
        si_form{j+2,i}= [slope b];
        temp2=v1(4,:);
        b = temp2(1,2)- slope*temp2(1,1);
        si_form{j+3,i}= [slope b];
        % Equations of Line perpendicular to L4
        temp2=v1(4,:);
        slope= -1/si_form{4,i}(1,1);
        b = temp2(1,2)- slope*temp2(1,1);
        si_form{j+4,i}= [slope b];
        temp2=v1(5,:);
        b = temp2(1,2)- slope*temp2(1,1);
        si_form{j+5,i}= [slope b];
%         beta1= [X Y];
%         beta2= v1(4,:);
%         m= (beta2(1,2)- beta1(1,2))/ (beta2(1,1)- beta1(1,1));
%         b= beta2(1,2)- m*beta2(1,1);
%         si_form{j+2,i}= [m b];
%         gam2= v1(4,:);
%         m= (alp2(1,2)- alp1(1,2))/ (alp2(1,1)- alp1(1,1));
%         b= gam2(1,2)- m*gam2(1,1);
%         si_form{j+3,i}= [m b];
    end
end

% Function to find which region a point is from given equations of the 10
% lines, the point. For when phi>0
function flag= region1(si_form, x_h)
    flag=0;
    x= x_h(2,1);
    y= x_h(3,1);
    temp= vertcat(si_form{:});
    m= temp(:,1);
    b= temp(:,2);
    if (y-m(5,1)*x-b(5,1))>0 && (y-m(2,1)*x-b(2,1))<=0 && (y-m(1,1)*x-b(1,1))>0
%         display('Point in R1');
        flag=1;
    elseif (y-m(2,1)*x-b(2,1))>0 && (y-m(1,1)*x-b(1,1))>=0
%         display('Point in R2')
        flag=2;
    elseif (y-m(1,1)*x-b(1,1))<0 && (y-m(6,1)*x-b(6,1))>=0 && (y-m(2,1)*x-b(2,1))>0
%         display('Point in R3')
        flag=3;
    elseif (y-m(6,1)*x-b(6,1))<0 && (y-m(7,1)*x-b(7,1))>=0
%         display('Point in R4')
        flag=4;
    elseif (y-m(8,1)*x-b(8,1))>=0 && (y-m(7,1)*x-b(7,1))<0 && (y-m(3,1)*x-b(3,1))>0
%         display('Point in R5')
        flag=5;
    elseif (y-m(8,1)*x-b(8,1))<0 && (y-m(9,1)*x-b(9,1))>=0
%         display('Point in R6')
        flag=6;
    elseif (y-m(9,1)*x-b(9,1))<0 && (y-m(10,1)*x-b(10,1))>=0 && (y-m(4,1)*x-b(4,1))<0
%         display('Point in R7')
        flag=7;
    elseif (y-m(10,1)*x-b(10,1))<0 && (y-m(6,1)*x-b(6,1))<=0
%         display('Point in R8')
        flag=8;
    elseif (y-m(1,1)*x-b(1,1))<=0 && (y-m(6,1)*x-b(6,1))>0 && (y-m(5,1)*x-b(5,1))<0
%         display('Point in R9')
        flag=9;
    elseif (y-m(1,1)*x-b(1,1))>0 && (y-m(5,1)*x-b(5,1))<=0
%         display('Point in R10')
        flag=10;
    else
%         display('Error')
        flag=11;
    end
end

% Function to find which region a point is from given equations of the 10
% lines, the point. For when phi=0
function flag= region2(si_form, x_h, v1)
    flag=0;
    x= x_h(2,1);
    y= x_h(3,1);
    temp= vertcat(si_form{:});
    m= temp(:,1);
    b= temp(:,2);
    if (y-m(5,1)*x-b(5,1))>0 && (y-m(2,1)*x-b(2,1))<=0 && (x-v1(1,1))>0
%         display('Point in R1')
        flag=1;
    elseif (y-m(2,1)*x-b(2,1))>0 && (x-v1(1,1))>=0
%         display('Point in R2')
        flag=2;
    elseif (x-v1(1,1))<0 && (x-v1(3,1))>=0 && (y-m(2,1)*x-b(2,1))>0
%         display('Point in R3')
        flag=3;
    elseif (x-v1(3,1))<0 && (y-m(7,1)*x-b(7,1))>=0
%         display('Point in R4')
        flag=4;
    elseif (y-m(8,1)*x-b(8,1))>=0 && (y-m(7,1)*x-b(7,1))<0 && (y-m(3,1)*x-b(3,1))>0
%         display('Point in R5')
        flag=5;
    elseif (y-m(8,1)*x-b(8,1))<0 && (y-m(9,1)*x-b(9,1))>=0
%         display('Point in R6')
        flag=6;
    elseif (y-m(9,1)*x-b(9,1))<0 && (y-m(10,1)*x-b(10,1))>=0 && (y-m(4,1)*x-b(4,1))<0
%         display('Point in R7')
        flag=7;
    elseif (y-m(10,1)*x-b(10,1))<0 && (x-v1(3,1))<=0
%         display('Point in R8')
        flag=8;
    elseif (x-v1(1,1))<=0 && (x-v1(3,1))>0 && (y-m(5,1)*x-b(5,1))<0
%         display('Point in R9')
        flag=9;
    elseif (x-v1(1,1))>0 && (y-m(5,1)*x-b(5,1))<=0
%         display('Point in R10')
        flag=10;
    else
%         display('Error')
        flag=11;
    end
end

% Function to find which region a point is from given equations of the 10
% lines, the point. For when phi<0
function flag= region3(si_form, x_h)
    flag=0;
    x= x_h(2,1);
    y= x_h(3,1);
    temp= vertcat(si_form{:});
    m= temp(:,1);
    b= temp(:,2);
    if (y-m(5,1)*x-b(5,1))>0 && (y-m(2,1)*x-b(2,1))<=0 && (y-m(1,1)*x-b(1,1))<0
%         display('Point in R1')
        flag=1;
    elseif (y-m(2,1)*x-b(2,1))>0 && (y-m(1,1)*x-b(1,1))<=0
%         display('Point in R2')
        flag=2;
    elseif (y-m(1,1)*x-b(1,1))>0 && (y-m(6,1)*x-b(6,1))<=0 && (y-m(2,1)*x-b(2,1))>0
%         display('Point in R3')
        flag=3;
    elseif (y-m(6,1)*x-b(6,1))>0 && (y-m(7,1)*x-b(7,1))>=0
%         display('Point in R4')
        flag=4;
    elseif (y-m(8,1)*x-b(8,1))>=0 && (y-m(7,1)*x-b(7,1))<0 && (y-m(3,1)*x-b(3,1))>0
%         display('Point in R5')
        flag=5;
    elseif (y-m(8,1)*x-b(8,1))<0 && (y-m(9,1)*x-b(9,1))>=0
%         display('Point in R6')
        flag=6;
    elseif (y-m(9,1)*x-b(9,1))<0 && (y-m(10,1)*x-b(10,1))>=0 && (y-m(4,1)*x-b(4,1))<0
%         display('Point in R7')
        flag=7;
    elseif (y-m(10,1)*x-b(10,1))<0 && (y-m(6,1)*x-b(6,1))>=0
%         display('Point in R8')
        flag=8;
    elseif (y-m(1,1)*x-b(1,1))>=0 && (y-m(6,1)*x-b(6,1))<0 && (y-m(5,1)*x-b(5,1))<0
%         display('Point in R9')
        flag=9;
    elseif (y-m(1,1)*x-b(1,1))<0 && (y-m(5,1)*x-b(5,1))<=0
%         display('Point in R10')
        flag=10;
    else
%         display('Error')
        flag=11;
    end
end

% Selection of the correct form of the value of K for the Yukawa Potential
% depending on which section it is in. 
function [K, u, ux, uy, uxx, uyy, uyx, uxy] = dist(flag, v, x_h_0, const)
    x= x_h_0(2,1);
    y= x_h_0(3,1);
    p= [x y];
    %%
    switch flag
        case 1
            p1= v(1,:); % First vertex of the obstacle (bottom right) as we are dealing with Region 1.
            p2= v(2,:); % Second vertex of the obstacle (top right) as we are dealing with Region 1.
            % The next four variables are used to seperate the vertices (1
            % and 2) of the obstacle into x and y positions
            x1= p1(1,1); 
            y1= p1(1,2);
            x2= p2(1,1);
            y2= p2(1,2);
            % Find the value of the distance from the location of the host
            % vehicle to the nearest point. As region 1 uses the distance
            % to the line segment formula, we use this formulation
            % gamma= ((x-x1) * (x2-x1)+ (y-y1) * (y2-y1))/((y2-y1)^2 + (x2-x1)^2)
            % K = sqrt((x_h- (x1+gamma(x2-x1)))^2+(y_h- (y1+gamma(x2-x1)))^2)
            alpha= (x-x1) * (x2-x1)+ (y-y1) * (y2-y1);
            beta= (y2-y1)^2 + (x2-x1)^2;
            gamma= alpha/beta;               
            xx= x1 + (gamma*(x2-x1));
            yy= y1 + (gamma*((y2-y1)));
            K = sqrt((x-xx)^2+ (y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv1(p1, p2, p, const);
        case 2
            p1= v(2,:); % As we are in region 6, we take the fourth vertex
            % of the vertices of the extended obstacle
            x0= p1(1,1); % x position of the fourth vertex
            y0= p1(1,2); % y position of the fourth vertex
            m= -(x-x0)/(y-y0); % Slope of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            b= y0-m*x0; % y-intercept of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            if isinf(m)==0
                if m==0
                    A= 0;
                    B= 1;
                    C= -y0;
                else
                    A= -m;
                    B= 1;
                    C= -b;
                end
            elseif isinf(m)==1
                A= 1;
                B= 0;
                C= -x0;
            end
            xx= ((B*(B*x-A*y)-A*C)/(A^2+B^2));
            yy= ((A*(-B*x+A*y)-B*C)/(A^2+B^2));
            K= sqrt((x-xx)^2+(y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv2(p,p1,const, m, b);
        case 3
            p1= v(2,:); % Second vertex of the obstacle (bottom right) as we are dealing with Region 1.
            p2= v(3,:); % Third vertex of the obstacle (top right) as we are dealing with Region 1.
            % The next four variables are used to seperate the vertices (1
            % and 2) of the obstacle into x and y positions
            x1= p1(1,1); 
            y1= p1(1,2);
            x2= p2(1,1);
            y2= p2(1,2);
            % Find the value of the distance from the location of the host
            % vehicle to the nearest point. As region 1 uses the distance
            % to the line segment formula, we use this formulation
            % gamma= ((x-x1) * (x2-x1)+ (y-y1) * (y2-y1))/((y2-y1)^2 + (x2-x1)^2)
            % K = sqrt((x_h- (x1+gamma(x2-x1)))^2+(y_h- (y1+gamma(x2-x1)))^2)
            alpha= (x-x1) * (x2-x1)+ (y-y1) * (y2-y1);
            beta= (y2-y1)^2 + (x2-x1)^2;
            gamma= alpha/beta;               
            xx= x1 + (gamma*(x2-x1));
            yy= y1 + (gamma*((y2-y1)));
            K = sqrt((x-xx)^2+ (y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv1(p1, p2, p, const);
        case 4
            p1= v(3,:); % As we are in region 6, we take the fourth vertex
            % of the vertices of the extended obstacle
            x0= p1(1,1); % x position of the fourth vertex
            y0= p1(1,2); % y position of the fourth vertex
            m= -(x-x0)/(y-y0); % Slope of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            b= y0-m*x0; % y-intercept of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            if isinf(m)==0
                if m==0
                    A= 0;
                    B= 1;
                    C= -y0;
                else
                    A= -m;
                    B= 1;
                    C= -b;
                end
            elseif isinf(m)==1
                A= 1;
                B= 0;
                C= -x0;
            end
            xx= ((B*(B*x-A*y)-A*C)/(A^2+B^2));
            yy= ((A*(-B*x+A*y)-B*C)/(A^2+B^2));
            K= sqrt((x-xx)^2+(y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv2(p,p1,const, m, b);
        case 5
            p1= v(3,:); % Third vertex of the obstacle (bottom right) as we are dealing with Region 1.
            p2= v(4,:); % Fourth vertex of the obstacle (top right) as we are dealing with Region 1.
            % The next four variables are used to seperate the vertices (1
            % and 2) of the obstacle into x and y positions
            x1= p1(1,1); 
            y1= p1(1,2);
            x2= p2(1,1);
            y2= p2(1,2);
            % Find the value of the distance from the location of the host
            % vehicle to the nearest point. As region 1 uses the distance
            % to the line segment formula, we use this formulation
            % gamma= ((x-x1) * (x2-x1)+ (y-y1) * (y2-y1))/((y2-y1)^2 + (x2-x1)^2)
            % K = sqrt((x_h- (x1+gamma(x2-x1)))^2+(y_h- (y1+gamma(x2-x1)))^2)
            alpha= (x-x1) * (x2-x1)+ (y-y1) * (y2-y1);
            beta= (y2-y1)^2 + (x2-x1)^2;
            gamma= alpha/beta;               
            xx= x1 + (gamma*(x2-x1));
            yy= y1 + (gamma*((y2-y1)));
            K = sqrt((x-xx)^2+ (y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv1(p1, p2, p, const);
        case 6
            p1= v(4,:); % As we are in region 6, we take the fourth vertex
            % of the vertices of the extended obstacle
            x0= p1(1,1); % x position of the fourth vertex
            y0= p1(1,2); % y position of the fourth vertex
            m= -(x-x0)/(y-y0); % Slope of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            b= y0-m*x0; % y-intercept of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            if isinf(m)==0
                if m==0
                    A= 0;
                    B= 1;
                    C= -y0;
                else
                    A= -m;
                    B= 1;
                    C= -b;
                end
            elseif isinf(m)==1
                A= 1;
                B= 0;
                C= -x0;
            end
            xx= ((B*(B*x-A*y)-A*C)/(A^2+B^2));
            yy= ((A*(-B*x+A*y)-B*C)/(A^2+B^2));
            K= sqrt((x-xx)^2+(y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv2(p,p1,const, m, b);
        case 7
            p1= v(4,:); % fourth vertex of the obstacle (bottom right) as we are dealing with Region 1.
            p2= v(5,:); % fifth vertex of the obstacle (top right) as we are dealing with Region 1.
            % The next four variables are used to seperate the vertices (1
            % and 2) of the obstacle into x and y positions
            x1= p1(1,1); 
            y1= p1(1,2);
            x2= p2(1,1);
            y2= p2(1,2);
            % Find the value of the distance from the location of the host
            % vehicle to the nearest point. As region 1 uses the distance
            % to the line segment formula, we use this formulation
            % gamma= ((x-x1) * (x2-x1)+ (y-y1) * (y2-y1))/((y2-y1)^2 + (x2-x1)^2)
            % K = sqrt((x_h- (x1+gamma(x2-x1)))^2+(y_h- (y1+gamma(x2-x1)))^2)
            alpha= (x-x1) * (x2-x1)+ (y-y1) * (y2-y1);
            beta= (y2-y1)^2 + (x2-x1)^2;
            gamma= alpha/beta;               
            xx= x1 + (gamma*(x2-x1));
            yy= y1 + (gamma*((y2-y1)));
            K = sqrt((x-xx)^2+ (y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv1(p1, p2, p, const);
        case 8
            p1= v(5,:); % As we are in region 6, we take the fourth vertex
            % of the vertices of the extended obstacle
            x0= p1(1,1); % x position of the fourth vertex
            y0= p1(1,2); % y position of the fourth vertex
            m= -(x-x0)/(y-y0); % Slope of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            b= y0-m*x0; % y-intercept of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            if isinf(m)==0
                if m==0
                    A= 0;
                    B= 1;
                    C= -y0;
                else
                    A= -m;
                    B= 1;
                    C= -b;
                end
            elseif isinf(m)==1
                A= 1;
                B= 0;
                C= -x0;
            end
            xx= ((B*(B*x-A*y)-A*C)/(A^2+B^2));
            yy= ((A*(-B*x+A*y)-B*C)/(A^2+B^2));
            K= sqrt((x-xx)^2+(y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv2(p,p1,const, m, b);
        case 9 
            p1= v(5,:); % fifth vertex of the obstacle (bottom right) as we are dealing with Region 1.
            p2= v(1,:); % first vertex of the obstacle (top right) as we are dealing with Region 1.
            % The next four variables are used to seperate the vertices (1
            % and 2) of the obstacle into x and y positions
            x1= p1(1,1); 
            y1= p1(1,2);
            x2= p2(1,1);
            y2= p2(1,2);
            % Find the value of the distance from the location of the host
            % vehicle to the nearest point. As region 1 uses the distance
            % to the line segment formula, we use this formulation
            % gamma= ((x-x1) * (x2-x1)+ (y-y1) * (y2-y1))/((y2-y1)^2 + (x2-x1)^2)
            % K = sqrt((x_h- (x1+gamma(x2-x1)))^2+(y_h- (y1+gamma(x2-x1)))^2)
            alpha= (x-x1) * (x2-x1)+ (y-y1) * (y2-y1);
            beta= (y2-y1)^2 + (x2-x1)^2;
            gamma= alpha/beta;               
            xx= x1 + (gamma*(x2-x1));
            yy= y1 + (gamma*((y2-y1)));
            K = sqrt((x-xx)^2+ (y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv1(p1, p2, p, const);
        case 10
            p1= v(1,:); % As we are in region 6, we take the fourth vertex
            % of the vertices of the extended obstacle
            x0= p1(1,1); % x position of the fourth vertex
            y0= p1(1,2); % y position of the fourth vertex
            m= -(x-x0)/(y-y0); % Slope of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            b= y0-m*x0; % y-intercept of the line perpendicular to the
            % line joining the fourth vertex of the obstacle and the 
            % centre of the host vehicle
            if isinf(m)==0
                if m==0
                    A= 0;
                    B= 1;
                    C= -y0;
                else
                    A= -m;
                    B= 1;
                    C= -b;
                end
            elseif isinf(m)==1
                A= 1;
                B= 0;
                C= -x0;
            end
            xx= ((B*(B*x-A*y)-A*C)/(A^2+B^2));
            yy= ((A*(-B*x+A*y)-B*C)/(A^2+B^2));
            K= sqrt((x-xx)^2+(y-yy)^2);
            [u, ux, uy, uxx, uyy, uyx, uxy]= derv2(p,p1,const, m, b);
        case 11
            display('Point Inside the Car');
            K= 2e-10;
            u=-const(1,1)*log(const(1,2)*K)/(const(1,2)*K);
            ux=0;
            uy=0;
            uxx=0;
            uyy=0;
            uyx=0;
            uxy=0;
    end

end

% Function to find the value of the obtacle potential and its deravatives
% if the point lies in R1, R3, R5, R7, R9. The inputs are the edges of the
% line and then point at which the vehicle lies. Also as input are the
% constants of the potential field

function [u, ux, uy, uxx, uyy, uyx, uxy]= derv1(p1, p2, p, const)
%%
    A0= const(1,1);
    b0= const(1,2);
    x= p(1,1);
    y= p(1,2);
    x1= p1(1,1);
    y1= p1(1,2);
    x2= p2(1,1);
    y2= p2(1,2);
%%  
    a1= x2-x1;
    a2= y2-y1;
    xx= x1+ ((a1^2*(x-x1)+a1*a2*(y-y1))/(a1^2+a2^2));
    yy= y1+ ((a1*a2*(x-x1)+a2^2*(y-y1))/(a1^2+a2^2));
    K= sqrt((y-yy)^2+(x-xx)^2);
    
    alpha_1= a2* x- a1*y + a1*y1- a2*x1;
    alpha_2= a1* y- a1*y1- a2*x+ a2* x1;
    alpha_3= a2^2*x^2+ (-2*a1*a2*y + 2*a1*a2*y1 - 2*a2^2*x1) * x...
        + a1^2*y^2 + (2*a1*a2*x1 - 2*a1^2*y1) * y + a1^2*y1^2 - 2*a1*a2*x1*y1 + a2^2*x1^2;
    alpha_4= a1^2*y^2+ (-2*a1^2*y1 - 2*a1*a2*x + 2*a1*a2*x1) * y ...
        + a1^2*y1^2 + (2*a1*a2*x - 2*a1*a2*x1) * y1+ a2^2*x^2 - 2*a2^2*x1*x + a2^2*x1^2;
    alpha_5= a1^2+a2^2;
%%
    u= -(A0*log(b0*K))/(b0*K);
    
    ux= (A0*a2*alpha_1*(log(b0*K)-1))/(alpha_5*b0*K^3);
    
    uy= (A0*a1*alpha_2*(log(b0*K)-1))/(alpha_5*b0*K^3);
    
    uxx= -(A0*a2^2*alpha_3*(2*log(b0*K)-3))/(alpha_5^2*b0*K^5);
    
    uyy= -(A0*a1^2*alpha_4*(2*log(b0*K)-3))/(alpha_5^2*b0*K^5);
    
    uxy= (A0*a1*a2*alpha_3*(2*log(b0*K)-3))/(alpha_5^2*b0*K^5);
    
    uyx= (A0*a1*a2*alpha_4*(2*log(b0*K)-3))/(alpha_5^2*b0*K^5);
end

% Function to find the value of the obstacle potential and its deravatives
% if the point lies in R2, R4, R6, R8, R10. The inputs the closest point on 
% the obstacle and point at which the vehicle lies. Also as input are the
% constants of the potential field

function [u, ux, uy, uxx, uyy, uyx, uxy]= derv2(p, p1, const, m, b)
    A0= const(1,1);
    b0= const(1,2);
    x= p(1,1);
    y= p(1,2);
    x_0= p1(1,1);
    y_0= p1(1,2);
    if isinf(m)==0
        if m==0
            A= 0;
            B= 1;
            C= -y_0;
        else
            A= -m;
            B= 1;
            C= -b;
        end
    elseif isinf(m)==1
        A= 1;
        B= 0;
        C= -x_0;
    end
    xx= ((B*(B*x-A*y)-A*C)/(A^2+B^2));
    yy= ((A*(-B*x+A*y)-B*C)/(A^2+B^2));
    K= sqrt((x-xx)^2+(y-yy)^2);
    
    alpha_1= A*x+B*y+C;
    alpha_2= A*x+B*y+C;
    alpha_3= A^2*x^2 + (2*A*B*y + 2* A * C)*x + B^2*y^2 + 2* B* C * y + C^2;
    alpha_4= B^2*y^2 + (2*A*B*x + 2* B * C)*y + A^2*x^2 + 2* A* C * x + C^2;
    alpha_5= A^2+B^2;
    
    u= -(A0*log(b0*K))/(b0*K);
    
    ux= (A*A0*alpha_1*(log(b0*K)-1))/(alpha_5*b0*K^3);
    
    uy= (B*A0*alpha_2*(log(b0*K)-1))/(alpha_5*b0*K^3);
    
    uxx= -(A^2*A0*alpha_3*(2*log(b0*K)-3))/(alpha_5^2*b0*K^5);
    
    uyy= -(B^2*A0*alpha_4*(2*log(b0*K)-3))/(alpha_5^2*b0*K^5);
    
    uxy= -(A*B*A0*alpha_3*(2*log(b0*K)-3))/(alpha_5^2*b0*K^5);
    
    uyx= -(A*B*A0*alpha_4*(2*log(b0*K)-3))/(alpha_5^2*b0*K^5);
end
% Draw a surface plot to show the actual obstacle and division into
% regions
function surfplot_rect(x_h_0, x_o_0, distance, size_veh, rl, alpha)
[bound_OV]= rect_p(size_veh, x_o_0, rl);
[tria_v]= trian(bound_OV, distance, x_o_0);

[v_alp1 v_alp2]= vertex(bound_OV, tria_v);
[si_form]= equations(v_alp1, v_alp2, x_o_0);
    for i=1:1:length(v_alp1)
        vertices1= v_alp1{i}(:,1);
        vertices2= v_alp1{i}(:,2);
        x_vert_plot= [vertices1(:);vertices1(1,1)]; 
        y_vert_plot= [vertices2(:);vertices2(1,1)];
        name= ['Obstacle' num2str(i)];
        z= ones(size(x_vert_plot))*alpha;
        hold on
        view(3);
        plot3(x_vert_plot,y_vert_plot,z ,'DisplayName', name, 'Color', 'k');        
        si= vertcat(si_form{:,i});
        m= si(:,1);
        b= si(:,2);
        if x_o_0(6,i)>0
            plot3([vertices1(2,1) vertices1(2,1)-0.04], [vertices2(2,1) m(1,1)*(vertices1(2,1)-0.04)+b(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L1'); %L1
            plot3([vertices1(1,1) vertices1(1,1)+0.04], [vertices2(1,1) m(1,1)*(vertices1(1,1)+0.04)+b(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L1*'); %L1'
            plot3([vertices1(2,1) vertices1(2,1)+0.2], [vertices2(2,1) m(2,1)*(vertices1(2,1)+0.2)+b(2,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L2'); %L2
            plot3([vertices1(1,1) vertices1(1,1)+0.2], [vertices2(1,1) m(5,1)*(vertices1(1,1)+0.2)+b(5,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L5'); %L5
            plot3([vertices1(3,1) vertices1(3,1)-0.04], [vertices2(3,1) m(6,1)*(vertices1(3,1)-0.04)+b(6,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L6'); %L6
            plot3([vertices1(5,1) vertices1(5,1)+0.04], [vertices2(5,1) m(6,1)*(vertices1(5,1)+0.04)+b(6,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L6*'); %L6'
            plot3([vertices1(3,1) vertices1(3,1)-0.2], [vertices2(3,1) m(7,1)*(vertices1(3,1)-0.1)+b(7,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L7'); %L7
            plot3([vertices1(4,1) vertices1(4,1)-0.2], [vertices2(4,1) m(8,1)*(vertices1(4,1)-0.1)+b(8,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L8'); %L8
            plot3([vertices1(4,1) vertices1(4,1)-0.1], [vertices2(4,1) m(9,1)*(vertices1(4,1)-0.1)+b(9,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L9'); %L9
            plot3([vertices1(5,1) vertices1(5,1)-0.1], [vertices2(5,1) m(10,1)*(vertices1(5,1)-0.1)+b(10,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L10'); %L10
        elseif x_o_0(6,i)==0
            plot3([vertices1(2,1) vertices1(2,1)], [vertices2(2,1) vertices2(2,1)+0.5], [alpha alpha], 'Color', 'r','DisplayName', 'L1'); %L1
            plot3([vertices1(1,1) vertices1(1,1)], [vertices2(1,1) vertices2(1,1)-0.5], [alpha alpha], 'Color', 'r','DisplayName', 'L1*'); %L1'
            plot3([vertices1(2,1) vertices1(2,1)+1], [vertices2(2,1) vertices2(2,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L2'); %L2
            plot3([vertices1(1,1) vertices1(1,1)+1], [vertices2(1,1) vertices2(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L5'); %L5
            plot3([vertices1(3,1) vertices1(3,1)], [vertices2(3,1) vertices2(3,1)+0.5], [alpha alpha], 'Color', 'r','DisplayName', 'L6'); %L6
            plot3([vertices1(5,1) vertices1(5,1)], [vertices2(5,1) vertices2(5,1)-0.5], [alpha alpha], 'Color', 'r','DisplayName', 'L6*'); %L6'
            plot3([vertices1(3,1) vertices1(3,1)-1], [vertices2(3,1) m(7,1)*(vertices1(3,1)-1)+b(7,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L7'); %L7
            plot3([vertices1(4,1) vertices1(4,1)-1], [vertices2(4,1) m(8,1)*(vertices1(4,1)-1)+b(8,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L8'); %L8
            plot3([vertices1(4,1) vertices1(4,1)-1], [vertices2(4,1) m(9,1)*(vertices1(4,1)-1)+b(9,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L9'); %L9
            plot3([vertices1(5,1) vertices1(5,1)-1], [vertices2(5,1) m(10,1)*(vertices1(5,1)-1)+b(10,1)], [alpha alpha], 'Color','r','DisplayName', 'L10'); %L10
        elseif x_o_0(6,i)<0
            plot3([vertices1(2,1) vertices1(2,1)+0.01], [vertices2(2,1) m(1,1)*(vertices1(2,1)+0.01)+b(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L1'); %L1
            plot3([vertices1(1,1) vertices1(1,1)-0.01], [vertices2(1,1) m(1,1)*(vertices1(1,1)-0.01)+b(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L1*'); %L1'
            plot3([vertices1(2,1) vertices1(2,1)+0.1], [vertices2(2,1) m(2,1)*(vertices1(2,1)+0.1)+b(2,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L2'); %L2
            plot3([vertices1(1,1) vertices1(1,1)+0.1], [vertices2(1,1) m(5,1)*(vertices1(1,1)+0.1)+b(5,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L5'); %L5
            plot3([vertices1(3,1) vertices1(3,1)+0.01], [vertices2(3,1) m(6,1)*(vertices1(3,1)+0.01)+b(6,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L6'); %L6
            plot3([vertices1(5,1) vertices1(5,1)-0.01], [vertices2(5,1) m(6,1)*(vertices1(5,1)-0.01)+b(6,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L6*'); %L6'
            plot3([vertices1(3,1) vertices1(3,1)-0.1], [vertices2(3,1) m(7,1)*(vertices1(3,1)-0.1)+b(7,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L7'); %L7
            plot3([vertices1(4,1) vertices1(4,1)-0.1], [vertices2(4,1) m(8,1)*(vertices1(4,1)-0.1)+b(8,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L8'); %L8
            plot3([vertices1(4,1) vertices1(4,1)-0.1], [vertices2(4,1) m(9,1)*(vertices1(4,1)-0.1)+b(9,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L9'); %L9
            plot3([vertices1(5,1) vertices1(5,1)-0.1], [vertices2(5,1) m(10,1)*(vertices1(5,1)-0.1)+b(10,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L10'); %L10
        end
    end
%     plot3(x_h_0(2,1),x_h_0(3,1),alpha,'Color','r', 'Marker', '*', 'DisplayName', 'Host Vehicle');
%     hold off;
%     legend;
end

function [A1 A2 A3]= forcontour(const_r, const_o, x_o, x_h, select, rl, distance)
    A= const_o(1,1);
    b= const_o(1,2);
    [row, col]=size(x_o);
    x= x_h(2,1);
    y= x_h(3,1);
    size_veh= [2.5 1.5]; % Size of the vehicle
    [bound_OV]= rect_p(size_veh, x_o, rl);
    % Given the vertices of the obstacle, we can find then distance from a
    % point to all the vertices and all the boundaries, and find the minimum of
    % these values. 
    [bound_OV]= rect_p(size_veh, x_o, rl);
    [tria_v]= trian(bound_OV, distance, x_o);
    [v_alp1 v_alp2]= vertex(bound_OV, tria_v);
    [si_form]= equations(v_alp1, v_alp2, x_o);
%% Obstacle Potential             
            % write the derivative for the obstacle potential for x, y, xx,
            % xy, yx, and yy. Substitute the actual values and then get the
            % values of A1, A2, and A3
            for i= 1:1:width(x_o)
                if x_o(6,1)>0
                    flag(i)= region1(si_form(:,i), x_h);
                elseif x_o(6,1)==0
                    flag(i)= region2(si_form(:,i), x_h, v_alp1{1,i});
                elseif x_o(6,1)<0
                    flag(i)= region3(si_form(:,i), x_h);
                end
                [K(i), apf_obst(i), obst_u_x(i), obst_u_y(i), obst_u_xx(i), obst_u_yy(i), obst_u_yx(i), obst_u_xy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
            end
            %%
            a1= sum(apf_obst);

end
