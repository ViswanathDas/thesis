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

x_h_0= [41.6 0 1.5 0 0 0]'; % state of the host vehicle. 

% U=[F_x delta]'
u_h_0= [0 0]';

%% Road Data

% decided that the global origin moved along the boundary of the right lane
% along with the HV (host vehicle).

n_lanes= 2; % number of lanes
size_lane = 3; % width of lane in meters
road_len= 1500;  % length of the road in meters

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
eta= 100;
A_skew = 2; 
b_skew = 1e3;

const_r= [eta A_skew b_skew n_lanes all_bound loc_lane_cent];

%% Obstacle Data

% States of the Obstacle
% x_o_0= [41.6 100 4.5 0 0 0]'; %vel drop 1
% x_o_0= [1 250 4.5 0 0 0;20 300 1.5 0 0 0]'; %DLC
% x_o_0= [1 230 4.5 0 0 0;1 230 1.5 0 0 0]'; %wait behind
% x_o_0= [20 230 4.5 0 0 0;1 500 1.5 0 0 0]';  %Triple LC 
% x_o_0= [0 500 4.5 0 0 0;10 500 1.5 0 0 0]';  %Triple LC
x_o_0= [20 300 1.5 0 0 0;10 700 4.5 0 0 0]';  %Triple LC
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
lambda_y_APFMPC= [  5 0 0 0 0;
                    0 15 0 0 0;
                    0 0 1 0 0;
                    0 0 0 1 0; 
                    0 0 0 0 1000;];

% for u= [F_x delta]^T
lambda_u_APFMPC= [  0       0;
                    0        1000;];
                
lambda_delta_u_APFMPC= [    0   0;
                            0   100;];

lambda_delta_x_APFMPC= [    1 0 0 0 0 0;
                            0 1 0 0 0 0;
                            0 0 1 0 0 0;
                            0 0 0 1 0 0;
                            0 0 0 0 1 0;
                            0 0 0 0 0 1;];
lambda_o_APFMPC= 0.25;
lambda_r_APFMPC= 0.1;

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
            if flag_ND==1
                distance= [];
            elseif flag_ND==0
                for i= 1:1:width(x_o)
                    distance(i,1)= (x_h(1,1)^2/(2*a_x_max)) - (x_o(1,i)^2/(2*a_x_max)) +x_h(1,1)*t1 + d0;
                    if distance(i,1)<5
                        distance(i,1)=5;
                    end
                end
            end
            flag_LC= set_flag_LC(x_h, x_o, lane_host, lane_obst, d_safe_max, size_lane, a_x_max, distance, x_h_0, size_veh);
        end
        
    %% Host Vehicle Model
    
    % Model of the host vehicle for APF-MPC
    ss_d_APFMPC= car_modelAPFMPC(car_const, T, x_h, u_h_0);
    
    %% Obstacle Model

    ss_d_obs_APFMPC= car_model_obs(T, 'APF-MPC');
    
    %% Constraints Boundaries

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
%     % Variable for the initial lane of the host vehicle
%     u_init= sdpvar(height(u_h_0),1); % 
    % Assigning the initial variables defined to the respective variables
    % on the extended variable set.
    x{1,1}= x_init;
%     u{1,1}= u_init;
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
                x_o1, x_h, 'obst', rl, distance, size_veh);
        end
        [U_0_r, U_1_r, U_2_r]= taylor2ndOrder(const_r, const_o,...
            x_o, x_h, 'road', rl, distance, size_veh);
        
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
%             constraints= [constraints, -0.2<=((u{1,i}(1,1)-u_h(1,1)))<=0.2];
%         elseif i>1
%             constraints= [constraints, -0.2<=((u{1,i}(1,1)-u{1,i-1}(1,1)))<=0.2];
%         end
%         if i==1
%             constraints= [constraints, -0.05<=(u{1,i}(2,1)-u_h(2,1))<=0.05];
%         elseif i>1
%             constraints= [constraints, -0.05<=(u{1,i}(2,1)-u{1,i-1}(2,1))<=0.05];
%         end
    end
    
    %% Optimization
    ops= sdpsettings('solver','gurobi','verbose', 2, 'gurobi.ScaleFlag', 0, 'gurobi.BarHomogeneous', 1);
    optimize([constraints, x_init== x_h],objective, ops);
%     optimize(constraints,objective);
    u_h= value(u{1});
    x_h= value(x{2});
    
    display(x_h')
    display(u_h')
    % Saving the values of the states and the input

    count= count + 1;
%     if count>140
%         x_o(1,:)=[20 1];
%     end
%     if count>250
%         x_o(1,:)=[1 20];
%     end
%     if count>250
%         x_o(3,:)=[1.5 4.5];
%     end
    if flag_ND==0
%         if count<=200
%             x_o= ss_d_obs_APFMPC.A* x_o;
%         else
%             if x_o(1,1)>0
%             testB= [T 0 0 0 0 0; 0 0 0 0 0 0]';
%             x_o= ss_d_obs_APFMPC.A* x_o + testB*[-a_x_max 0; 0 0];
%             end
%             if x_o(1,1)<=0
%                 x_o(1,1)=0;
%                 x_o= ss_d_obs_APFMPC.A* x_o;
%             end
%         end
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
    % First plot the given data. This data includes the road and lane markers,
    % the host vehicle (with no tail), the obstacle vehicles with tail, as 
    % the paths taken by the host and the obstacle vehicles. In the second
    % plot, plot the host and the obstacle vehicle velocities, third plot
    % includes the distances between the vehicles, the fourth plot have the
    % accelation as well as the chance in optimal lane number. Asssumption that
    % at the initial step the current value is the optimal state of the host
    % vehicle. 
    
    if count>=362
        display(count);
    end
    % x and y position of the host vehicle.
    name= ['Fig' num2str(count)];
%     if count>363
%         R= rem(count,1);
%     elseif count<=363
        R= rem(count,10);
%     end

    if (R==0)
%         f= figure('Name',name, 'Position', get(0, 'Screensize'));
        figure(count);
%         figure.Position= get(0, 'Screensize');
        subplot(6,1,1)
        hold on;
        if flag_ND==0
            for i=1:1:width(x_o)
                disname= sprintf('Path OV %d',i);
                plot(x_o_list(:,i),y_o_list(:,i), 'Color','k', 'DisplayName',disname);
            end
            [bound_OV1]= rect_p(size_veh, x_o, rl);
            [tria_v1]= trian(bound_OV1, distance, x_o);
            rect_plot(bound_OV1, tria_v1,'obst', 'g');
        end
        [bound_OV2]= rect_p(size_veh, x_h, rl);
        tria_v2= [];
        rect_plot(bound_OV2, tria_v2,'host','g');
        plotdata2(const_r)
        plot(x_HV_list,y_HV_list, 'Color','g', 'DisplayName','Path of the Host Vehicle')
        ylim(loc_road_bound);
        yticks(linspace(0,n_lanes*size_lane, n_lanes*2+1));
        xlim([0 road_len]);
        legend('Location', 'northeastoutside')
        xlabel('Distance in the Longitudinal Direction') 
        ylabel({'Lateral'; 'Position'}) 
        title('Path of the Host Vehicle')
        
        subplot(6,1,2)
        hold on;
        plot(x_HV_list,v_x_HV_list, 'Color','g', 'DisplayName','Host Vehicle Velocity')
        plot(x_HV_list,v_x_ref_HV_list, 'Color','r', 'DisplayName','Reference Host Vehicle Velocity','LineStyle', '--')
        if flag_ND==0
            for i=1:1:width(x_o)
                disname= sprintf('Velocity of OV %d',i);
                plot(x_o_list(:,i),v_o_list(:,i), 'Color','k', 'DisplayName',disname);
            end
        end
        xlabel('Distance in the Longitudinal Direction') 
        ylabel({'Longitudinal';' Velocity'})
        title('Longitudinal Velocity of Host Vehicle')
        legend('Location', 'northeastoutside')
        xlim([0 road_len]);
        ylim([f_ineq(2,1) f_ineq(1,1)])
        
        subplot(6,1,3)
        hold on;   
        plot(x_HV_list,l_act_list, 'Color','g', 'DisplayName','Current Lane')
%         plot(x_HV_list,l_act_list,'r', 'DisplayName','Optimal Lane from ','LineStyle', '--')
        xlabel('Distance in the Longitudinal Direction') 
        ylabel('Lane Number')
        title('Optimal Lane of Host Vehicle')
        legend('Location', 'northeastoutside')
        xlim([0 road_len]);
        ylim([-1 2])
        
        subplot(6,1,4)
        plot(x_HV_list,a_HV_list, 'Color','g','DisplayName', 'Acceleration')
        hold on;
        plot(x_HV_list,a_ref_HV_list, 'Color','r', 'DisplayName', 'Reference Acceleration','LineStyle', '--')
        
        xlabel('Distance in the Longitudinal Direction') 
        ylabel('Acceleration')
        title('Acceleration of the Host Vehicle')
        legend('Location', 'northeastoutside')
        xlim([0 road_len]);
        
        subplot(6,1,5)
        hold on;
%         plot(x_HV_list_APF_MPC,flag_delta_list,'g', 'DisplayName','flag_\delta')
        plot(x_HV_list,flag_LC_list,'g', 'DisplayName','flag_{LC}')
%         plot(x_HV_list,flag_ND_list,'r', 'DisplayName','flag_{ND}')
%         plot(x_HV_list_APF_MPC,flag_ND_roi_list,'m', 'DisplayName','Flag_{ND_{roi}}', 'LineStyle', '--')
        xlabel('Distance in the Longitudinal Direction')
        title('Flag for Lane Change')
        legend('Location', 'northeastoutside')
        ylim([-1 2]);
        xlim([0 road_len]);
        
        subplot(6,1,6)
        hold on;
        plot(x_HV_list,del_HV_list,'g', 'DisplayName','Steering Angle')
        plot(x_HV_list,del_ref_HV_list,'r', 'DisplayName','Reference Steering Angle','LineStyle', '--')
        xlabel('Distance in the Longitudinal Direction')
        ylabel('Steering Angle')
        title('Steering Angle')
        legend('Location', 'northeastoutside')
        xlim([0 road_len]);
        
        sgtitle('MIMPC-APFMPC based Integrated Path Planning and Trajectory Tracking (Double Lane Change)')
%         page_name_png=['Fig' num2str(count) '.png'];
%         % Requires R2020a or later
%         exportgraphics(f,page_name_png,'Resolution',300)
    end
    %% Plot 1  
%     % First plot the given data. This data includes the road and lane markers,
%     % the host vehicle (with no tail), the obstacle vehicles with tail, as 
%     % the paths taken by the host and the obstacle vehicles. In the second
%     % plot, plot the host and the obstacle vehicle velocities, third plot
%     % includes the distances between the vehicles, the fourth plot have the
%     % accelation as well as the chance in optimal lane number. Asssumption that
%     % at the initial step the current value is the optimal state of the host
%     % vehicle. 
%     
%     if count>=362
%         display(count);
%     end
%     % x and y position of the host vehicle.
%     name= ['Fig' num2str(count)];
% %     if count>363
% %         R= rem(count,1);
% %     elseif count<=363
%         R= rem(count,10);
% %     end
% 
%     if (R==0)
% %         f= figure('Name',name, 'Position', get(0, 'Screensize'));
%         figure(count)
%         subplot(6,1,1)
%         hold on;
%         if flag_ND==0
% %             for i=1:1:width(x_o)
% %                 disname= sprintf('Path OV %d',i);
% %                 plot(x_o_list(:,i),y_o_list(:,i), 'Color','k', 'DisplayName',disname);
% %             end
%             [bound_OV1]= rect_p(size_veh, x_o, rl);
%             [tria_v1]= trian(bound_OV1, distance, x_o);
%             rect_plot(bound_OV1, tria_v1,'obst', 'c');
%         end
%         [bound_OV2]= rect_p(size_veh, x_h, rl);
%         tria_v2= [];
%         rect_plot(bound_OV2, tria_v2,'host','c');
% %         plotdata2(const_r)
%         plot(x_HV_list,y_HV_list, 'Color','c', 'DisplayName','Path of the Host Vehicle without LPR')
%         ylim(loc_road_bound);
%         yticks(linspace(0,n_lanes*size_lane, n_lanes*2+1));
%         xlim([0 road_len]);
%         legend('Location', 'northeastoutside')
%         xlabel('Distance in the Longitudinal Direction') 
%         ylabel({'Lateral'; 'Position'}) 
%         title('Path of the Host Vehicle')
%         
%         subplot(6,1,2)
%         hold on;
%         plot(x_HV_list,v_x_HV_list, 'Color','c', 'DisplayName','Host Vehicle Velocity without LPR')
%         plot(x_HV_list,v_x_ref_HV_list, 'Color','y', 'DisplayName','Reference Host Vehicle Velocity without LPR', 'LineStyle', '--')
% %         if flag_ND==0
% %             for i=1:1:width(x_o)
% %                 disname= sprintf('Velocity of OV %d',i);
% %                 plot(x_o_list(:,i),v_o_list(:,i), 'Color','k', 'DisplayName',disname);
% %             end
% %         end
%         xlabel('Distance in the Longitudinal Direction') 
%         ylabel({'Longitudinal';' Velocity'})
%         title('Longitudinal Velocity of Host Vehicle')
%         legend('Location', 'northeastoutside')
%         xlim([0 road_len]);
%         ylim([f_ineq(2,1) f_ineq(1,1)])
%         
%         subplot(6,1,3)
%         hold on;   
%         plot(x_HV_list,l_act_list, 'Color','c', 'DisplayName','Current Lane without LPR')
% %         plot(x_HV_list,l_act_list,'y', 'DisplayName','Optimal Lane from MIMPC without LPR', 'LineStyle', '--')
%         xlabel('Distance in the Longitudinal Direction') 
%         ylabel('Lane Number')
%         title('Optimal Lane of Host Vehicle')
%         legend('Location', 'northeastoutside')
%         xlim([0 road_len]);
%         ylim([-1 2])
%         
%         subplot(6,1,4)
%         plot(x_HV_list,a_HV_list, 'Color','c','DisplayName', 'Acceleration without LPR')
%         hold on;
% %         plot(x_HV_list,a_ref_HV_list, 'Color','r', 'LineStyle', '--', 'DisplayName', 'Reference Acceleration')
%         
%         xlabel('Distance in the Longitudinal Direction') 
%         ylabel('Acceleration')
%         title('Acceleration of the Host Vehicle')
%         legend('Location', 'northeastoutside')
%         xlim([0 road_len]);
%         
%         subplot(6,1,5)
%         hold on;
% %         plot(x_HV_list_APF_MPC,flag_delta_list,'g', 'DisplayName','flag_\delta')
%         plot(x_HV_list,flag_LC_list,'c', 'DisplayName','flag_{LC} without LPR')
% %         plot(x_HV_list,flag_ND_list,'r', 'DisplayName','flag_{ND}')
% %         plot(x_HV_list_APF_MPC,flag_ND_roi_list,'m', 'DisplayName','Flag_{ND_{roi}}', 'LineStyle', '--')
%         xlabel('Distance in the Longitudinal Direction')
%         title('Flag for Lane Change')
%         legend('Location', 'northeastoutside')
%         ylim([-1 2]);
%         xlim([0 road_len]);
%         
%         subplot(6,1,6)
%         hold on;
%         plot(x_HV_list,del_HV_list,'c', 'DisplayName','Steering Angle without LPR')
% %         plot(x_HV_list,del_ref_HV_list,'r', 'DisplayName','Reference Steering Angle', 'LineStyle', '--')
%         xlabel('Distance in the Longitudinal Direction')
%         ylabel('Steering Angle')
%         title('Steering Angle')
%         legend('Location', 'northeastoutside')
%         xlim([0 road_len]);
%         sgtitle('MIMPC-APFMPC based Integrated Path Planning and Trajectory Tracking (Double Lane Change)')
% %         page_name_png=['Fig' num2str(count) '.png'];
% %         % Requires R2020a or later
% %         exportgraphics(f,page_name_png,'Resolution',300)
%     end
end

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
% model with varying longitudinal velocity. Linearized model.
function ss_d= car_modelAPFMPC(const, T, x_0, u)
    C_f= const(1);        % Negative front tire cornering stiffness
    C_r= const(2);       % Negative rear tire cornering stiffness
    L_wb= const(3);          % Wheel base
    l_r= const(4);         % Distance between rear axle and vehicle CoG
    l_f= const(5);         % Distance between front axle and vehicle CoG
    I_z= const(6);       % Vehicle yaw moment of inertia
    m= const(7);            % Vehicle Mass
    
    v_x = x_0(1,1);
    x = x_0(2,1);
    y = x_0(3,1);
    v_y = x_0(4,1);
    psi_dot = x_0(5,1);
    psit = x_0(6,1);
    
    Fx= u(1,1);
    delta= u(2,1);
%%
    A1= -v_x * sind(psit) - v_y * cosd(psit);
    A2= v_x * cosd(psit) + v_y * sind(psit);
    A3= ((C_f + C_r)* v_y)/(m* v_x^2)+ ((l_f * C_f - l_r * C_r) * psi_dot)/(m* v_x^2)- psi_dot;
    A4= (l_r * C_r - l_f * C_f)/(m* v_x);
    A5= ((l_f * C_f - l_r * C_r) * v_y)/(I_z* v_x^2) + ((l_f^2 * C_f + l_r^2 * C_r) * psi_dot)/(I_z* v_x^2);
    A6= (l_r * C_r - l_f * C_f)/(I_z* v_x);
    A7= (-l_f^2 * C_f - l_r^2 * C_r)/(I_z* v_x);
    A8= tand(delta)/(l_f + l_r);
    A9= -(C_f + C_r)/(m* v_x);
    
    
    B0= 1;
    B1= C_f/m;
    B2= (l_f*C_f)/I_z;
    B3= (v_x * (secd(delta))^2)/ (l_f + l_r);
    % Matrices of the continuous-time car model using ZOH
    A= [       0          0       0         psi_dot          v_y         0;
            cosd(psit)    0       0       -sind(psit)         0          A1;
            sind(psit)    0       0        cosd(psit)         0          A2;
               A3         0       0           A9              A4         0;
               A5         0       0           A6              A7         0;
               A8         0       0            0               1         0;];

    B= [    B0      0;
            0       0;
            0       0;
            0       B1;
            0       B2;
            0       B3;];
%% 
    C= [    1       0       0       0       0    0;
            0       0       1       0       0    0;
            0       0       0       1       0    0;
            0       0       0       0       1    0;
            0       0       0       0       0    1;];    
%     C= [    0       0       1       0       0       0;
%             0       0       0       1       0       0;
%             0       0       0       0       1       0;
%             0       0       0       0       0       1;];
%     C= [    0       0       1       0       0       0;];
        
%     C= [  0       0       0       1       0       0;
%             0       0       0       0       1       0;
%             0       0       0       0       0       1;];
    if v_x==0
        A= [        0	0	0	0   0   0;
                    0   1   0   0   0   0;
                    0   0   0   0   0   0;
                   	0   0   0   0   0   0;
                  	0   0   0   0   0   0;
                   	0   0   0   0   0   0;];
        B= [    0      0;
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
                if x_o_front_SL(2,1)- x_h(2,1)<=230+ (x_h(1,1)-x_o_front_SL(1,1))*size_veh(1,1)
                    vx_ref= x_o_front_SL(1,1);
                end
            elseif flag_LC==1 % Implies that there is no vehicle in the adjacent
                % lane in the safety region.
                vx_ref= x_h_0(1,1);
                if x_o_front_SL(2,1)- x_h(2,1)<=distance(index41)+15*size_veh(1,1)
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

function flag_LC= set_flag_LC(x_h, x_o, lane_host, lane_obst, d_safe_max, size_lane, a_x_max, distance, x_h_0, size_veh)
%%
    dist_LC= size_lane*sind(abs(atand(x_h(4,1)/x_h(1,1))));
    time_LC=  dist_LC/sqrt(x_h(1,1)^2+x_h(4,1)^2);
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
    

    if flag_delta==0
        flag_LC=0;
    elseif flag_delta==1
        if x_o_front_SL(1,1)-x_h_0(1,1)<= 0
            if flag_epsilon1==0 && flag_epsilon2==0 % There are no vehicles
                % infront of or behind the HV in the adjacent lane.
                flag_LC=1;
            elseif flag_epsilon1==1 && flag_epsilon2==0 % There are vehicles
                % infront of the HV in the adjacent lane but not behind the
                % vehicle.
                if x_o_front_AL(2,1)-x_h(2,1)<= d_safe_max+5*size_veh(1,1)-...
                    (x_h(1,1)-x_o_front_AL(1,1))*time_LC 
                    % Here I have to add another check which checks if the
                    % vehicle can slow down without crossing the boundary, If
                    % not then flag_LC==1 rather than flag_LC==0 
                    if x_o_front_AL(1,1)-x_o_front_SL(1,1)>0 &&...
                            (x_o_front_SL(1,1)-x_h(1,1))/a_x_max<= x_o_front_SL(2,1)-x_h(2,1)-distance(index41,1) &&...
                            x_o_front_SL(2,1)-x_h(2,1)-distance(index41,1)<x_o_front_AL(2,1)-x_h(2,1)-distance(index42,1)
                        flag_LC=1;
                    else
                        flag_LC=0;
                    end
                else
                    flag_LC=1;
                end
            elseif flag_epsilon1==0 && flag_epsilon2==1
                if x_h(2,1)-x_o_rear_AL(2,1)>=  d_safe_max + ...
                    (x_h(1,1)-x_o_rear_AL(1,1))*time_LC

                    flag_LC=1;
                else
                    flag_LC=0;
                end
            elseif flag_epsilon1==1 && flag_epsilon2==1
                if x_o_front_AL(2,1)-x_h(2,1)<= d_safe_max+5*size_veh(1,1)-...
                    (x_h(1,1)-x_o_front_AL(1,1))*time_LC  &&...
                   x_h(2,1)-x_o_rear_AL(2,1)>=  d_safe_max + ...
                   (x_h(1,1)-x_o_rear_AL(1,1))*time_LC
                    if x_o_front_AL(1,1)-x_o_front_SL(1,1)>0  &&...
                            (x_o_front_SL(1,1)-x_h(1,1))/a_x_max<= x_o_front_SL(2,1)-x_h(2,1)-distance(index41,1) &&...
                            x_o_front_SL(2,1)-x_h(2,1)-distance(index41,1)<x_o_front_AL(2,1)-x_h(2,1)-distance(index42,1)
                        flag_LC=1;
                    else
                        flag_LC=0;
                    end
                else
                    flag_LC=1; 
                end
            end
        else
            flag_LC=0;
        end
    end 
end

% Function which generates the matrices to be used in the quadratic cost
% function which represent the taylor series approximation of the APF.
function [A1, A2, A3]= taylor2ndOrder(const_r, const_o, x_o, x_h, select, rl, distance,size_veh)
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
            for i = 1:length(loc_road_bound)
% %                 apf_road(i)= 0.5* eta * (1/(y- loc_road_bound(i)))^2;
% %                 obst_ur_x(i) = 0;
% %                 obst_ur_y(i) = -eta/((y- loc_road_bound(i))^3);
% %                 obst_ur_xx(i) = 0;
% %                 obst_ur_xy(i) = 0;
% %                 obst_ur_yx(i) = 0;
% %                 obst_ur_yy(i) = (3*eta)/((y-loc_road_bound(i))^4);
                apf_road(i)= 0.5* eta * (1/(y- loc_road_bound(i)))^4;
                obst_ur_x(i) = 0;
                obst_ur_y(i) = -(2*eta)/((y- loc_road_bound(i))^5);
                obst_ur_xx(i) = 0;
                obst_ur_xy(i) = 0;
                obst_ur_yx(i) = 0;
                obst_ur_yy(i) = (10*eta)/((y-loc_road_bound(i))^6);
            end
            a1= sum(apf_road);
            a2= sum(obst_ur_x);
            a3= sum(obst_ur_y);
            a4= sum(obst_ur_xx);
            a5= sum(obst_ur_xy);
            a6= sum(obst_ur_yx);
            a7= sum(obst_ur_yy);
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
function rect_plot(vert, tria_v,select, color)
    if select=='obst' % when plotting an obstacle
        for i=1:1:length(vert)
            v= tria_v{i};
            vertices= vert{i};
            x_vert= vertices(:,1); 
            y_vert= vertices(:,2);
            x_vert_plot{i}= [x_vert(1:3,1);v(1,1);x_vert(4,1);x_vert(1,1)];
            y_vert_plot{i}= [y_vert(1:3,1);v(1,2);y_vert(4,1);y_vert(1,1)];
            plot(x_vert_plot{i}, y_vert_plot{i},'HandleVisibility','off', 'Color', color);
            hold on
        end
    elseif select=='host' % when plotting the host vehicle
         for i=1:1:length(vert)
            vertices= vert{i};
            x_vert= vertices(:,1); 
            y_vert= vertices(:,2);
            x_vert_plot{i}= [x_vert;x_vert(1,1)];
            y_vert_plot{i}= [y_vert;y_vert(1,1)];
            plot(x_vert_plot{i}, y_vert_plot{i},'HandleVisibility','off', 'Color', color);
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
            u=-(const(1,1)*log(const(1,2)*K))/(const(1,2)*K);
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
