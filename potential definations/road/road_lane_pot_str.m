    clear all;
clc;
%% Data input (RL Potentail)

% decided that the global origin moved along the boundary of the right lane
% along with the HV (host vehicle).

n_lanes= 2; % number of lanes
size_lane = 3; % width of lane in meters
road_len= 1000;  % length of the road in meters
data_count= 500;
Y_var= linspace(0, 6, data_count);
X_var= linspace(231, 238, data_count);
for i= 1:n_lanes-1
    loc_lane_bound(i) = i*size_lane; % Y location of the lane internal boundaries (does not include the outer boundaries of the road)
end
loc_road_bound = [0 n_lanes*size_lane]; % Y values of the boundaries of the road
% Two different potentials are to be generated: a lane potential and a road potential for a straight road
U_lane_fin= zeros(length(Y_var),length(X_var)); % Matrix defined to store the lane potential values
U_road_fin= zeros(length(Y_var),length(X_var)); % Matrix defined to store the road potential values

%% Lane Potential
A_lane = 2; % the height of the lane divider potential
sig_lane = 0.3* size_lane; % determines how quickly the potential rises/falls and is proportional to the lane width
for i= 1:n_lanes-1
    for j= 1:length(Y_var)
        for k= 1:length(X_var)
            U_lane(i,j,k)= A_lane* exp(-((Y_var(j)- loc_lane_bound(i))^2/(2* sig_lane^2)));
        end
    end
end
for i=1:n_lanes-1
    U_lane_temp1= U_lane(i,:,:);
    U_lane_temp2= squeeze(U_lane_temp1);
    U_lane_fin= U_lane_fin+ U_lane_temp2;
end

% surf(X_var,Y_var,U_lane_fin);
% shading interp;

%% Road Potential
% eta= 1000;
% a= 2;
% b=100000;
eta= 1;
a= 1;
b=100;
for i= 1:length(loc_road_bound)
    for j= 1:length(Y_var)
        for k= 1:length(X_var)
%             U_road(i,j,k)= 0.5* eta * (1/(Y_var(j)- loc_road_bound(i)))^2;
%             U_road(i,j,k)= b*(exp(a*(Y_var(j)-4.5))-a*(Y_var(j)-4.5)-1)+ 0.5* eta * (1/(Y_var(j)- loc_road_bound(i)))^2;
            U_road(i,j,k)= b*(exp(a*(Y_var(j)-3))-a*(Y_var(j)-3)-1)+ 0.5* eta * (1/(Y_var(j)- loc_road_bound(i)))^2;
%             U_obst(i,j,k)= 
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

%% Road + Lane Potential

U_RplusL = U_road_fin;

surf(X_var,Y_var,U_RplusL);
shading interp;
grid off;
xlabel('Longitudinal Position')
ylabel('Lateral Position')
zlabel('Cost of the Potential Function')
yticks(0:0.5:6)
ytickangle(45)
% legend(s, 'Cost around the Obstacle')
% legend
