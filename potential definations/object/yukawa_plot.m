%%
% [1]E. Snapper, “Model-based Path Planning and Control for Autonomous
% Vehicles using Artificial Potential Fields,” master’s thesis, 
% January 2018.

% [2] MIA ISAKSSON PALMQVIST, "Model Predictive Control for Autonomous 
% Driving of a Truck, master’s thesis, 2016.

% [3] W. Jansen, "Lateral Path-Following Control for Automated Vehicle 
% Platoons", ,master's thesis, 2016 


% Given the model of the system, a reference trajectory and the cost
% function, create an MPC which can follow the reference trajectory.

%% Note1: Rewrite the region1 and region2 values to get the correct values when the obstacle is also changing lanes and other scenarios

clear all;
clc;

%% Host Data

% states are of the form [v_x X Y v_y r psi]. r is the yaw rate and psi is
% the yaw angle. v_x and v_y are longitudinal and lateral velocities and X
% Y are  coordinates.
x_h_0= [0 235 1 0 0 0]'; % state of the host vehicle. 
u_h1= [0];

% U=[F_x delta]'
u_h2= [0 0]';

%% Road Data

% decided that the global origin moved along the boundary of the right lane
% along with the HV (host vehicle).
n_lanes= 2; % number of lanes
size_lane = 3; % width of lane in meters
road_len= 1000;  % length of the road in meters
% digits(5)
data_count=3000;
alpha= 1000;
Y_var= linspace(0, 6, data_count);
X_var= linspace(220, 245, data_count);
for i= 1:n_lanes-1
    loc_lane_bound(i) = i*size_lane; % Y location of the lane internal
    % boundaries (does not include the outer boundaries of the road)
end
loc_road_bound = [0 n_lanes*size_lane]; % Y values of the boundaries of
% the road

all_bound= [0 loc_lane_bound n_lanes*size_lane];
for i=1:1:n_lanes
    loc_lane_cent(i)=(all_bound(i)+all_bound(i+1))/2;
end
r_data= [n_lanes size_lane];
eta= 1;
% A_lane = 5; % the height of the lane divider potential
% sig_lane = 0.3* size_lane; % determines how quickly the potential rises/falls and is proportional to the lane width
A_lane = 1; % the height of the lane divider potential
sig_lane = 0.15* size_lane; % determines how quickly the potential rises/falls and is proportional to the lane width
A_lane1 = 20; % the height of the lane divider potential
sig_lane1 = 0.13* size_lane; % determines how quickly the potential rises/falls and is proportional to the lane width
const_r= [eta A_lane sig_lane A_lane1 sig_lane1 n_lanes loc_lane_bound loc_road_bound all_bound];
% U_lane_fin= zeros(length(Y_var),length(X_var)); % Matrix defined to store the lane potential values
% U_road_fin= zeros(length(Y_var),length(X_var)); % Matrix defined to store the road potential values

%% Obstacle Data

x_o_0= [0 235 3 0 0 0]';

% Data to generate the obstacle potential field
% For modyuk with log
A_o= 10;
b_o= 0.1;
% for yukawa
% A_o= 100;
% b_o= 0.001;
const_o= [A_o b_o]; 
size_veh= [2.5 1.5]; % Size of the vehicle

%%

select1= 1;% select1 variable is used to define if the host vehicle is
% moving to the left or the right. Use select1= 1 for right and select1=2
% for left

[bound_OV]= rect_p(size_veh, x_o_0, select1);% used to define the vertices
% of the rectangle which represents the obstacle vehicle. 


distance= 5; % This is the distance that the third vertex of the triangle
% added behind the host vehicle (for help with safe distance and lane
% change) is at.

[tria_v]= trian(bound_OV, distance, x_o_0); % Function to generate the 
% third vertex of the triangle.

[v_alp1 v_alp2]= vertex(bound_OV, tria_v);% Function to generate the
% vertices of the extended host vehicle ( rectangle + triangle ) in order.
% The order starts from the bottom right (left) when select1= 1 (0)

[si_form]= equations(v_alp1, v_alp2, x_o_0); % Function to generate the 
% the equations of the lines representing the edges of the extended host
% vehicle along with other lines in slope-intercept form.

% [x_vert_plot y_vert_plot]= rect_plot(bound_OV, tria_v);% 2D plot of the 
% % obstacle vehicle.
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 100);% 2D plot of
% % of the obstacle vehicle at a height of alpha meters. This is used to
% % check if the potential function works well at all heights.

%%
x_h= x_h_0;
x_h1= x_h_0;
% Given the equation of the lines containing each region, and the heading
% angle of the obstacles, the HV can be in a different region. Here we have
% to check where the host vehicle is wrt each of the obstacles. This can be
% done using the flag variable above. Having obtained the flag variable,
% we have to select the corresponding lines which enclose the region
temp= vertcat(si_form{:});
m= temp(:,1);
b= temp(:,2);
v1= v_alp1{:,1};
alpha= 1000;
%%
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 238, data_count);
% x_o_0= [0 235 3 0 0 0]';
% count=0;
% alpha= 10000;
% apf_obst_fin=0;
% time2= tic;
% for i= 1:length(Y_var)
%     for j=1:length(X_var)
%         x_h_0(2,1)= X_var(j);
%         x_h_0(3,1)= Y_var(i);
%         for k=1:width(x_o_0)
%             apf_obst_fin1{1,k}= U_obst(const_o, x_o_0(:,k), x_h_0, select1, distance, size_veh);
%         end
%         if width(x_o_0)>1
%             apf_obst_fin= plus(apf_obst_fin1{1,:});
%         else
%             apf_obst_fin= apf_obst_fin1{1,1};
%         end
%         apf_obst1(i,j)= double(apf_obst_fin(X_var(j),Y_var(i)));
%         if imag(apf_obst1(i,j))~=0
%             apf_obst1(i,j)=abs(apf_obst1(i,j));
%             count= count+1;
%         end
%             
%         if apf_obst1(i,j)>alpha
%             apf_obst1(i,j)= alpha;
%         end        
%     end
% end
% 
% % 
% figure(123456)
% view(3)
% hold on;
% % surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% s=surf(X_var,Y_var,apf_obst1);
% % contour(X_var,Y_var,apf_obst1, 10000);
% % hold off;
% hold on
% xlabel('Horizontal Distance')
% ylabel('Vertical Distance')
% zlabel('Cost of the Potential Function')
% legend(s, 'Cost around the Obstacle')
% % legend
% toc(time2)

% %% For region 1
% x_h_0= [0 238 3 0 0 0]';
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 245, data_count);
% tic
% for i= 1:1:width(x_o_0)
%     if x_o_0(6,i)>0
%         flag(1,i)= region1(si_form(:,i), x_h_0);
%     elseif x_o_0(6,i)==0
%         flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
%     elseif x_o_0(6,i)<0
%         flag(1,i)= region3(si_form(:,i), x_h_0);
%     end
% end
% count1=zeros(1,width(x_o_0));
% count2=zeros(1,width(x_o_0));
% X_vartempin= zeros(1,0);
% Y_vartempin= zeros(1,0);
% X_vartempout= zeros(1,0);
% Y_vartempout= zeros(1,0);
% phone= zeros(1,0,0);
% for j= 1:length(Y_var)
%     for k=1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag1(i)= region1(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)==0
%                 flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)<0
%                 flag1(i)= region3(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%         end
% %         apf_obst1(j,k)= sum(u);
%     end
% end
% figure(1)
% hold on;
% plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 1')
% plot(X_vartempout, Y_vartempout,'b*', 'DisplayName','Rest of the Regions')
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active Region when the HV is in Region 1')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% hold off;
% apf_obst2= zeros(1,0);
% for j= 1:length(Y_var)
%     for k= 1:1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag(1,i)= region1(si_form(:,i), x_h);
%             elseif x_o_0(6,i)==0
%                 flag(1,i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%             elseif x_o_0(6,i)<0
%                 flag(1,i)= region3(si_form(:,i), x_h);
%             end
%             if phone(j,k,i)==1
%                 [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(1,i), v_alp1{:,i}, x_h, const_o);
%             elseif phone(j,k,i)==0
%                 u(i)=0;
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             apf_obst2(j,k)= sum(u);
%             if apf_obst2(j,k)>1000
%                 apf_obst2(j,k)= 1000;
%             end
%         end
%         
%     end
% end
% figure(11)
% % subplot(2,1,1)
% view(3)
% hold on;
% s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 1');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% zlabel('Cost of the Potential Function')
% title('Active APF (surface) when the HV is in Region 1')
% legend;
% % legend(s, 'Potential For Region 1')
% xlim([220 245]);
% ylim([0 6]);
% figure(12)
% view(3)
% hold on;
% c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
% % [x_vert_plot y_vert_plot]= rect_plot(bound_OV, tria_v);
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active APF (contour) when the HV is in Region 1 ')
% legend;
% % legend(c, 'Contour of potential when HV lies in Region 1')
% xlim([220 245]);
% ylim([0 6]);
% toc
% 
% %% For Region 2
% x_h_0= [0 238 6 0 0 0]';
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 245, data_count);
% tic
% for i= 1:1:width(x_o_0)
%     if x_o_0(6,i)>0
%         flag(1,i)= region1(si_form(:,i), x_h_0);
%     elseif x_o_0(6,i)==0
%         flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
%     elseif x_o_0(6,i)<0
%         flag(1,i)= region3(si_form(:,i), x_h_0);
%     end
% end
% count1=zeros(1,width(x_o_0));
% count2=zeros(1,width(x_o_0));
% X_vartempin= zeros(1,0);
% Y_vartempin= zeros(1,0);
% X_vartempout= zeros(1,0);
% Y_vartempout= zeros(1,0);
% phone= zeros(1,0,0);
% for j= 1:length(Y_var)
%     for k=1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag1(i)= region1(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)==0
%                 flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)<0
%                 flag1(i)= region3(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%         end
% %         apf_obst1(j,k)= sum(u);
%     end
% end
% figure(2)
% hold on;
% plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 2')
% plot(X_vartempout, Y_vartempout, 'b*', 'DisplayName','Rest of the Regions')
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active Region when the HV is in Region 2')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% hold off;
% apf_obst2= zeros(1,0);
% for j= 1:length(Y_var)
%     for k= 1:1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag(1,i)= region1(si_form(:,i), x_h);
%             elseif x_o_0(6,i)==0
%                 flag(1,i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%             elseif x_o_0(6,i)<0
%                 flag(1,i)= region3(si_form(:,i), x_h);
%             end
%             if phone(j,k,i)==1
%                 [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(1,i), v_alp1{:,i}, x_h, const_o);
%             elseif phone(j,k,i)==0
%                 u(i)=0;
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             apf_obst2(j,k)= sum(u);
%             if apf_obst2(j,k)>1000
%                 apf_obst2(j,k)= 1000;
%             end
%         end
%         
%     end
% end
% figure(21)
% % subplot(2,1,1)
% view(3)
% hold on;
% s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 2');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% zlabel('Cost of the Potential Function')
% title('Active APF (surface) when the HV is in Region 2')
% legend;
% % legend(s, 'Potential For Region 2')
% xlim([220 245]);
% ylim([0 6]);
% figure(22)
% view(3)
% hold on;
% c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active APF (contour) when the HV is in Region 2')
% legend
% xlim([220 245]);
% ylim([0 6]);
% toc
% 
% %% For region 3
% x_h_0= [0 235 6 0 0 0]';
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 245, data_count);
% tic
% for i= 1:1:width(x_o_0)
%     if x_o_0(6,i)>0
%         flag(1,i)= region1(si_form(:,i), x_h_0);
%     elseif x_o_0(6,i)==0
%         flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
%     elseif x_o_0(6,i)<0
%         flag(1,i)= region3(si_form(:,i), x_h_0);
%     end
% end
% count1=zeros(1,width(x_o_0));
% count2=zeros(1,width(x_o_0));
% X_vartempin= zeros(1,0);
% Y_vartempin= zeros(1,0);
% X_vartempout= zeros(1,0);
% Y_vartempout= zeros(1,0);
% phone= zeros(1,0,0);
% for j= 1:length(Y_var)
%     for k=1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag1(i)= region1(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)==0
%                 flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)<0
%                 flag1(i)= region3(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%         end
% %         apf_obst1(j,k)= sum(u);
%     end
% end
% figure(3)
% hold on;
% plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 3')
% plot(X_vartempout, Y_vartempout, 'b*',  'DisplayName','Rest of the Regions')
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active Region when the HV is in Region 3')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% hold off;
% apf_obst2= zeros(1,0);
% for j= 1:length(Y_var)
%     for k= 1:1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag(1,i)= region1(si_form(:,i), x_h);
%             elseif x_o_0(6,i)==0
%                 flag(1,i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%             elseif x_o_0(6,i)<0
%                 flag(1,i)= region3(si_form(:,i), x_h);
%             end
%             if phone(j,k,i)==1
%                 [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(1,i), v_alp1{:,i}, x_h, const_o);
%             elseif phone(j,k,i)==0
%                 u(i)=0;
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             apf_obst2(j,k)= sum(u);
%             if apf_obst2(j,k)>1000
%                 apf_obst2(j,k)= 1000;
%             end
%         end
%         
%     end
% end
% figure(31)
% % subplot(2,1,1)
% view(3)
% hold on;
% s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 3');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% zlabel('Cost of the Potential Function')
% title('Active APF (surface) when the HV is in Region 3')
% legend;
% % legend(s, 'Potential For Region 3')
% xlim([220 245]);
% ylim([0 6]);
% figure(32)
% view(3)
% hold on;
% c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active APF (contour) when the HV is in Region 3')
% legend
% xlim([220 245]);
% ylim([0 6]);
% toc
% 
% %% For region 4
% x_h_0= [0 233.49 6 0 0 0]';
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 245, data_count);
% tic
% for i= 1:1:width(x_o_0)
%     if x_o_0(6,i)>0
%         flag(1,i)= region1(si_form(:,i), x_h_0);
%     elseif x_o_0(6,i)==0
%         flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
%     elseif x_o_0(6,i)<0
%         flag(1,i)= region3(si_form(:,i), x_h_0);
%     end
% end
% count1=zeros(1,width(x_o_0));
% count2=zeros(1,width(x_o_0));
% X_vartempin= zeros(1,0);
% Y_vartempin= zeros(1,0);
% X_vartempout= zeros(1,0);
% Y_vartempout= zeros(1,0);
% phone= zeros(1,0,0);
% for j= 1:length(Y_var)
%     for k=1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag1(i)= region1(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)==0
%                 flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)<0
%                 flag1(i)= region3(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%         end
% %         apf_obst1(j,k)= sum(u);
%     end
% end
% figure(4)
% hold on;
% plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 4')
% plot(X_vartempout, Y_vartempout, 'b*',  'DisplayName','Rest of the Regions')
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active Region when the HV is in Region 4')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% hold off;
% apf_obst2= zeros(1,0);
% for j= 1:length(Y_var)
%     for k= 1:1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag(1,i)= region1(si_form(:,i), x_h);
%             elseif x_o_0(6,i)==0
%                 flag(1,i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%             elseif x_o_0(6,i)<0
%                 flag(1,i)= region3(si_form(:,i), x_h);
%             end
%             if phone(j,k,i)==1
%                 [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(1,i), v_alp1{:,i}, x_h, const_o);
%             elseif phone(j,k,i)==0
%                 u(i)=0;
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             apf_obst2(j,k)= sum(u);
%             if apf_obst2(j,k)>1000
%                 apf_obst2(j,k)= 1000;
%             end
%         end
%         
%     end
% end
% figure(41)
% % subplot(2,1,1)
% view(3)
% hold on;
% s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 4');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% zlabel('Cost of the Potential Function')
% title('Active APF (surface) when the HV is in Region 4')
% legend;
% % legend(s, 'Potential For Region 4')
% xlim([220 245]);
% ylim([0 6]);
% figure(42)
% view(3)
% hold on;
% c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active APF (contour) when the HV is in Region 4')
% legend
% xlim([220 245]);
% ylim([0 6]);
% toc

%% For region 5
x_h_0= [0 229.5 6 0 0 0]';
Y_var= linspace(0, 6, data_count);
X_var= linspace(220, 245, data_count);
for i= 1:1:width(x_o_0)
    if x_o_0(6,i)>0
        flag(1,i)= region1(si_form(:,i), x_h_0);
    elseif x_o_0(6,i)==0
        flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
    elseif x_o_0(6,i)<0
        flag(1,i)= region3(si_form(:,i), x_h_0);
    end
end
count1=zeros(1,width(x_o_0));
count2=zeros(1,width(x_o_0));
X_vartempin= zeros(1,0);
Y_vartempin= zeros(1,0);
X_vartempout= zeros(1,0);
Y_vartempout= zeros(1,0);
phone= zeros(1,0,0);
for j= 1:length(Y_var)
    for k=1:length(X_var)
        x_h(2,1)= X_var(k);
        x_h(3,1)= Y_var(j);
        for i= 1:1:width(x_o_0)
            if x_o_0(6,i)>0
                flag1(i)= region1(si_form(:,i), x_h);
                if flag1(1,i)==flag(1,i)
%                     count1= count1+1;
                    count1(1,i)= count1(1,i)+1;
                    X_vartempin(i,count1(1,i))= x_h(2,1);
                    Y_vartempin(i,count1(1,i))= x_h(3,1);
                    phone(j,k,i)= 1;
                else
%                     count2= count2+1;
                    count2(1,i)= count2(1,i)+1;
                    X_vartempout(i,count2(1,i))= x_h(2,1);
                    Y_vartempout(i,count2(1,i))= x_h(3,1);
                    phone(j,k,i)= 0;
                end
            elseif x_o_0(6,i)==0
                flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
                if flag1(1,i)==flag(1,i)
%                     count1= count1+1;
                    count1(1,i)= count1(1,i)+1;
                    X_vartempin(i,count1(1,i))= x_h(2,1);
                    Y_vartempin(i,count1(1,i))= x_h(3,1);
                    phone(j,k,i)= 1;
                else
%                     count2= count2+1;
                    count2(1,i)= count2(1,i)+1;
                    X_vartempout(i,count2(1,i))= x_h(2,1);
                    Y_vartempout(i,count2(1,i))= x_h(3,1);
                    phone(j,k,i)= 0;
                end
            elseif x_o_0(6,i)<0
                flag1(i)= region3(si_form(:,i), x_h);
                if flag1(1,i)==flag(1,i)
%                     count1= count1+1;
                    count1(1,i)= count1(1,i)+1;
                    X_vartempin(i,count1(1,i))= x_h(2,1);
                    Y_vartempin(i,count1(1,i))= x_h(3,1);
                    phone(j,k,i)= 1;
                else
%                     count2= count2+1;
                    count2(1,i)= count2(1,i)+1;
                    X_vartempout(i,count2(1,i))= x_h(2,1);
                    Y_vartempout(i,count2(1,i))= x_h(3,1);
                    phone(j,k,i)= 0;
                end
            end
%             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
        end
%         apf_obst1(j,k)= sum(u);
    end
end
figure(5)
hold on;
plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 5')
plot(X_vartempout, Y_vartempout, 'b*',  'DisplayName','Rest of the Regions')
surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
xlabel('Longitudinal Distance')
ylabel('Lateral Distance')
title('Active Region when the HV is in Region 5')
legend;
xlim([220 245]);
ylim([0 6]);
hold off;
apf_obst2= zeros(1,0);
for j= 1:length(Y_var)
    for k= 1:1:length(X_var)
        x_h(2,1)= X_var(k);
        x_h(3,1)= Y_var(j);
        for i= 1:1:width(x_o_0)
            if x_o_0(6,i)>0
                flag(i)= region1(si_form(:,i), x_h);
            elseif x_o_0(6,i)==0
                flag(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
            elseif x_o_0(6,i)<0
                flag(i)= region3(si_form(:,i), x_h);
            end
            if phone(j,k,i)==1
                [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
            elseif phone(j,k,i)==0
                u(i)=0;
            end
%             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
            apf_obst2(j,k)= sum(u);
            if apf_obst2(j,k)>1000
                apf_obst2(j,k)= 1000;
            end
        end
        
    end
end
figure(51)
% subplot(2,1,1)
view(3)
hold on;
s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 5');
surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
hold off;
shading interp;
xlabel('Longitudinal Distance')
ylabel('Lateral Distance')
zlabel('Cost of the Potential Function')
title('Active APF (surface) when the HV is in Region 5')
legend;
xlim([220 245]);
ylim([0 6]);
% legend(s, 'Potential For Region 5')
figure(52)
view(3)
hold on;
c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
hold off;
shading interp;
xlabel('Longitudinal Distance')
ylabel('Lateral Distance')
title('Active APF (contour) when the HV is in Region 5')
legend
xlim([220 245]);
ylim([0 6]);
% toc

% %% For region 6
% x_h_0= [0 228.49 3 0 0 0]';
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 245, data_count);
% for i= 1:1:width(x_o_0)
%     if x_o_0(6,i)>0
%         flag(1,i)= region1(si_form(:,i), x_h_0);
%     elseif x_o_0(6,i)==0
%         flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
%     elseif x_o_0(6,i)<0
%         flag(1,i)= region3(si_form(:,i), x_h_0);
%     end
% end
% count1=zeros(1,width(x_o_0));
% count2=zeros(1,width(x_o_0));
% X_vartempin= zeros(1,0);
% Y_vartempin= zeros(1,0);
% X_vartempout= zeros(1,0);
% Y_vartempout= zeros(1,0);
% phone= zeros(1,0,0);
% for j= 1:length(Y_var)
%     for k=1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag1(i)= region1(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)==0
%                 flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)<0
%                 flag1(i)= region3(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%         end
% %         apf_obst1(j,k)= sum(u);
%     end
% end
% figure(6)
% hold on;
% plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 6')
% plot(X_vartempout, Y_vartempout, 'b*',  'DisplayName','Rest of the Regions')
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active Region when the HV is in Region 6')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% hold off;
% apf_obst2= zeros(1,0);
% for j= 1:length(Y_var)
%     for k= 1:1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag(i)= region1(si_form(:,i), x_h);
%             elseif x_o_0(6,i)==0
%                 flag(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%             elseif x_o_0(6,i)<0
%                 flag(i)= region3(si_form(:,i), x_h);
%             end
%             if phone(j,k,i)==1
%                 [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             elseif phone(j,k,i)==0
%                 u(i)=0;
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             apf_obst2(j,k)= sum(u);
%             if apf_obst2(j,k)>1000
%                 apf_obst2(j,k)= 1000;
%             end
%         end
%         
%     end
% end
% figure(61)
% % subplot(2,1,1)
% view(3)
% hold on;
% s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 6');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% zlabel('Cost of the Potential Function')
% title('Active APF (surface) when the HV is in Region 6')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% % legend(s, 'Potential For Region 6')
% figure(62)
% view(3)
% hold on;
% c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active APF (contour) when the HV is in Region 6')
% legend
% xlim([220 245]);
% ylim([0 6]);
% toc
% 
% %% For region 7
% x_h_0= [0 229.5 0 0 0 0]';
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 245, data_count);
% for i= 1:1:width(x_o_0)
%     if x_o_0(6,i)>0
%         flag(1,i)= region1(si_form(:,i), x_h_0);
%     elseif x_o_0(6,i)==0
%         flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
%     elseif x_o_0(6,i)<0
%         flag(1,i)= region3(si_form(:,i), x_h_0);
%     end
% end
% count1=zeros(1,width(x_o_0));
% count2=zeros(1,width(x_o_0));
% X_vartempin= zeros(1,0);
% Y_vartempin= zeros(1,0);
% X_vartempout= zeros(1,0);
% Y_vartempout= zeros(1,0);
% phone= zeros(1,0,0);
% for j= 1:length(Y_var)
%     for k=1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag1(i)= region1(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)==0
%                 flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)<0
%                 flag1(i)= region3(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%         end
% %         apf_obst1(j,k)= sum(u);
%     end
% end
% figure(7)
% hold on;
% plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 7')
% plot(X_vartempout, Y_vartempout, 'b*',  'DisplayName','Rest of the Regions')
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active Region when the HV is in Region 7')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% hold off;
% apf_obst2= zeros(1,0);
% for j= 1:length(Y_var)
%     for k= 1:1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag(i)= region1(si_form(:,i), x_h);
%             elseif x_o_0(6,i)==0
%                 flag(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%             elseif x_o_0(6,i)<0
%                 flag(i)= region3(si_form(:,i), x_h);
%             end
%             if phone(j,k,i)==1
%                 [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             elseif phone(j,k,i)==0
%                 u(i)=0;
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             apf_obst2(j,k)= sum(u);
%             if apf_obst2(j,k)>1000
%                 apf_obst2(j,k)= 1000;
%             end
%         end
%         
%     end
% end
% figure(71)
% % subplot(2,1,1)
% view(3)
% hold on;
% s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 7');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% zlabel('Cost of the Potential Function')
% title('Active APF (surface) when the HV is in Region 7')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% % legend(s, 'Potential For Region 7')
% figure(72)
% view(3)
% hold on;
% c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% hold off;
% shading interp;
% xlabel('Longitudinal Distan=ce')
% ylabel('Lateral Distance')
% title('Active APF (contour) when the HV is in Region 7')
% legend
% xlim([220 245]);
% ylim([0 6]);
% toc

%% For region 8
x_h_0= [0 233.49 0 0 0 0]';
Y_var= linspace(0, 6, data_count);
X_var= linspace(220, 245, data_count);
for i= 1:1:width(x_o_0)
    if x_o_0(6,i)>0
        flag(1,i)= region1(si_form(:,i), x_h_0);
    elseif x_o_0(6,i)==0
        flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
    elseif x_o_0(6,i)<0
        flag(1,i)= region3(si_form(:,i), x_h_0);
    end
end
count1=zeros(1,width(x_o_0));
count2=zeros(1,width(x_o_0));
X_vartempin= zeros(1,0);
Y_vartempin= zeros(1,0);
X_vartempout= zeros(1,0);
Y_vartempout= zeros(1,0);
phone= zeros(1,0,0);
for j= 1:length(Y_var)
    for k=1:length(X_var)
        x_h(2,1)= X_var(k);
        x_h(3,1)= Y_var(j);
        for i= 1:1:width(x_o_0)
            if x_o_0(6,i)>0
                flag1(i)= region1(si_form(:,i), x_h);
                if flag1(1,i)==flag(1,i)
%                     count1= count1+1;
                    count1(1,i)= count1(1,i)+1;
                    X_vartempin(i,count1(1,i))= x_h(2,1);
                    Y_vartempin(i,count1(1,i))= x_h(3,1);
                    phone(j,k,i)= 1;
                else
%                     count2= count2+1;
                    count2(1,i)= count2(1,i)+1;
                    X_vartempout(i,count2(1,i))= x_h(2,1);
                    Y_vartempout(i,count2(1,i))= x_h(3,1);
                    phone(j,k,i)= 0;
                end
            elseif x_o_0(6,i)==0
                flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
                if flag1(1,i)==flag(1,i)
%                     count1= count1+1;
                    count1(1,i)= count1(1,i)+1;
                    X_vartempin(i,count1(1,i))= x_h(2,1);
                    Y_vartempin(i,count1(1,i))= x_h(3,1);
                    phone(j,k,i)= 1;
                else
%                     count2= count2+1;
                    count2(1,i)= count2(1,i)+1;
                    X_vartempout(i,count2(1,i))= x_h(2,1);
                    Y_vartempout(i,count2(1,i))= x_h(3,1);
                    phone(j,k,i)= 0;
                end
            elseif x_o_0(6,i)<0
                flag1(i)= region3(si_form(:,i), x_h);
                if flag1(1,i)==flag(1,i)
%                     count1= count1+1;
                    count1(1,i)= count1(1,i)+1;
                    X_vartempin(i,count1(1,i))= x_h(2,1);
                    Y_vartempin(i,count1(1,i))= x_h(3,1);
                    phone(j,k,i)= 1;
                else
%                     count2= count2+1;
                    count2(1,i)= count2(1,i)+1;
                    X_vartempout(i,count2(1,i))= x_h(2,1);
                    Y_vartempout(i,count2(1,i))= x_h(3,1);
                    phone(j,k,i)= 0;
                end
            end
%             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
        end
%         apf_obst1(j,k)= sum(u);
    end
end
figure(8)
hold on;
plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 8')
plot(X_vartempout, Y_vartempout, 'b*',  'DisplayName','Rest of the Regions')
surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
xlabel('Longitudinal Distance')
ylabel('Lateral Distance')
title('Active Region when the HV is in Region 8')
legend;
xlim([220 245]);
ylim([0 6]);
hold off;
apf_obst2= zeros(1,0);
for j= 1:length(Y_var)
    for k= 1:1:length(X_var)
        x_h(2,1)= X_var(k);
        x_h(3,1)= Y_var(j);
        for i= 1:1:width(x_o_0)
            if x_o_0(6,i)>0
                flag(i)= region1(si_form(:,i), x_h);
            elseif x_o_0(6,i)==0
                flag(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
            elseif x_o_0(6,i)<0
                flag(i)= region3(si_form(:,i), x_h);
            end
            if phone(j,k,i)==1
                [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
            elseif phone(j,k,i)==0
                u(i)=0;
            end
%             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
            apf_obst2(j,k)= sum(u);
            if apf_obst2(j,k)>1000
                apf_obst2(j,k)= 1000;
            end
        end
        
    end
end
figure(81)
% subplot(2,1,1)
view(3)
hold on;
s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 8');
surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
hold off;
shading interp;
xlabel('Longitudinal Distance')
ylabel('Lateral Distance')
zlabel('Cost of the Potential Function')
title('Active APF (surface) when the HV is in Region 8')
legend;
xlim([220 245]);
ylim([0 6]);
% legend(s, 'Potential For Region 8')
figure(82)
view(3)
hold on;
c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
hold off;
shading interp;
xlabel('Longitudinal Distance')
ylabel('Lateral Distance')
title('Active APF (contour) when the HV is in Region 8')
legend
xlim([220 245]);
ylim([0 6]);
% toc

% %% For region 9
% x_h_0= [0 235 0 0 0 0]';
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 245, data_count);
% for i= 1:1:width(x_o_0)
%     if x_o_0(6,i)>0
%         flag(1,i)= region1(si_form(:,i), x_h_0);
%     elseif x_o_0(6,i)==0
%         flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
%     elseif x_o_0(6,i)<0
%         flag(1,i)= region3(si_form(:,i), x_h_0);
%     end
% end
% count1=zeros(1,width(x_o_0));
% count2=zeros(1,width(x_o_0));
% X_vartempin= zeros(1,0);
% Y_vartempin= zeros(1,0);
% X_vartempout= zeros(1,0);
% Y_vartempout= zeros(1,0);
% phone= zeros(1,0,0);
% for j= 1:length(Y_var)
%     for k=1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag1(i)= region1(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)==0
%                 flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)<0
%                 flag1(i)= region3(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%         end
% %         apf_obst1(j,k)= sum(u);
%     end
% end
% figure(9)
% hold on;
% plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 9')
% plot(X_vartempout, Y_vartempout, 'b*',  'DisplayName','Rest of the Regions')
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active Region when the HV is in Region 9')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% hold off;
% apf_obst2= zeros(1,0);
% for j= 1:length(Y_var)
%     for k= 1:1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag(i)= region1(si_form(:,i), x_h);
%             elseif x_o_0(6,i)==0
%                 flag(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%             elseif x_o_0(6,i)<0
%                 flag(i)= region3(si_form(:,i), x_h);
%             end
%             if phone(j,k,i)==1
%                 [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             elseif phone(j,k,i)==0
%                 u(i)=0;
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             apf_obst2(j,k)= sum(u);
%             if apf_obst2(j,k)>1000
%                 apf_obst2(j,k)= 1000;
%             end
%         end
%         
%     end
% end
% figure(91)
% % subplot(2,1,1)
% view(3)
% hold on;
% s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 9');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% zlabel('Cost of the Potential Function')
% title('Active APF (surface) when the HV is in Region 9')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% % legend(s, 'Potential For Region 9')
% figure(92)
% view(3)
% hold on;
% c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active APF (contour) when the HV is in Region 9')
% legend
% xlim([220 245]);
% ylim([0 6]);
% toc
% 
% %% For region 10
% x_h_0= [0 238 0 0 0 0]';
% Y_var= linspace(0, 6, data_count);
% X_var= linspace(220, 245, data_count);
% for i= 1:1:width(x_o_0)
%     if x_o_0(6,i)>0
%         flag(1,i)= region1(si_form(:,i), x_h_0);
%     elseif x_o_0(6,i)==0
%         flag(1,i)= region2(si_form(:,i), x_h_0, v_alp1{:,i});
%     elseif x_o_0(6,i)<0
%         flag(1,i)= region3(si_form(:,i), x_h_0);
%     end
% end
% count1=zeros(1,width(x_o_0));
% count2=zeros(1,width(x_o_0));
% X_vartempin= zeros(1,0);
% Y_vartempin= zeros(1,0);
% X_vartempout= zeros(1,0);
% Y_vartempout= zeros(1,0);
% phone= zeros(1,0,0);
% for j= 1:length(Y_var)
%     for k=1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag1(i)= region1(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)==0
%                 flag1(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             elseif x_o_0(6,i)<0
%                 flag1(i)= region3(si_form(:,i), x_h);
%                 if flag1(1,i)==flag(1,i)
% %                     count1= count1+1;
%                     count1(1,i)= count1(1,i)+1;
%                     X_vartempin(i,count1(1,i))= x_h(2,1);
%                     Y_vartempin(i,count1(1,i))= x_h(3,1);
%                     phone(j,k,i)= 1;
%                 else
% %                     count2= count2+1;
%                     count2(1,i)= count2(1,i)+1;
%                     X_vartempout(i,count2(1,i))= x_h(2,1);
%                     Y_vartempout(i,count2(1,i))= x_h(3,1);
%                     phone(j,k,i)= 0;
%                 end
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%         end
% %         apf_obst1(j,k)= sum(u);
%     end
% end
% figure(10)
% hold on;
% plot(X_vartempin, Y_vartempin, 'g*', 'DisplayName','Region 10')
% plot(X_vartempout, Y_vartempout, 'b*',  'DisplayName','Rest of the Regions')
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active Region when the HV is in Region 10')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% hold off;
% apf_obst2= zeros(1,0);
% for j= 1:length(Y_var)
%     for k= 1:1:length(X_var)
%         x_h(2,1)= X_var(k);
%         x_h(3,1)= Y_var(j);
%         for i= 1:1:width(x_o_0)
%             if x_o_0(6,i)>0
%                 flag(i)= region1(si_form(:,i), x_h);
%             elseif x_o_0(6,i)==0
%                 flag(i)= region2(si_form(:,i), x_h, v_alp1{:,i});
%             elseif x_o_0(6,i)<0
%                 flag(i)= region3(si_form(:,i), x_h);
%             end
%             if phone(j,k,i)==1
%                 [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             elseif phone(j,k,i)==0
%                 u(i)=0;
%             end
% %             [K(i), u(i), ux(i), uy(i), uxx(i), uyy(i), uyx(i), uxy(i)] = dist(flag(i), v_alp1{:,i}, x_h, const_o);
%             apf_obst2(j,k)= sum(u);
%             if apf_obst2(j,k)>1000
%                 apf_obst2(j,k)= 1000;
%             end
%         end
%         
%     end
% end
% figure(101)
% % subplot(2,1,1)
% view(3)
% hold on;
% s=surf(X_var,Y_var,apf_obst2, 'DisplayName','Potential For Region 10');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% zlabel('Cost of the Potential Function')
% title('Active APF (surface) when the HV is in Region 10')
% legend;
% xlim([220 245]);
% ylim([0 6]);
% % legend(s, 'Potential For Region 10')
% figure(102)
% view(3)
% hold on;
% c= contour(X_var,Y_var,apf_obst2,1000, 'DisplayName', 'Contour');
% surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, 0);
% hold off;
% shading interp;
% xlabel('Longitudinal Distance')
% ylabel('Lateral Distance')
% title('Active APF (contour) when the HV is in Region 10')
% legend
% xlim([220 245]);
% ylim([0 6]);
% toc

%% Functions

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

% The next two functions are used to generate the distance between points
% and a line segment(defined by two vertices)
function r=p_l_dist(line_p1, line_p2, p)
    vx = line_p1(1)-p(1);
    vy = line_p1(2)-p(2);
    ux = line_p2(1)-line_p1(1);
    uy = line_p2(2)-line_p1(2);
    lenSqr= (ux*ux+uy*uy);
    detP= -vx*ux + -vy*uy;

    if( detP < 0 )
        r = norm(line_p1-p,2);
    elseif( detP > lenSqr )
        r = norm(line_p2-p,2);
    else
        r = abs(ux*vy-uy*vx)/sqrt(lenSqr);
    end
%     A = p(1)-line_p1(1);
%     B = p(2)-line_p1(2);
%     C = line_p2(1)-line_p1(1);
%     D = line_p2(2)-line_p1(2);
%     lenSqr= (C*C+D*D);
%     dot= A*C+ B*D;
%     param= -1;
%     if (lenSqr~=0)
%         param= dot/lenSqr;
%     end
%     if( param<0 )
%         xx= line_p1(1);
%         yy= line_p1(2);
%     elseif( param>1 )
%         xx= line_p2(1);
%         yy= line_p2(2);
%     else
%         xx= line_p1(1)+ param*C;
%         yy= line_p1(2)+param*D;
%     end
%     r= sqrt((p(1)-xx)^2+(p(2)-yy)^2);
end

% Function to find the distance to all the boundaries and vertices of a
% rectangle from a point and finding the minimum distance

function [min_dist, d]= min_dist_p_rect(bound, p, tria_v)
% Things to do would be 1. Take the bound and form vertex pairs using line1. Use those
% vertex pairs along with p and use p_l_dist to find the distance to each
% of those lines. Now find the distance between the point and all the four
% vertices. Save all this data in a d variable and find the minimum of it. 
    [v_alp1 v_alp2]= vertex(bound, tria_v);
    for i= 1:1:width(bound)
        vset1= v_alp1{1,i};
        vset2= v_alp2{1,i};
        for j= 1:1:height(vset1)
            v1= vset1(j,:);
            v2= vset2(j,:);
            d(i,j)= p_l_dist(v1', v2', p');
        end
    end
    min_dist= min(d,[],2);
end

% Function to find the vertices of the rectangle given the size of the
% rectangle, the angle and the centre. 
function [points]= rect_p(size, data, select)
    l = size(1,1);
    b= size(1,2);
    ang1= atand(b/l);
    ang2= 2*ang1;
    ang3= 2*atand(l/b);
    h= sqrt(l^2+b^2)/2;
    for i=1:1:width(data)
        x= data(2,i);
        y= data(3,i);
        ang= data(6,i);
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
    points_mat= cell2mat(points);
end

% function to plot a rectangle given its vertices
function [x_vert_plot y_vert_plot]= rect_plot(vert, tria_v)
    for i=1:1:length(vert)
        v= tria_v{i};
        vertices= vert{i};
        x_vert= vertices(:,1); 
        y_vert= vertices(:,2);
        x_vert_plot{i}= [x_vert;x_vert(1,1)];
        y_vert_plot{i}= [y_vert;y_vert(1,1)];
        x_vert_plot{i}= [x_vert(1:3,1);v(1,1);x_vert(4,1);x_vert(1,1)];
        y_vert_plot{i}= [y_vert(1:3,1);v(1,2);y_vert(4,1);y_vert(1,1)];
        plot(x_vert_plot{i}, y_vert_plot{i});
        hold on
    end
%     plot(x_h_0(2,1),x_h_0(3,1),'Color','r', 'Marker', '*', 'DisplayName', 'Host Vehicle');
%     hold off;
%     legend;
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
function surfplot_rect(x_h_0, x_o_0, distance, size_veh, select1, alpha)
[bound_OV]= rect_p(size_veh, x_o_0, select1);
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
        view(3);
        plot3(x_vert_plot,y_vert_plot,z ,'DisplayName', name, 'Color', 'k');
        hold on;
        si= vertcat(si_form{:,i});
        m= si(:,1);
        b= si(:,2);
        if x_o_0(6,i)>0
            plot3([vertices1(2,1) vertices1(2,1)-0.04], [vertices2(2,1) m(1,1)*(vertices1(2,1)-0.04)+b(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L1'); %L1 upwards
            plot3([vertices1(1,1) vertices1(1,1)+0.04], [vertices2(1,1) m(1,1)*(vertices1(1,1)+0.04)+b(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L1'); %L1 downwards
            plot3([vertices1(2,1) vertices1(2,1)+0.2], [vertices2(2,1) m(2,1)*(vertices1(2,1)+0.2)+b(2,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L2'); %L2
            plot3([vertices1(1,1) vertices1(1,1)+0.2], [vertices2(1,1) m(5,1)*(vertices1(1,1)+0.2)+b(5,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L5'); %L5
            plot3([vertices1(3,1) vertices1(3,1)-0.04], [vertices2(3,1) m(6,1)*(vertices1(3,1)-0.04)+b(6,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L6'); %L6 upwards
            plot3([vertices1(5,1) vertices1(5,1)+0.04], [vertices2(5,1) m(6,1)*(vertices1(5,1)+0.04)+b(6,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L6'); %L6 downwards
            plot3([vertices1(3,1) vertices1(3,1)-0.1], [vertices2(3,1) m(7,1)*(vertices1(3,1)-0.1)+b(7,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L7'); %L7
            plot3([vertices1(4,1) vertices1(4,1)-0.1], [vertices2(4,1) m(8,1)*(vertices1(4,1)-0.1)+b(8,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L8'); %L8
            plot3([vertices1(4,1) vertices1(4,1)-0.1], [vertices2(4,1) m(9,1)*(vertices1(4,1)-0.1)+b(9,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L9'); %L9
            plot3([vertices1(5,1) vertices1(5,1)-0.1], [vertices2(5,1) m(10,1)*(vertices1(5,1)-0.1)+b(10,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L10'); %L10
        elseif x_o_0(6,i)==0
            plot3([vertices1(2,1) vertices1(2,1)], [vertices2(2,1) vertices2(2,1)+1], [alpha alpha], 'Color', 'r','DisplayName', 'L1','HandleVisibility','off'); %L1
            text(vertices1(2,1),vertices2(2,1)+1, alpha, 'L_1', 'Color', 'r');
            
            plot3([vertices1(1,1) vertices1(1,1)], [vertices2(1,1) vertices2(1,1)-1], [alpha alpha], 'Color', 'r','DisplayName', 'L1','HandleVisibility','off'); %L1
            text(vertices1(1,1),vertices2(1,1)-1, alpha, 'L_1', 'Color', 'r');
            
            plot3([vertices1(2,1) vertices1(2,1)+1], [vertices2(2,1) vertices2(2,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L2','HandleVisibility','off'); %L2
            text(vertices1(2,1)+1,vertices2(2,1), alpha, 'L_2', 'Color', 'r');
            
            plot3([vertices1(1,1) vertices1(1,1)+1], [vertices2(1,1) vertices2(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L5','HandleVisibility','off'); %L5
            text(vertices1(1,1)+1,vertices2(1,1), alpha, 'L_5', 'Color', 'r');
            
            plot3([vertices1(3,1) vertices1(3,1)], [vertices2(3,1) vertices2(3,1)+1], [alpha alpha], 'Color', 'r','DisplayName', 'L6','HandleVisibility','off'); %L6
            text(vertices1(3,1), vertices2(3,1)+1, alpha, 'L_6', 'Color', 'r');
            
            plot3([vertices1(5,1) vertices1(5,1)], [vertices2(5,1) vertices2(5,1)-1], [alpha alpha], 'Color', 'r','DisplayName', 'L6','HandleVisibility','off'); %L6
            text(vertices1(5,1), vertices2(5,1)-1, alpha, 'L_6', 'Color', 'r');
            
            plot3([vertices1(3,1) vertices1(3,1)-1], [vertices2(3,1) m(7,1)*(vertices1(3,1)-1)+b(7,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L7','HandleVisibility','off'); %L7
            text(vertices1(3,1)-0.25,m(7,1)*(vertices1(3,1)-0.25)+b(7,1), alpha, 'L_7', 'Color', 'r');
            
            plot3([vertices1(4,1) vertices1(4,1)-1], [vertices2(4,1) m(8,1)*(vertices1(4,1)-1)+b(8,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L8','HandleVisibility','off'); %L8
            text(vertices1(4,1)-0.40, m(8,1)*(vertices1(4,1)-0.40)+b(8,1), alpha, 'L_8', 'Color', 'r');
            
            plot3([vertices1(4,1) vertices1(4,1)-1], [vertices2(4,1) m(9,1)*(vertices1(4,1)-1)+b(9,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L9','HandleVisibility','off'); %L9
            text(vertices1(4,1)-0.40, m(9,1)*(vertices1(4,1)-0.40)+b(9,1), alpha, 'L_9', 'Color', 'r');
            
            plot3([vertices1(5,1) vertices1(5,1)-1], [vertices2(5,1) m(10,1)*(vertices1(5,1)-1)+b(10,1)], [alpha alpha], 'Color','r','DisplayName', 'L10','HandleVisibility','off'); %L10
            text(vertices1(5,1)-0.25, m(10,1)*(vertices1(5,1)-0.25)+b(10,1), alpha, 'L_{10}', 'Color', 'r');
            
        elseif x_o_0(6,i)<0
            plot3([vertices1(2,1) vertices1(2,1)+0.01], [vertices2(2,1) m(1,1)*(vertices1(2,1)+0.01)+b(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L1'); %L1
            plot3([vertices1(1,1) vertices1(1,1)-0.01], [vertices2(1,1) m(1,1)*(vertices1(1,1)-0.01)+b(1,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L1'); %L1
            plot3([vertices1(2,1) vertices1(2,1)+0.1], [vertices2(2,1) m(2,1)*(vertices1(2,1)+0.1)+b(2,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L2'); %L2
            plot3([vertices1(1,1) vertices1(1,1)+0.1], [vertices2(1,1) m(5,1)*(vertices1(1,1)+0.1)+b(5,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L5'); %L5
            plot3([vertices1(3,1) vertices1(3,1)+0.01], [vertices2(3,1) m(6,1)*(vertices1(3,1)+0.01)+b(6,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L6'); %L6
            plot3([vertices1(5,1) vertices1(5,1)-0.01], [vertices2(5,1) m(6,1)*(vertices1(5,1)-0.01)+b(6,1)], [alpha alpha], 'Color', 'r','DisplayName', 'L6'); %L6
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

%% Functions (Symbolic)

% Design an obstacle APF for multiple obstacles. Input for this is the
% constant, the states of the vehicle as well as the 
function [apf_obst_fin apf_road_fin apf_lane_fin]= U_tot(const_o, const_r, x_o, x_h)
    syms apf_obst_fin(X,Y)
    syms apf_lane_fin(X,Y)
    syms apf_road_fin(X,Y)
    syms apf_fin(X,Y)
    syms t1(X,Y)
    syms t2(X,Y)
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
%% Obstacle Potential    
    for i=1:col
        psi= x_o(6,i);
        o_x= x_o(2,i);
        o_y= x_o(3,i);
        a_psi(i)= ((cosd(psi)^2)/(2*sigma_x^2))  +   (sind(psi)^2)/(2*sigma_y^2);
        b_psi(i)= -((sind(2*psi)^2)/(4*sigma_y^2) - (sind(2*psi)^2)/(4*sigma_x^2));
        c_psi(i)= (sind(psi)^2)/(2*sigma_x^2)  +   (cosd(psi)^2)/(2*sigma_y^2);
        temp1(i)= a_psi(i)*(X-o_x)^2;
        temp2(i)= 2*b_psi(i)*(X- o_x)* (Y- o_y);
        temp3(i)= c_psi(i)*(Y- o_y)^2;
        temp4(i)= -(temp1(i)+temp2(i)+temp3(i));
        apf_obst(i)= A_o*exp(temp4(i));
    end
    apf_obst_fin(X,Y)= sum(apf_obst);
    vpa_obst= vpa(simplify(expand(apf_obst_fin)),5);
%% Lane + Road Potential
% Lane Potential
    for i= 1:width(all_bound)
        apf_lane(i)= A_lane* exp(-((Y- all_bound(i))^2/(2* sig_lane^2)));
    end
    for i= 1:width(loc_road_bound)
        apf_lane1(i)= A_lane1* exp(-((Y- loc_road_bound(i))^2/(2* sig_lane1^2)));
    end
    apf_lane_fin(X,Y)= sum(apf_lane)+ sum(apf_lane1);
    vpa_lane= vpa(simplify(expand(apf_lane_fin)),5);
% Road Potential
    for i= 1:length(loc_road_bound)
        apf_road(i)= 0.5* eta * (1/(Y- loc_road_bound(i)))^2;
    end
    apf_road_fin(X,Y)= sum(apf_road);  
    vpa_road= vpa(simplify(expand(apf_road_fin)),5);
%% Total Potential 
%     apf_fin(X,Y)= 10*apf_obst_fin + apf_road_fin; % Here the lane 
%     % potential is not included as it leads to weird kinds of taylor
%     % approximations. 
%     fin1= vpa(simplify(expand(apf_fin)),5);
% %% Taylor Series for Testing
%     t1(X,Y)= taylor(fin1,[X, Y],[x_h(2,1) x_h(3,1)] ,'Order', 3); % Cannot use this here
%     % as it has to be included in the cost function as a function of the
%     % input which can be only done when I rewrite the whole system as a
%     % function of the input
%     simp_t1= vpa(simplify(expand(t1)),5);
end

% Taylor series approximation of the the obstacle APF to be done at a
% point. This function is used to get the second order function by using
% the differentiation of the APF function. The input is the function and
% then output is three matrices which together can generate the taylor
% series
function [A1, A2, A3]= taylor_2order(C, x_h)
syms x y
% syms a1temp(x,y) a2temp(x,y) a3temp(x,y) a4temp(x,y) a5temp(x,y) a6temp(x,y) a7temp(x,y)
x_c= x_h(1,1);
y_c= x_h(1,2);
a1temp= C;
a1= double(a1temp(x_c, y_c));      % U(Xh(k), Yh(k))

a2temp= diff(C,x);          % U_Xh
a2= double(a2temp(x_c, y_c));

a3temp= diff(C,y);          % U_Yh
a3= double(a3temp(x_c, y_c));

a4temp= diff(diff(C,x),x);  % U_XhXh
a4= double(a4temp(x_c, y_c));

a5temp= diff(diff(C,x),y);  % U_XhYh
a5= double(a5temp(x_c, y_c));

a6temp= diff(diff(C,y),x);  % U_YhXh
a6= double(a6temp(x_c, y_c));

a7temp= diff(diff(C,y),y);  % U_YhYh
a7= double(a7temp(x_c, y_c));

A1= a1 - a2*x_c - a3*y_c + 0.5*a4*x_c^2 + a5*x_c*y_c + 0.5*a7*y_c^2;

A2= [ a2 - a4*x_c - a5*y_c; 
     a3 - a5*x_c - a7*y_c]';
 
A3= [a4 a5;
    a6 a7];

% t3(x,y)= A1 + A2*[x;y] + 0.5*[x y]*A3*[x;y];
% t4= vpa(t3,5);
end

% Design an obstacle APF for multiple obstacles. Input for this is the
% constant, the states of the vehicle as well as the 
function [apf_lane_fin,apf_road_fin,apf_obst_fin]= U(const_o, const_r, x_o_0, x_h_0, rl, distance, size_veh)
    syms apf_obst_fin(x,y)
    syms apf_lane_fin(x,y)
    syms apf_road_fin(x,y)
    syms apf_fin(x,y)
    syms t1(x,y)
    syms t2(x,y)
    A0= const_o(1,1);
    b0= const_o(1,2);
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
    [row, col]=size(x_o_0);
    
    if rl=='r'
        select1= 1;
    elseif rl== 'l'
        select1= 2;
    else
        display('Wrong Input')
    end
    
    [bound_OV]= rect_p(size_veh, x_o_0, select1);
    [tria_v]= trian(bound_OV, distance, x_o_0);
    [v_alp1 v_alp2]= vertex(bound_OV, tria_v);
    [si_form]= equations(v_alp1, v_alp2, x_o_0);
    apf_obst_fin(x,y)=0;
%% Obstacle Potential    
    for i=1:col
        if x_o_0(6,i)>0
            flag1= region1(si_form(:,i), x_h_0);
        elseif x_o_0(6,i)==0
            flag1= region2(si_form(:,i), x_h_0, v_alp1{:,i});
        elseif x_o_0(6,i)<0
            flag1= region3(si_form(:,i), x_h_0);
        end
        v= v_alp1{:,i};
        switch flag1
            case 1
                p1= v(1,:);
                p2= v(2,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= alpha/beta;               
                xx= x1 + (gamma*(x2-x1));
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 2
                p1= v(2,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end                
            case 3
                p1= v(2,:);
                p2= v(3,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= vpa(alpha/beta,5);               
                xx= vpa(x1 + (gamma*(x2-x1)),5);
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 4
                p1= v(3,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end
            case 5
                p1= v(3,:);
                p2= v(4,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= vpa(alpha/beta,5);               
                xx= vpa(x1 + (gamma*(x2-x1)),5);
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 6
                p1= v(4,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end
            case 7
                p1= v(4,:);
                p2= v(5,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= vpa(alpha/beta,5);               
                xx= vpa(x1 + (gamma*(x2-x1)),5);
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 8
                p1= v(5,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end
            case 9 
                p1= v(5,:);
                p2= v(1,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= vpa(alpha/beta,5);               
                xx= vpa(x1 + (gamma*(x2-x1)),5);
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 10
                p1= v(1,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end
            case 11
                display('Point Inside the Car');
                K= 2e-3;
        end
%             apf_obst= (A0*exp(-b0*K))/K;
            apf_obst= -A0*log(K);
%             apf_obst= -A0*(log(K)/log(b0));
%             apf_obst= (A0*exp(-b0*K));
%         
        apf_obst_fin= apf_obst_fin+apf_obst;
    end

%% Road + Lane Potential
    for i= 1:width(all_bound)
        apf_lane(i)= A_lane* exp(-((x- all_bound(i))^2/(2* sig_lane^2)));
    end
    apf_lane_fin(x,y)= sum(apf_lane);
    for i= 1:length(loc_road_bound)
        apf_road(i)= 0.5* eta * (1/(y- loc_road_bound(i)))^2;
    end
    apf_road_fin(x,y)= sum(apf_road);  
    
end

function [apf_obst_fin]= U_obst(const_o, x_o_0, x_h_0, select, distance, size_veh)
% function [apf_obst_fin]= U_tot(const_o, x_o_0, x_h_0, select, distance, size_veh)
    syms apf_obst_fin(x,y)
    syms t1(x,y)
    syms t2(x,y)
    syms t3_fin(x,y)
    A0= const_o(1,1);
    b0= const_o(1,2);
    [row, col]=size(x_o_0);
    [bound_OV]= rect_p(size_veh, x_o_0, select);
    [tria_v]= trian(bound_OV, distance, x_o_0);
    [v_alp1 v_alp2]= vertex(bound_OV, tria_v);
    [si_form]= equations(v_alp1, v_alp2, x_o_0);
    apf_obst_fin(x,y)=0;
%% Obstacle Potential    
    for i=1:col
        if x_o_0(6,i)>0
            flag1= region1(si_form(:,i), x_h_0);
        elseif x_o_0(6,i)==0
            flag1= region2(si_form(:,i), x_h_0, v_alp1{:,i});
        elseif x_o_0(6,i)<0
            flag1= region3(si_form(:,i), x_h_0);
        end
        v= v_alp1{:,i};
        switch flag1
            case 1
                p1= v(1,:);
                p2= v(2,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= alpha/beta;               
                xx= x1 + (gamma*(x2-x1));
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 2
                p1= v(2,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end               
            case 3
                p1= v(2,:);
                p2= v(3,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= vpa(alpha/beta,5);               
                xx= vpa(x1 + (gamma*(x2-x1)),5);
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 4
                p1= v(3,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end
            case 5
                p1= v(3,:);
                p2= v(4,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= vpa(alpha/beta,5);               
                xx= vpa(x1 + (gamma*(x2-x1)),5);
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 6
                p1= v(4,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end
            case 7
                p1= v(4,:);
                p2= v(5,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= vpa(alpha/beta,5);               
                xx= vpa(x1 + (gamma*(x2-x1)),5);
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 8
                p1= v(5,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end
            case 9 
                p1= v(5,:);
                p2= v(1,:);
                x1= p1(1,1);
                y1= p1(1,2);
                x2= p2(1,1);
                y2= p2(1,2);
                alpha= vpa((x-x1) * (x2-x1)+ (y-y1) * (y2-y1),5);
                beta= (y2-y1)^2 + (x2-x1)^2;
                gamma= vpa(alpha/beta,5);               
                xx= vpa(x1 + (gamma*(x2-x1)),5);
                yy= vpa(y1 + (gamma*((y2-y1))),5);
                K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
            case 10
                p1= v(1,:);
                x0= p1(1,1);
                y0= p1(1,2);
                x1= x_h_0(2,1);
                y1= x_h_0(3,1);
                m= -(x1-x0)/(y1-y0);
                if isinf(m)==0
                    if m==0
                        yy= y0;
                        xx= x;
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    elseif m~=0
                        b= y0-m*x0;
                        xx= ((x+m*y-m*b)/(m^2+1));
                        yy= ((m*(m*y+x)+b)/(m^2+1));
                        K = sqrt((x-xx)^2+ (y-yy)^2);
                    end
                elseif isinf(m)==1
                    xx= x0;
                    yy= y;
                    K = vpa(sqrt((x-xx)^2+ (y-yy)^2),5);
                end
            case 11
                display('Point Inside the Car');
                m='Flag=11';
                K= 2e-3;
        end
%             apf_obst= (A0*exp(-b0*K))/K;
            apf_obst= -(A0*log(b0*K))/(b0*K);
        apf_obst_fin= apf_obst_fin+apf_obst;
    end
end