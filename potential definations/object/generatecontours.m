clc;
clear all;
Y_var= linspace(0, 6, 300);
X_var= linspace(220, 238, 300);
S1= load('apf_data_yukawa_300.mat');
S2= load('apf_data_modyukawa_300.mat');
figure(1)
s=surf(X_var,Y_var,S1.apf_obst1);
hold on;
s.EdgeColor = 'none';
s.FaceColor = 'interp';
% shading interp;
hold on;
plot3(X_var(:,1), Y_var(:,1), S1.apf_obst1(:,1),'k');
spacing = 4;  % play around so it fits the size of your data set
for i = 1 : spacing : length(X_var)
    plot3(X_var(:,i), Y_var(:,i), S1.apf_obst1(:,i),'-k');
    plot3(X_var(:,i), Y_var(:,i), S1.apf_obst1(:,i),'-k');
end
xlabel('Horizontal Distance')
ylabel('Vertical Distance')
zlabel('Cost of the Potential Function')
legend(s, 'Cost around the Obstacle')
legend
figure(2)
contour(X_var,Y_var,S1apf_obst1, 10000);
xlabel('Horizontal Distance')
ylabel('Vertical Distance')
zlabel('Cost of the Potential Function')
legend(s, 'Cost around the Obstacle')
legend