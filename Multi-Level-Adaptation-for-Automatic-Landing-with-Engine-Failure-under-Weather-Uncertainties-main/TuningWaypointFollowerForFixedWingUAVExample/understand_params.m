%% This function is for testing for converge to planned straight line
xb = 18672;
yb = -17316;
xf = 21822;
yf = -9751.8;
Rl = 1016;
psif = atan2(sin(24.17*pi/180) , cos(24.17*pi/180));
xl         =   xf + 7 * Rl * cos(psif - pi);
yl         =   yf + 7 * Rl * sin(psif - pi);
xu = xl + Rl * cos(psif - pi);
yu = yl + Rl * sin(psif - pi);
dInput = [xf,yf;
          xl,yl;
          xu,yu;
          xb,yb];
      fh = figure;
ah = axes(fh);
hold(ah,'on');
plot(ah,dInput(:,1),dInput(:,2),'*')
hold on
p = nsidedpoly(1000, 'Center', [xl yl], 'Radius', 1016);
plot(p, 'FaceColor', 'r')
axis equal
hold on
%Ru         =   sqrt((xl + Rl * cos(psif - pi) - xi)^2 + (yl + Rl * sin(psif - pi) - yi)^2);
%thetau     =   atan2( yi - yl - Rl * sin(psif - pi), xi - xl - Rl * cos(psif - pi));
%% Initialization of params
waypoints = [[xu, yu, 0]; [xf, yf, 0]; [150, 600, 0];[-300,600,0]; [0, 0, 0]; [300,0,0]; [0, 0, 0];];

wp_1 = waypoints(1,:);
w1_x = wp_1(1); w1_y = wp_1(2);

wp_2 = waypoints(2,:);
w2_x = wp_2(1); w2_y = wp_2(2);

lw = 1;
tspan = 0:0.1:500;
%% Straight Line Initial Condition
ao = 78.2;
curr_x = xl + 3333*cos(ao*pi/180);
curr_y = yl + 3333*sin(ao*pi/180);
%curr_x = 113778;
%curr_y = 67988.5;
curr_si= 0.7;
delta = 833;
fprintf("Plotting trajectories\n");
arr_x = [xu xf];  
arr_y = [yu yf];
plot(arr_x, arr_y,'-d');
hold on
string_x = [xl xb];
string_y = [yl yb];
plot(string_x, string_y,'-d');
%axis([0 60 0 60]);
%%
a = sqrt((xb-xl)^2+(yb-yl)^2);
angle_to_goal = atan2(yb-yl,xb-xl);