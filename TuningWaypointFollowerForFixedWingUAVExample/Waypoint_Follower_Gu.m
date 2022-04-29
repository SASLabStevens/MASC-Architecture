function [status, distance, DesiredHeading] = Waypoint_Follower_Gu(FixedWingStateBus,ConfigureStatus,state)

%% before running this planning algorithm, You are suppose to test trjectory following algorithm at three different status invidsually so that adjust parameters in while condition.

%% INITIALIZED PARAMETERS
DesiredHeading=0;  
% initialize some variable to avoid the bug not fully defined on execuation
% path                    
% Current and Final states %
xi     =   FixedWingStateBus.North;          % Current x-axis position (ft)
yi     =   FixedWingStateBus.East;          % Current y-axis position (ft)
zi     =   FixedWingStateBus.Height;          % Current z-axis position (ft)
%Vi     =   FlightState.V;          % Current forward velocity (ft/s)
gammai =   FixedWingStateBus.FlightPathAngle;          % Current flight path angle (rad)
psii   =   atan2(sin(FixedWingStateBus.HeadingAngle) , cos(FixedWingStateBus.HeadingAngle));          % Current heading angle phi (rad)
betai  =   FixedWingStateBus.AngleOfSideslip;          % Current sideslip angle (rad)

xf     =   ConfigureStatus.x_final;          % Final x-axis position (ft)
yf     =   ConfigureStatus.y_final;          % Final y-axis position (ft)
zf     =   ConfigureStatus.z_final;         % Final z-axis position (ft)
Vf     =   ConfigureStatus.V_final;         % Final forward velocity (ft/s)
gammaf =   ConfigureStatus.FlightPathAngle_final;         % Final flight path angle (rad)
psif   =   atan2(sin(ConfigureStatus.HeadingAngle_final) , cos(ConfigureStatus.HeadingAngle_final));         % Final heading angle phi (rad)
betaf  =   ConfigureStatus.SideslipAngle_final;         % Final sideslip angle (rad)

% Positon Where engine is broken
xb     =   ConfigureStatus.x_turnoff;
yb     =   ConfigureStatus.y_turnoff;
zb     =   ConfigureStatus.z_turnoff; 
psib   =   atan2(sin(ConfigureStatus.HeadingAngle_turnoff*pi/180) , cos(ConfigureStatus.HeadingAngle_turnoff*pi/180)); % convert range (0~360) to (-pi~pi)
% Mission phase
status =   state;         % Mission phase

%% Motion primitive 1: maintain the current flight path angle and heading angle
%{
State      =   zeros(1,21);
State(1)   =   xi;    % Update xi
State(2)   =   yi;    % Update yi
State(3)   =   zi;    % Update zi
State(4)   =   Vi;    % Update Vi
State(5)   =   gammai;% Update gammai
State(6)   =   atan2(sin(psii) , cos(psii)); % convert range (0~360) to (-180~180)
State(7)   =   betai; % Update betai
State(8:21)=   [xf, yf, zf, Vf, gammaf, psif, betaf, xo1, yo1, Ro1, xb, yb, zb , status];

[status, distance, DesiredHeading, DesiredFlightPath] = EvaluateCost_Gu(State);
%}
    %%
    Rl         =   3333;                                  % Loiter radius (ft)
    xl         =   xf + 4 * Rl * cos(psif - pi);          % Shifting the x-axis center of loitering 
    yl         =   yf + 4 * Rl * sin(psif - pi);          % Shifting the x-axis center of loitering
    xu         =   xl + Rl * cos(psif - pi);
    yu         =   yl + Rl * sin(psif - pi);
    
    distance   =   sqrt((xi-xl)^2+(yi-yl)^2);             % Distance to loitering center
    % Call function for conducting planned path
   
    if(distance > 3538)    %  Cruise mode: Far away from landing site %% if condition has problem
        
        status = 1;
           
%% Parameters for Straight Line Chasing
delta = 1000; %distance of the look ahead point
W_i = [xb-xl yb-yl];%straight line origin
W_ip1 = [0 0];  %straight line destination
%^^^^^^^^^^^^^^^^^^^^^Update UAV movements^^^^^^^^^^^^^^^^^^^^^^^^^
P_new = [xi-xl yi-xl zi];
psi_new = psii;
%% consider to replace P_new with a real time position
if(P_new(1)>W_ip1(1) && P_new(2)>W_ip1(2))
% while condition is up to broken position and final landing position, before defining it, do test program first.

path_g = abs (((P_new(1) - W_i(1))^2) + ((P_new(2) - W_i(2))^2)) ^(1/2); %Calculation of UAV distance from origin
theta = atan2((W_ip1(2)-W_i(2)),(W_ip1(1)-W_i(1))); %path angle calculation
thetau = atan2((P_new(2)-W_i(2)),P_new(1)-W_i(1));  %UAV path angle from origin

l_d=theta-thetau;
Ru=((path_g^2)-((path_g*sin(l_d))^2))^(1/2);
%^^^^^^^^^^^^^^^^^^^^^^Definition of the look ahead point^^^^^^^^^^^^^^^^^^
x_i = ((Ru)+delta)*(cos(theta));
y_i = ((Ru)+delta)*(sin(theta));
p_i = [x_i y_i];
psi_d = atan2((y_i-(P_new(2))),(x_i-(P_new(1)))); %calculation of desired heading angle
if psi_d >=0
    DesiredHeading = psi_d;
elseif psi_d < 0
    DesiredHeading = psi_d+2*pi;
end
u = (psi_d-psi_new); %controller input for changing heading angle
end
                     
    elseif(zi>1000)      % Loiter mode: Close to the landing site, but a high altitude
       
       status = 2;     
%% Parameters for loiter Chasing %%
r = Rl; %radius of loiter curve
O = [xl yl]; %center of loiter or circular orbit
delta = 0.2; %look ahead position
%^^^^^^^^^^^^^^^^^^^^^Update UAV movements^^^^^^^^^^^^^^^^^^^^^^^^^
P_new = [xi yi zi];
psi_new = psii;
       % Carrot-chasing algorithm for loitering       
       % [DesiredHeading,DesiredFlightPath] = Carrot_Loiter_Path(curr_x,curr_y,curr_z, psi_start, xl, yl, Rl, psi, xi, yi, zi);
if (P_new(3) >1000)
Ru = abs ((((P_new(1) - O(1))^2) + ((P_new(2) - O(2))^2)) ^(1/2)-r);%Calculation of UAV distance from center
theta_new = atan2((P_new(2)-O(2)),(P_new(1)-O(1))); %new path angle calculation
x_i = ((r*(cos(theta_new+delta)))+O(1));
y_i = ((r*(sin(theta_new+delta)))+O(2));
psi_d = atan2((y_i-P_new(2)),(x_i-P_new(1)));%calculation of desired heading angle
if psi_d >=0
    DesiredHeading = psi_d;
elseif psi_d < 0
    DesiredHeading = psi_d + 2*pi;
end
u = wrapToPi(psi_d-psi_new); %controller input for changing heading angle
end

    else           % Approach to the landing strip
     
       status = 3;              
%% Parameters for Straight Line Chasing
delta = 1000; %distance of the look ahead point
W_i = [0 0]; %straight line origin
W_ip1 = [xf-xu yf-yu]; %straight line destination
%^^^^^^^^^^^^^^^^^^^^^Update UAV movements^^^^^^^^^^^^^^^^^^^^^^^^^
P_new = [xi-xu yi-yu zi];
psi_new = psii;
   
%% consider to replace P_new with a real time position
if (P_new(1)<W_ip1(1) && P_new(2)<W_ip1(2))
path_g = abs (((P_new(1) - W_i(1))^2) + ((P_new(2) - W_i(2))^2)) ^(1/2); %Calculation of UAV distance from origin
theta = atan2((W_ip1(2)-W_i(2)),(W_ip1(1)-W_i(1))); %path angle calculation
thetau = atan2((P_new(2)-W_i(2)),P_new(1)-W_i(1));%UAV path angle from origin

l_d=theta-thetau;
Ru=((path_g^2)-((path_g*sin(l_d))^2))^(1/2);
%^^^^^^^^^^^^^^^^^^^^^^Definition of the look ahead point^^^^^^^^^^^^^^^^^^
x_i = ((Ru)+delta)*(cos(theta));
y_i = ((Ru)+delta)*(sin(theta));
p_i = [x_i y_i];
psi_d = atan2((y_i-(P_new(2))),(x_i-(P_new(1)))); %calculation of desired heading angle
if psi_d >=0
    DesiredHeading = psi_d;
elseif psi_d < 0
    DesiredHeading = psi_d+2*pi;
end
u = (psi_d-psi_new); %controller input for changing heading angle

end
    end

end