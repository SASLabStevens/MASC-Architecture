
function [status, distance, distancef, DesiredHeading,Cross_Tracking_Error] = Waypoint_Follower_Test(FixedWingStateBus,ConfigureStatus,state)

%% before running this planning algorithm, You are suppose to test trjectory following algorithm at three different status invidsually so that adjust parameters in while condition.

    % initialize some variable to avoid the bug not fully defined on execuation
    % path   
    DesiredHeading = 0;
    Cross_Tracking_Error = 0;
    % Current and Final states %
    xi     =   FixedWingStateBus.North;          % Current x-axis position (m)
    yi     =   FixedWingStateBus.East;          % Current y-axis position (m)
    zi     =   FixedWingStateBus.Height;          % Current z-axis position (m)
    %Vi     =   FlightState.V;          % Current forward velocity (m/s)
    gammai =   FixedWingStateBus.FlightPathAngle;          % Current flight path angle (rad)
    psii   =   atan2(sin(FixedWingStateBus.HeadingAngle) , cos(FixedWingStateBus.HeadingAngle));          % Current heading angle phi (rad)
    betai  =   0;          % Current sideslip angle (rad)

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
    psib   =   atan2(sin(ConfigureStatus.HeadingAngle_turnoff) , cos(ConfigureStatus.HeadingAngle_turnoff)); % convert range (0~360) to (-pi~pi)
    % Mission phase
    status =   state;         % Mission phase

    %%
    Rl         =   1016;                                  % Loiter radius (m)
    xl         =   xf + 7 * Rl * cos(psif - pi);          % Shifting the x-axis center of loitering 
    yl         =   yf + 7 * Rl * sin(psif - pi);          % Shifting the x-axis center of loitering
    xu         =   xl + Rl * cos(psif - pi);
    yu         =   yl + Rl * sin(psif - pi);
    
    distance   =   sqrt((xi-xl)^2+(yi-yl)^2);             % Distance to loitering center
    % Call function for conducting planned path
    distancef  =   sqrt((xi-xf)^2+(yi-yf)^2);
    za = 1000;
    %{
    count_A = 0;
    count_C = 0;
    count_B = 0;
    CurrX1 = 0;
    CurrY1 = 0;
    CurrZ1 = 0;
    CurrX2 = 0;
    CurrY2 = 0;
    CurrZ2 = 0;
    CurrX3 = 0;
    CurrY3 = 0;
    CurrZ3 = 0;
    %}
    if(distance > 2000 && zi>za)    %  Cruise mode: Far away from landing site %% if condition has problem
        
        status = 1;
        %{
        count_A = count_A +1; 
        if(count_A <=1)
            CurrX1 = xi;
            CurrY1 = yi;
            CurrZ1 = zi;
        end
        %}
%% Parameters for Straight Line Chasing
delta = 1000; %distance of the look ahead point
W_i = [0 0];%straight line origin
W_ip1 = [(xl-xb) (yl-yb)];  %straight line destination
%^^^^^^^^^^^^^^^^^^^^^Update UAV movements^^^^^^^^^^^^^^^^^^^^^^^^^
P_new = [(xi-xb) (yi-yb) zi];
psi_new = psii;
%% consider to replace P_new with a real time position
%if(P_new(1)>W_ip1(1) && P_new(2)>W_ip1(2))
if(distance>2000 && zi>za)
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
   DesiredHeading = -psi_d-pi;
elseif psi_d < 0
   DesiredHeading = -(psi_d + 2*pi)-pi;
end

Cross_Tracking_Error = (abs(Ru^2 - path_g^2))^(1/2);

end
%}                     


    elseif(distance<2000 && zi>za)      % Loiter mode: Close to the landing site, but a high altitude
       
       status = 2; 
       %{
       count_B = count_B +1; 
        if(count_B <=1)
            CurrX2 = xi;
            CurrY2 = yi;
            CurrZ2 = zi;
        end
       %}
%% Parameters for loiter Chasing %%
    r = 1016; %radius of loiter curve(for test need we customize the radius, the actual value is loiter radius)
    %O = [xb 1500+yb]; %center of loiter or circular orbit(for test need we customize the loiter center, the actual loiter center is loiter center calculated before)
    O = [xl yl];
    delta = 0.3; %look ahead position
%^^^^^^^^^^^^^^^^^^^^^Update UAV movements^^^^^^^^^^^^^^^^^^^^^^^^^
    %P_new = [FixedWingStateBus.North FixedWingStateBus.East FixedWingStateBus.Height];
    P_new = [xi yi 0];
    psi_new = psii;

    if (distance<2000 && zi>za)
    Ru = abs ((((P_new(1) - O(1))^2) + ((P_new(2) - O(2))^2)) ^(1/2)-r);%Calculation of UAV distance from center
    theta_new = atan2((P_new(2)-O(2)),(P_new(1)-O(1))); %new path angle calculation
    x_i = ((r*(cos(theta_new+delta)))+O(1));
    y_i = ((r*(sin(theta_new+delta)))+O(2));
    psi_d = atan2((y_i-P_new(2)),(x_i-P_new(1)));%calculation of desired heading angle
    %Desired Heading Angle is very important!!!it is different with that of
    %low fidelity test program.
    if psi_d >=0
       DesiredHeading = -psi_d-pi;
    elseif psi_d < 0
       DesiredHeading = -(psi_d + 2*pi)-pi;
    end

    Cross_Tracking_Error = (abs(((O(1)-P_new(1))^2)+((O(2)-P_new(2))^2))^(1/2))-r;
    end

%}

   else           % Approach to the landing strip
     
               
%% Parameters for Straight Line Chasing
    delta = 1000; %distance of the look ahead point
    W_i = [0 0]; %straight line origin
    W_ip1 = [(xf-xu) (yf-yu)]; %straight line destination suggestion!!! set y coordinate of reference line as zero to reduce the cross tracking error

%^^^^^^^^^^^^^^^^^^^^^Update UAV movements^^^^^^^^^^^^^^^^^^^^^^^^^
   %P_new = [(xi-xb) (yi-yb) zi];%() is very important!
 
   P_new = [(xi-xu) (yi-yu) zi];
   psi_new = psii;  
   %% consider to replace P_new with a real time position
  % if (P_new(1)<W_ip1(1) && P_new(1)<W_ip1(2))
   if (zi<za)
   status = 3;  
   %{
          count_C = count_C +1; 
        if(count_C <=1)
            CurrX3 = xi;
            CurrY3 = yi;
            CurrZ3 = zi;
        end
   %}
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
       DesiredHeading = -psi_d-pi;
    elseif psi_d < 0
       DesiredHeading = -(psi_d + 2*pi)-pi;
    end
   Cross_Tracking_Error = (abs(Ru^2 - path_g^2))^(1/2);
        
    end
   
    end
end