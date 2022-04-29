%%
SIZE = 10000;
e = zeros(1,SIZE);
c = zeros(1,SIZE);
aileron_e = zeros(1,SIZE);
psi_ref = zeros(1,SIZE);
gamma_ref = zeros(1,SIZE);
%% This function is for testing for whether planned straight line can converge to straight line
%% This function is for testing for converge to planned straight line
xb = 31018;
yb = -23100;
xf = 34018;
yf = -27100;
Rl = 1016; %convert 3333 ft to 1016 meters
psif = 0;
xl         =   xf + 4 * Rl * cos(psif - pi);
yl         =   yf + 4 * Rl * sin(psif - pi);
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
%%
V = 200; %constant UAV velocity 
g = 9.81; %gravitational acceleration
delta = 1000; %distance of the look ahead point
W_ip1 = [0 0]; %straight line origin
W_i = [xb-xl yb-yl]; %straight line destination
%^^^^^^^^^^^^^^^^definition of controller parameters^^^^^^^^^^^^^^^^^^^^^^^
k_p=0.8; %proportional gain
k_i=0.01; %integral gain
k_d=1; %derivative gain
%^^^^^^^^^^^^^^^^^^^Specification of time step^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
dt= 0.1; % change time unit can speed up simulation
U_0=100; %initial UAV speed
U_d=120; %desired UAV speed 
%^^^^^^^^^^^^^^^^^^^^^^definition of desired path^^^^^^^^^^^^^^^^^^^^^^^^^^
path=[W_i;W_ip1];
%ao = 18;
%curr_x = xl + Rl*cos(ao*pi/180);
%curr_y = yl + Rl*sin(ao*pi/180);
psi=2.5; %Initial heading angle of UAV
%p=[1000 3000]; %initial position of UAV
p = [xb-xl yb-yl];%initial position of UAV
theta = atan2((W_ip1(2)-W_i(2)),(W_ip1(1)-W_i(1))); %Calculation of LOS angle
path_g = abs (((p(1) - W_i(1))^2) + ((p(2) - W_i(2))^2)) ^(1/2); %Ground distance
thetau = atan2((p(2)-W_i(2)),p(1)-W_i(1));%UAV angle at start of path
beta=theta-thetau;
Ru=((path_g^2)-((sin(beta)*path_g)^2))^(1/2);
%^^^^^^^^^^^^^^^^^^^^^^Definition of the look ahead point^^^^^^^^^^^^^^^^^^
x_i = ((Ru)+delta)*(cos(theta));
y_i = ((Ru)+delta)*(sin(theta));
p_i = [x_i y_i];
psi_d = atan2(y_i-(p(2)),x_i-(p(1))); %commanded heading angle
u = (psi_d-psi); %controller input for changing heading angle
%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^Motion of UAV^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
x_d=U_0*(cos(psi))*dt;
y_d=U_0*(sin(psi))*dt;
 
%^^^^^^^^^^^^^^^estimation of heading angle and position^^^^^^^^^^^^^^^^^^^
P_new=[(p(1)+x_d),(p(2)+y_d)];
psi_new=(psi+u);
%^^^^^^^^^^^^^^^^^^^^^^over time positioning and heading of UAV^^^^^^^^^^^^
X=[p(1)];
Y=[p(2)];
S=511; %area of UAV wing
rho=0.3045; %density of air
b=59.64; %span of wing
mass = 333400; %mass of UAV
I_xx=0.247e8; %inertial moment
L_p=-1.076e7; %rolling moment
Cl_da=0.668e-2; %roll moment due to aileron deflection coefficient
Q_dS=1/2*rho*U_0^2*S;  %dynamic pressure
L_da=Q_dS*b*Cl_da;  %roll moment due to aileron
%^^^^^^^^^^^initialising controller^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
roll_ref=0;     %initial UAV roll position
rollrate_ref=0; %initial UAV rollrate
t_ei=0;         %thrust PI integrator
ei=0;           %aileron PID integrator
%^^^^^^^^^^^^^^^^^estimation of stability derivatives^^^^^^^^^^^^^^^^^^^^^^
a=L_p/I_xx;
beta=L_da/I_xx;
roll_d=atan(u*U_0/g); %desired roll calculation

if abs(roll_d) > 1.5;  %limit of roll

   if roll_d < 0;
      roll_d = -1.5;
   else if roll_d>0;
           roll_d = 1.5;
       end
   end
end
rollrate_d=roll_d*dt; %desired rollrate
aileron = k_p*(roll_d-roll_ref)+(k_i*ei)+k_d*(rollrate_d-rollrate_ref); %deflection of aileron
rollrate_new = (((a*rollrate_ref)+(beta*aileron))*dt); %new roll rate output
roll_new = (rollrate_new/dt)+roll_ref; %new roll output
roll_old=roll_ref; %old roll initialisation as feedback
rollrate_old=rollrate_ref; %initiallising old rollrate for feedback
%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^control of thurst^^^^^^^^^^^^^^^^^^^^^^^^^^
t_ei=t_ei+(U_d-U_0)*dt;
thrust=k_p*(U_d-U_0)+(k_i*t_ei);
V_new=U_0+(thrust*dt);
V_old=V_new;
n = 0; %starting count

while (P_new(1)>W_ip1(1) && P_new(2)>W_ip1(2))
path_g = abs (((P_new(1) - W_i(1))^2) + ((P_new(2) - W_i(2))^2)) ^(1/2); %Calculation of UAV distance from origin
theta = atan2((W_ip1(2)-W_i(2)),(W_ip1(1)-W_i(1))); %path angle calculation
thetau = atan2((P_new(2)-W_i(2)),P_new(1)-W_i(1));%UAV path angle from origin

l_d=theta-thetau;
Ru=((path_g^2)-((path_g*sin(l_d))^2))^(1/2);
x_i = ((Ru)+delta)*(cos(theta));
y_i = ((Ru)+delta)*(sin(theta));
p_i = [x_i y_i];
psi_d = atan2((y_i-(P_new(2))),(x_i-(P_new(1)))); %calculation of desired heading angle
u = wrapToPi(psi_d-psi_new); %controller input for changing heading angle
ei=ei+((roll_d-roll_old)*dt); %updating the integrator
roll_d=atan(u*V_old/g); %desired roll calculation
if abs(roll_d) > 1.5
   if roll_d < 0
      roll_d = -1.5;
   else if roll_d>0
      roll_d = 1.5;
       end
    end
end
rollrate_d=(roll_d-roll_old)*dt; %calculation of desired rollrate
aileron = (k_p*(roll_d-roll_old)+(k_i*ei)+(k_d*(rollrate_d-rollrate_old))); %calculation of deflection of aileron
rollrate_new = (((a*rollrate_old)+(beta*aileron))*dt); %new rollrate calculation
roll_new = (rollrate_new/dt)+roll_old; %new roll angle calculation
rollrate_old=rollrate_new; %rollrate as feedback
roll_old=roll_new; %roll angle as feedback
%psi_old = psi_new; %UAV heading as feedback
psi_b=g/V_old*(tan(roll_new)); %due to new roll change in heading effect
psi_new = wrapToPi(psi_new+psi_b); %calculation of new heading angle
Q_dS=1/2*rho*V_old^2*S; %calculation of dynamic pressure
L_da=Q_dS*b*Cl_da; %due to aileron calculation of roll moment
beta=L_da/I_xx;
a=L_p/I_xx;
%Calculation of UAV movements
x_d=V_old*(cos(psi_new))*dt;
y_d=V_old*(sin(psi_new))*dt;
%contorl of thrust
t_ei=t_ei+(U_d-V_old)*dt;
  thrust=k_p*(U_d-V_old)+(k_i*t_ei);
   V_new=V_old+(thrust*dt);
V_old=V_new;
%estimation of new position of UAV
P_new=[(P_new(1)+x_d),(P_new(2)+y_d)] ;
figure(1)
Y=[ Y P_new(2)];
X=[ X P_new(1)];
plot(X,Y)
hold on
plot(path(:,1),path(:,2),':')
%xlim([0 10000]);
%ylim([0 10000]);
xlim([xl-xl-Rl xb-xl]);
ylim([yl-yl-Rl yb-yl]);
xlabel('x-direction in meter')
ylabel('y-direction in meter')
title('Followed path using carrot chasing algorithm')
drawnow
n = n+1
hold on
for j = n;
%array of measurment 
d = (abs(Ru^2 - path_g^2))^(1/2);
c(1 , j) = d;
e(1,j) = u;
aileron_e(1,j)=aileron;
psi_ref(1,j)= psi_d;
gamma_ref(1,j) = -15*pi/180;
end
end
%%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^measurment plots^^^^^^^^^^^^^^^^^^^^^^^^^^^^
figure(2)
f = [1:1:n];
plot(f,e)
xlabel('time in (sec/100)')
ylabel('Change in heading in radian')
title('Variation in controller effort with time')
figure(3)
plot(f,c)
xlabel('time in (sec/100)')
ylabel('cross track deviation(ft)')
title('Variation of cross track deviation with time')
figure(4)
plot(f,aileron_e)
xlabel('time in (sec/100)')
ylabel('Deflection of aileron in radian')
title('Variation in aileron control with time')
plot(f,psi_ref)
xlabel('time in (sec/100)')
ylabel('heading new in radian')
title('Variation in controller effort with time')
figure(5)
plot(f,gamma_ref)
xlabel('time in (sec/100)')
ylabel('pitch angle in radian')
title('Variation in controller effort with time')
figure(6)
time = n*dt;
