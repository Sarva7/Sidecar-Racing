%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   F1 Sidecar Model for R. Weekers Techniek                                %                               
%   Title         : Sidecar racing dynamics                                                 %
%   Team          : Abhinanda Kalsi Nagaraja , Frédérick Wentzel , Manish Varma Raathimiddi % 
%                   Ronnie Edavattathu, Sarvajith Guruprakash, Somyadeep Dutta              %
%   Date          : 18/January/2022                                                         %
%   University    : HAN University of Applied Sciences                                      %
%   Place         : Arnhem, Netherlands                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note:                                                                   %
% 1] Run sectionwise if you encounter any error                           %
% 2] Run the scripts based on which model you need                        %
% 3] Don't run the whole script at once, as simulation time will be high  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear Workspace
clear all
clc

%% Add Path (Comment out the rest of the script based on which model you use or else you will encounter position violation error, if so clear the workspace)
addpath([pwd '/Scripts_Data']);
addpath([pwd '/Images']);
addpath([pwd '/CAD_Geometry']);
addpath([pwd '/Complex_Model_CAD']);


% TNO_Vehicle_PARAM                             % For Simplified model
% susp_sys_params                               % For Double wishbone model
% Complex_Script                                % For Complex model
Front_Suspension_DataFile                       % For Sidecar model
Rear_Suspension_DataFile                        % For Sidecar model
% open_system('Simplified.slx');
% open_system('Simple_Double_Wishbone.slx');
% open_system('Complex_Model.slx');
open_system('Sidecar.slx');
% open_system('Simulator.slx');

%% For Sidecar Simulator and for variable update
simtime = 200;
L = 0;                      % Passenger input
N = 1;                      % Passenger input
R = 2;                      % Passenger input
ST_S = 0;                   % Step time for steering 
U_A = 3.055;                % Upright allignment vertically towards road in deg    
Camber_F = 0;               % Camber front in deg
Camber_S = 0;               % Camber side in deg
Front_Toe = 0;              % Front toe in deg
Toe_S = 4;                  % Side toe in mm
Side_Toe = 0.2292;          % Front toe in deg
Fk = 70000;                 % Stiffness N/m
Rk = 70000;                 % Stiffness N/m
Passenger_Mass = 75;        % Mass in kg
V = 0;                      % Vehicle speed km/h
Delta = 0;                  % Steering in deg
P = N;                      % Passenger position
Front_Arm = 302;            % Front arm length mm
Rear_Arm_Lower = 223;       % Rear lower arm length mm 
Rear_Arm_Upper = 231;       % Rear upper arm length mm
Handle_Arm = 224.4;         % Handle arm length mm

% Floor parameters
Floor.l = 1000;  % m
Floor.w = 1000;  % m
Floor.h = 0.01;  % m

% Grid parameters
Grid.clr = [1 1 1]*1;
Grid.numSqrs = 250;
Grid.lineWidth = 0.02;
Grid.box_h = (Floor.l-(Grid.lineWidth*(Grid.numSqrs+1)))/Grid.numSqrs;
Grid.box_l = (Floor.l-(Grid.lineWidth*(1+1)))/1;
Grid.extr_data = Extr_Data_Mesh(Floor.w,Floor.w,Grid.numSqrs,1,Grid.box_h,Grid.box_l);

%% Modelling starts from here:
%% Load
load('Fz.mat');
figure(1)
plot(out.Fz(:,1),'LineWidth',1.5); grid on;
xlabel('Time (s)');
ylabel('Load (N)');
legend('Front Load','Rear Load','Side Load','Location','northeast');
title('Load Distribution')

%% Straignt line
simtime = 15;
P = N;
ST_S = 5;
Delta = 0;
for V=90:30:180
    sim('Sidecar.slx')
    figure(2)
    plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on
    xlim([-10 400])
    ylim([-200 200])
    xlabel('Vehicle position in X cordinate (m)');
    ylabel('Vehicle position in Y cordinate (m)');
    legend('Vehicle speed =90km/hr','Vehicle speed =120km/hr','Vehicle speed =150km/hr','Vehicle speed =180km/hr','Location','northeast');
    info = ({['Side Toe in = ',num2str(Side_Toe'),'[deg]']});
    title(info)
end

%% Straightline stability after tuning parameters 
simtime = 25;
P = N;
V = 180;
ST_S = 6.2;
U_A = 3.055;                     
Toe_out = 3;
Delta = 0;
sim('Sidecar.slx')
figure(3) 
plot(ans.X(11684:end,1)-171.615,ans.Y(11684:end,1)-3.15,'linewidth',1.5); 
grid on;
hold on
xlim([-10 700])
ylim([-350 350])
xlabel('Vehicle position in X cordinate (m)');
ylabel('Vehicle position in Y cordinate (m)');
legend('Vehicle speed =180km/hr','Location','northeast');
info = ({['Front Toe out = ',num2str(Toe_out'),'[deg]',[', Side Toe in = ',num2str(Side_Toe')],'[deg]']});
title(info)

%% Steady state-left
simtime = 35;
Delta = 5;
P = L;
V = 95;
ST_S = 8.5;
sim('Sidecar.slx')
figure(4)
title('Steady state')
subplot(2,2,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
plot(ans.X(11000:end,1),ans.Y(11000:end,1),'linewidth',2); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
legend('Non-steady state','Steady state','Location','southwest')
title('Position')
subplot(2,2,3)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
plot(ans.tout(11000:end),ans.Lateral_Acceleration(11000:end)/9.81,'linewidth',2); grid on;hold on;
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
legend('Non-steady state','Steady state','Location','southwest')
title('Lateral acceleration')
subplot(2,2,4)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
plot(ans.tout(11000:end),ans.yawrate(11000:end),'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Non-steady state','Steady state','Location','southwest')
title('Yaw rate')
subplot(2,2,2)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
plot(ans.tout(11000:end),ans.Velocity(11000:end)/3.6,'linewidth',2); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Non-steady state','Steady state','Location','southwest')
title('Velocity')

%% Steady state-right
simtime = 35;
Delta = -5;
P = R;
V = 100;
ST_S = 8.5;
sim('Sidecar.slx')
figure(5)
title('Steady state')
subplot(2,2,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
plot(ans.X(11000:end,1),ans.Y(11000:end,1),'linewidth',1.5); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
legend('Non-steady state','Steady state','Location','southwest')
title('Position')
subplot(2,2,3)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
plot(ans.tout(11000:end),ans.Lateral_Acceleration(11000:end)/9.81,'linewidth',2); grid on;hold on;
ylim([-1 1])
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
legend('Non-steady state','Steady state','Location','southwest')
title('Lateral acceleration')
subplot(2,2,4)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
plot(ans.tout(11000:end),ans.yawrate(11000:end),'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Non-steady state','Steady state','Location','southwest')
title('Yaw rate')
subplot(2,2,2)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
plot(ans.tout(11000:end),ans.Velocity(11000:end)/3.6,'linewidth',2); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Non-steady state','Steady state','Location','southwest')
title('Velocity')

%% Transient behaviour
%% Step Test-left
P = L;
simtime = 15;
ST_S = 8.5;
Delta = 5;
V = 96;
sim('Sidecar.slx')
figure(6)
subplot(2,2,4)
plot(ans.tout,ans.roll,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Roll (rad/s)')
legend('Delta= 5 ^o','Location','southwest')
title('Roll')
subplot(2,2,2)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
legend('Delta= 5 ^o','Location','southwest')
title('Lateral acceleration')
subplot(2,2,3)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Delta= 5 ^o','Location','southwest')
title('Yaw rate')
subplot(2,2,1)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Delta= 5 ^o','Location','southwest')
title('Velocity')

%% Step Test-right
P = R;
simtime = 15;
ST_S = 8.5;
Delta = -5;
V = 96;
sim('Sidecar.slx')
figure(7)
subplot(2,2,4)
plot(ans.tout,ans.roll,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Roll (rad/s)')
legend('Delta= -5 ^o','Location','southwest')
title('Roll')
subplot(2,2,2)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
legend('Delta= -5 ^o','Location','southwest')
title('Lateral acceleration')
subplot(2,2,3)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Delta= -5 ^o','Location','southwest')
title('Yaw rate')
subplot(2,2,1)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Delta= -5 ^o','Location','southwest')
title('Velocity')

%% Validation for race track
simtime = 30;
Delta= -15;
P = R;
V = 52;
ST_S = 5;
sim('Sidecar.slx')
figure(8)
subplot(2,2,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
legend('Delta= -15 ^o','Steady state','Location','southwest')
title('Position')
subplot(2,2,3)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
ylim([-0.2 0.7])
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
legend('Delta= -15 ^o','Steady state','Location','southwest')
title('Lateral acceleration')
subplot(2,2,4)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Delta= -15 ^o','Steady state','Location','southwest')
title('Yaw rate')
subplot(2,2,2)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Delta= -15 ^o','Steady state','Location','southwest')
title('Velocity')

%% Handling behaviour
%% Steady state for under and oversteer
simtime = 35;
% P = L;
P = R;
ST_S = 8.5;
% Delta = 5;
Delta = -5;
    for V = 80:10:100
        sim('Sidecar.slx')
        figure(9)
        subplot(2,2,1)
        plot(ans.X(11000:end,1),ans.Y(11000:end,1),'linewidth',2); grid on;hold on;
        xlabel('Vehicle position in X cordinate (m)')
        ylabel('Vehicle position in Y cordinate (m)')
        title('Position')
        subplot(2,2,2)
        plot(ans.tout,ans.Delta,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Steering (deg)')
        title('Steering')
        subplot(2,2,3)
        plot(ans.tout(12073:end,1),ans.Lateral_Acceleration(12073:end,1)/9.81,'linewidth',1.5); grid minor;hold on;
        xlabel('Time (s)')
        ylabel('Lateral Acceleration (g)')
        xlim([15 35])
        ylim([0 1])
        title('Lateral acceleration')
        subplot(2,2,4)
        plot(ans.tout(12073:end,1),ans.yawrate(12073:end,1),'linewidth',1.5); grid minor;hold on;
        xlabel('Time (s)')
        ylabel('Yaw rate (rad/s)')
        xlim([15 35])
        ylim([-0.5 0])
        title('Yaw rate')
    end
subplot(2,2,1)
legend('Speed = 80 km/hr','Speed = 90km/hr','Speed = 100km/hr','Location','southwest')
subplot(2,2,2)
legend('Speed = 80 km/hr','Speed = 90km/hr','Speed = 100km/hr','Location','southwest')
subplot(2,2,3)
legend('Speed = 80 km/hr','Speed = 90km/hr','Speed = 100km/hr','Location','southwest')
subplot(2,2,4)
legend('Speed = 80 km/hr','Speed = 90km/hr','Speed = 100km/hr','Location','southwest')

%% Step Test-left and right
P = L;
simtime = 15;
ST_S = 8.5;
Delta = 10;
V = 80;
sim('Sidecar.slx')
figure(10)
subplot(3,1,2)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
ylim([-1.5 1.5])
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
title('Lateral acceleration')
subplot(3,1,3)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
title('Yaw rate')
subplot(3,1,1)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Velocity')

P = R;
simtime = 15;
ST_S = 8.5;
Delta = -10;
V = 80;
sim('Sidecar.slx')
figure(10)
subplot(3,1,2)
plot(ans.tout,-ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
ylim([-1.5 1.5])
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
title('Lateral acceleration')
subplot(3,1,3)
plot(ans.tout,-ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
title('Yaw rate')
subplot(3,1,1)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Velocity')

subplot(3,1,1)
legend('Left turn','Right turn','Location','southwest')
subplot(3,1,2)
legend('Left turn','Right turn','Location','southwest')
subplot(3,1,3)
legend('Left turn','Right turn','Location','southwest')

%% Note:
% From here onwards before you run a section you need to run the Manual simulating  section to make it default

%% Stiffness of the damper- Front
% 30% increament and decreament of stiffness for front 
simtime = 35;
P = R;
% P = L;
ST_S = 5;
V = 95;
Delta = -5;
% Delta = 5;
for Fk=[49000 91000]
    sim('Sidecar.slx')
    figure(11)
    subplot(2,1,1)
    plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
    xlabel('Vehicle position in X cordinate (m)')
    ylabel('Vehicle position in Y cordinate (m)')
    title('Position')
    subplot(2,1,2)
    plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
    xlabel('Time (s)')
    ylabel('Yaw rate (rad/s)')
    title('Yaw rate')
end
 subplot(2,1,1)
 legend('Stiffness = 49000 N/m','Stiffness = 91000 N/m','Location','southwest')
 subplot(2,1,2)
 legend('Stiffness = 49000 N/m','Stiffness = 91000 N/m','Location','southwest')
 
%% Stiffness of the damper- Rear
% 30% increament and decreament of stiffness for rear
simtime = 35;
P = R;
% P = L;
ST_S = 5;
V = 100;
Delta = -5;
% Delta = 5;
for Rk=[49000 91000]
    sim('Sidecar.slx')
    figure(12)
    subplot(2,1,1)
    plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
    xlabel('Vehicle position in X cordinate (m)')
    ylabel('Vehicle position in Y cordinate (m)')
    title('Position')
    subplot(2,1,2)
    plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
    xlabel('Time (s)')
    ylabel('Yaw rate (rad/s)')
    title('Yaw rate')
end
 subplot(2,1,1)
 legend('Stiffness = 49000 N/m','Stiffness = 91000 N/m','Location','southwest')
 subplot(2,1,2)
 legend('Stiffness = 49000 N/m','Stiffness = 91000 N/m','Location','southwest')
 
%% Sensitivity Analysis
%% Front camber steady state- left
simtime = 30;
P = L;
ST_S = 5;
Delta = 5;
V = 90;
    for Camber_F =-3:1.5:3
        sim('Sidecar.slx')
        figure(13)
        subplot(3,1,1)
        plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
        xlabel('Vehicle position in X cordinate (m)')
        ylabel('Vehicle position in Y cordinate (m)')
        title('Position')
        subplot(3,1,2)
        plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Lateral acceleration (g)')
        title('Lateral acceleration')
        subplot(3,1,3)
        plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
    	ylabel('Yaw rate (rad/s)')
        title('Yaw rate')
    end
subplot(3,1,1)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')
subplot(3,1,2)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')
subplot(3,1,3)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')

%% Front camber steady state- right
simtime = 35;
P = R;
ST_S = 5;
Delta = -5;
V = 90;
    for Camber_F =-3:1.5:3
        sim('Sidecar.slx')
        figure(14)
        subplot(3,1,1)
        plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
        xlabel('Vehicle position in X cordinate (m)')
        ylabel('Vehicle position in Y cordinate (m)')
        title('Position')
        subplot(3,1,2)
        plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Lateral acceleration (g)')
        title('Lateral acceleration')
        subplot(3,1,3)
        plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
    	ylabel('Yaw rate (rad/s)')
        title('Yaw rate')
    end
subplot(3,1,1)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')
subplot(3,1,2)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')
subplot(3,1,3)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')

%% Side camber steady state- left
simtime = 30;
P = L;
ST_S = 5;
Delta = 5;
V = 90;
    for Camber_F =-3:1.5:3
        sim('Sidecar.slx')
        figure(15)
        subplot(3,1,1)
        plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
        xlabel('Vehicle position in X cordinate (m)')
        ylabel('Vehicle position in Y cordinate (m)')
        title('Position')
        subplot(3,1,2)
        plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Lateral acceleration (g)')
        title('Lateral acceleration')
        subplot(3,1,3)
        plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
    	ylabel('Yaw rate (rad/s)')
        title('Yaw rate')
    end
subplot(3,1,1)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')
subplot(3,1,2)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')
subplot(3,1,3)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')

%% Side camber steady state- right
simtime = 35;
P = R;
ST_S = 5;
Delta = -5;
V = 90;
    for Camber_F =-3:1.5:3
        sim('Sidecar.slx')
        figure(16)
        subplot(3,1,1)
        plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
        xlabel('Vehicle position in X cordinate (m)')
        ylabel('Vehicle position in Y cordinate (m)')
        title('Position')
        subplot(3,1,2)
        plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Lateral acceleration (g)')
        title('Lateral acceleration')
        subplot(3,1,3)
        plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
    	ylabel('Yaw rate (rad/s)')
        title('Yaw rate')
    end
subplot(3,1,1)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')
subplot(3,1,2)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')
subplot(3,1,3)
legend('Camber = -3 ^o','Camber = -1.5 ^o','Camber = 0 ^o','Camber = 1.5 ^o','Camber = 3 ^o','Location','southwest')

%% Steady state-right max camber to increase grip
simtime = 35;
Delta = -5;
P = R;
V = 95;
ST_S = 5;
Camber_S = -3;
Camber_F = 3;
sim('Sidecar.slx')
figure(17)
title('Steady state')
subplot(2,2,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
legend('Delta= -5 ^o','Camber_S = -3 ^o','Steady state','Location','southwest')
title('Position')
subplot(2,2,2)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
legend('Delta= -5 ^o','Camber_S = -3 ^o','Steady state','Location','southwest')
title('Lateral acceleration')
subplot(2,2,3)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Delta= -5 ^o','Camber_S = -3 ^o','Steady state','Location','southwest')
title('Yaw rate')
%% Stiffness
simtime = 15;
P = L;
ST_S = 8.5;
Delta = 5;
V = 90;
    for Rk = 60000:30000:120000
        sim('Sidecar.slx')
        figure(18)
        subplot(3,2,1)
        plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
        xlabel('Vehicle position in X cordinate (m)')
        ylabel('Vehicle position in Y cordinate (m)')
        title('Position')
        subplot(3,2,2)
        plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Lateral acceleration (g)')
        title('Lateral acceleration')
        subplot(3,2,3)
        plot(ans.logsout{13}.Values,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Load Front (N)')
        title('Load transfer')
        subplot(3,2,4)
        plot(ans.logsout{15}.Values,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Load Rear (N)')
        title('Load transfer')
        subplot(3,2,5)
        plot(ans.logsout{18}.Values,'linewidth',1.5); grid on;hold on;
        xlabel('Time (s)')
        ylabel('Load Side (N)')
        title('Load transfer')
    end
subplot(3,2,1)
legend('Rk = 60000 N/m','Rk = 90000 N/m','Rk = 120000 N/m','Location','southwest')
subplot(3,2,2)
legend('Rk = 60000 N/m','Rk = 90000 N/m','Rk = 120000 N/m','Location','southwest')
subplot(3,2,3)
legend('Rk = 60000 N/m','Rk = 90000 N/m','Rk = 120000 N/m','Location','southwest')
subplot(3,2,4)
legend('Rk = 60000 N/m','Rk = 90000 N/m','Rk = 120000 N/m','Location','southwest')
subplot(3,2,5)
legend('Rk = 60000 N/m','Rk = 90000 N/m','Rk = 120000 N/m','Location','southwest')

%% Toe for all different joint points on handle bar 
% For T1
simtime = 35;
P = L;
% P = R;
ST_S = 5;
Delta = 5;
% Delta = -5;
V = 90;
sim('Sidecar.slx')
figure(19)
subplot(2,2,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
legend('Toe = T1','Location','southwest')
title('Position')
subplot(2,2,3)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Lateral acceleration (g)')
legend('Toe = T1','Location','southwest')
title('Lateral acceleration')
subplot(2,2,4)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Toe = T1','Location','southwest')
title('Yaw rate')
subplot(2,2,2)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Toe = T1','Location','southwest')
title('Velocity')
%% For T2
simtime = 35;
P = L;
% P = R;
ST_S = 5;
Delta = 5;
% Delta = -5;
V = 90;
sim('Sidecar.slx')
figure(19)
subplot(2,2,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
legend('Toe = T2','Location','southwest')
title('Position')
subplot(2,2,3)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Lateral acceleration (g)')
legend('Toe = T2','Location','southwest')
title('Lateral acceleration')
subplot(2,2,4)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Toe = T2','Location','southwest')
title('Yaw rate')
subplot(2,2,2)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Toe = T2','Location','southwest')
title('Velocity')

%% For T3
simtime = 35;
P = L;
% P = R;
ST_S = 5;
Delta = 5;
% Delta = -5;
V = 90;
sim('Sidecar.slx')
figure(19)
subplot(2,2,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
legend('Toe = T3','Location','southwest')
title('Position')
subplot(2,2,3)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Lateral acceleration (g)')
legend('Toe = T3','Location','southwest')
title('Lateral acceleration')
subplot(2,2,4)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
legend('Toe = T3','Location','southwest')
title('Yaw rate')
subplot(2,2,2)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Toe = T3','Location','southwest')
title('Velocity')

subplot(2,2,1)
legend('Toe = T1','Toe = T2','Toe = T3','Location','southwest')
subplot(2,2,2)
legend('Toe = T1','Toe = T2','Toe = T3','Location','southwest')
subplot(2,2,3)
legend('Toe = T1','Toe = T2','Toe = T3','Location','southwest')
subplot(2,2,4)
legend('Toe = T1','Toe = T2','Toe = T3','Location','southwest')

%% Offset 
simtime = 35;
% P = L;
P = R;
ST_S = 5;
% Delta = 5;
Delta = -5;
V = 90;
for Front_Arm = [292 312]
sim('Sidecar.slx')
figure(20)
subplot(2,2,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
title('Position')
subplot(2,2,3)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
ylim([-1 1]);
xlabel('Time (s)')
ylabel('Lateral acceleration (g)')
title('Lateral acceleration')
subplot(2,2,4)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
title('Yaw rate')
subplot(2,2,2)
plot(ans.tout,ans.Velocity/3.6,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Velocity')
end
subplot(2,2,1)
legend('Offset = -10mm','Offset = +10mm','Location','southwest')
subplot(2,2,2)
legend('Offset = -10mm','Offset = +10mm','Location','southwest')
subplot(2,2,3)
legend('Offset = -10mm','Offset = +10mm','Location','southwest')
subplot(2,2,4)
legend('Offset = -10mm','Offset = +10mm','Location','southwest')

%% Tire data comparision
%% Steady state-left
simtime = 35;
Delta = 5;
P = L;
V = 102;
ST_S = 8.5;
sim('Sidecar.slx')
figure(21)
title('Steady state')
subplot(3,1,1)
plot(ans.X(:,1),ans.Y(:,1),'linewidth',1.5); grid on;hold on;
xlabel('Vehicle position in X cordinate (m)')
ylabel('Vehicle position in Y cordinate (m)')
title('Position')
subplot(3,1,2)
plot(ans.tout,ans.Lateral_Acceleration/9.81,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Lateral Acceleration (g)')
title('Lateral acceleration')
subplot(3,1,3)
plot(ans.tout,ans.yawrate,'linewidth',1.5); grid on;hold on;
xlabel('Time (s)')
ylabel('Yaw rate (rad/s)')
title('Yaw rate')

subplot(3,1,1)
legend('TNO Data','Acquired Data','Location','southwest')
subplot(3,1,2)
legend('TNO Data','Acquired Data','Location','southwest')
subplot(3,1,3)
legend('TNO Data','Acquired Data','Location','southwest')

%% Complex model suspension forces
load('Suspension_Force.mat')
figure(22)
plot(out.logsout{1}.Values);
mean(out.logsout{1}.Values)
figure(23)
plot(out.logsout{2}.Values);
mean(out.logsout{2}.Values)
                                                                    %%%%%%%%%%%%%%%%%%%
                                                                    %    Thank you    %
                                                                    %%%%%%%%%%%%%%%%%%%