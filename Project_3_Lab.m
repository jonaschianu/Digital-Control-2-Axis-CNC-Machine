%% Matlab Code for Project 3
% LAB
% MECH 541
% University of British Columbia
% Jonas Chianu
% 31298391
% jchianu@student.ubc.ca
% December 2020

close all; bdclose all;
Project_3_Prelab;
close all; bdclose all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part E - Effect of Bandwidth
%% Part E.1
xe_L=xr-xa_L; ye_L=yr-ya_L;
xe_H=xr-xa_H; ye_H=yr-ya_H;
xe_M=xr-xa_M; ye_M=yr-ya_M;

figure(1);
subplot(2,1,1);
plot(tr,xe_L,tr,xe_H);
title({'Figure 1: Tracking Errors in the X and Y Axes as Functions of Time'
    ,'for the LBW and HBW Controllers (E.1)'});
ylabel('Tracking Error (x-axis) (mm)');
legend('LBW','HBW','Location','SouthEast');
grid();

subplot(2,1,2);
plot(tr,ye_L,tr,ye_H);
xlabel(' Time (sec)'); ylabel('Tracking Error (y-axis)  (mm)');
grid();

%% Part E.2
figure(2);
sgtitle({'Figure 2: Tool Motion for Reference Toolpath and',
    'Simulations using Three Cases of Control System (E.2)'});
subplot(2,2,1);
plot(xa_L,ya_L,'r',xa_H,ya_H,'g',xa_M,ya_M,'b',xr,yr,'--');
title('R1')
ylabel('y (mm)');
axis([19.94 20.13 14.84 15.05])
grid;

subplot(2,2,2);
plot(xa_L,ya_L,'r',xa_H,ya_H,'g',xa_M,ya_M,'b',xr,yr,'--');
title('R2')
axis([39.95 40.25 29.95 30.25])
grid;

subplot(2,2,3);
plot(xa_L,ya_L,'r',xa_H,ya_H,'g',xa_M,ya_M,'b',xr,yr,'--');
title('R3')
xlabel(' x (mm'); ylabel('y (mm)');
legend('LBW','HBW','MD','Ref');
axis([59.85 60.3 29.6 30.05])
grid;

subplot(2,2,4);
plot(xa_L,ya_L,'r',xa_H,ya_H,'g',xa_M,ya_M,'b',xr,yr,'--');
title('R4')
xlabel(' x (mm');
axis([104.97 105.05 55.95 56.5])
grid;

%% Part E.3
i = find(xr > 19, 1, 'first');
iend= find(xr > 20, 1, 'first');

Ce_max=0;
for k=i:iend
    leR1 = sqrt(xe_L(k)^2+ye_L(k)^2);
    alpha = atan(ye_L(k)/xe_L(k));
    theta = atan(3/4);
    beta = alpha - theta;
    Ce = leR1*sin(beta);
    
    if Ce>Ce_max
        Ce_max=Ce;
    end
end

Ce_max;

%% Part F - Effect of Maximum Feedrate
%% Part F.1
%The Displacement, Feedrate, and Tangential Acceleration Profiles
figure(3)
subplot(3,1,1)
plot(t_fast,l_fast)
title({'Figure 3 : The Displacement, Feedrate and Tangential Acceleration',
    'as Functions of Time (F=250 mm/s) (F.1)'})
ylabel('Displacement (mm)');
grid();

subplot(3,1,2)
plot(t_fast,f_fast)
ylabel('Velocity (mm/s)');
grid();

subplot(3,1,3)
plot(t_fast,a_fast)
xlabel(' Time (sec)'); ylabel('Acceleration (mm/s^2)');
grid();

%The Axis Position, Velocity, and Acceleration Profiles
figure(4)
subplot(3,1,1)
plot(t_fast,x_fast,t_fast,y_fast)
title({'Figure 4: The Axis Position, Velocity, and Acceleration Commands',
    'as Functions of Time (F=250 mm/s) (F.1)'})
ylabel('Axis Position (mm)');
legend('x','y','Location','NorthEast')
grid();

subplot(3,1,2)
plot(t_fast,dx_fast,t_fast,dy_fast)
ylabel('Axis Velocity (mm/s)');
grid();

subplot(3,1,3)
plot(t_fast,dxdx_fast,t_fast,dydy_fast)
xlabel(' Time (sec)'); ylabel('Axis Acceleration (mm/s^2)');
grid();

%% Part F.2
var_x=[transpose(t_fast) transpose(x_fast)];
var_y=[transpose(t_fast) transpose(y_fast)];

contx_Z=LLI_L_x_Z;
conty_Z=LLI_L_y_Z;
stop_time=2;
out_L=sim('Part_C1.slx');
bdclose all;
xr_Fast = out_L.simout.signals.values(:,2);
yr_Fast = out_L.simout.signals.values(:,4);
xa_L_Fast = out_L.simout.signals.values(:,3);
ya_L_Fast = out_L.simout.signals.values(:,5);

xe_L_Fast=xr_Fast-xa_L_Fast; ye_L_Fast=yr_Fast-ya_L_Fast;

figure(5);
subplot(2,1,1);
plot(tr,xe_L,tr,xe_L_Fast);
title({'Figure 5: Tracking Errors in the X and Y Axes as Functions of Time'
,'for High and Low Feedrate Trajectories using a LBW Controller (F.2)'});
ylabel('Tracking Error (x-axis) (mm)');
legend('Low F','High F','Location','SouthEast');
grid();

subplot(2,1,2);
plot(tr,ye_L,tr,ye_L_Fast);
xlabel(' Time (sec)'); ylabel('Tracking Error (y-axis)  (mm)');
grid();

%% Part F.3
figure(6);
sgtitle({'Figure 6: Tool Motion for Reference Toolpath and',
    'Simulations (LBW) using The Low and High Feedrates (F.3)'});
subplot(2,2,1);
plot(xa_L,ya_L,xa_L_Fast,ya_L_Fast,xr,yr,'--g');
title('R1')
ylabel('y (mm)');
axis([19.995 20.005 14.93 15.01])
grid;

subplot(2,2,2);
plot(xa_L,ya_L,xa_L_Fast,ya_L_Fast,xr,yr,'--g');
title('R2')
axis([39.95 40.25 29.95 30.25])
grid;

subplot(2,2,3);
plot(xa_L,ya_L,xa_L_Fast,ya_L_Fast,xr,yr,'--g');
title('R3')
xlabel(' x (mm'); ylabel('y (mm)');
legend('Low F','High F','Ref');
axis([59.85 60.3 29.6 30.05])
grid;

subplot(2,2,4);
plot(xa_L,ya_L,xa_L_Fast,ya_L_Fast,xr,yr,'--g');
title('R4')
xlabel(' x (mm');
axis([104.95 105.04 55.9 56.7])
grid;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part G - Experiment vs Simulation
%% Part G.1
%%% Tracking Error
load('lli_', 'lli');
t_exp=lli.X.Data;
xe_exp=lli.Y(3).Data-lli.Y(1).Data; ye_exp=lli.Y(4).Data-lli.Y(2).Data;

figure(7);
subplot(2,1,1);
plot(tr,xe_L,t_exp(8616:28615)-0.8614,xe_exp(8616:28615));
title({'Figure 7: Tracking Errors in the X and Y Axes as Functions of Time'
    ,'for the Experiment and Simulation using the LBW Controller (G.1)'});
ylabel('Tracking Error (x-axis) (mm)');
legend('Simulation','Experiment','Location','SouthEast');
grid();

subplot(2,1,2);
plot(tr,ye_L,t_exp(8616:28615)-0.8614,ye_exp(8616:28615));
xlabel(' Time (sec)'); ylabel('Tracking Error (y-axis)  (mm)');
grid();

%%% Toolpaths
figure(8);
sgtitle({'Figure 8: Tool Motion for Reference, Simulated',
    ' and Experimental Toolpaths using the LBW Controller (G.1)'});
subplot(2,2,1);
plot(xa_L,ya_L,lli.Y(1).Data,lli.Y(2).Data,xr,yr,'--g');
title('R1')
ylabel('y (mm)');
axis([19.995 20.005 14.93 15.01])
grid;

subplot(2,2,2);
plot(xa_L,ya_L,lli.Y(1).Data,lli.Y(2).Data,xr,yr,'--g');
title('R2')
axis([39.95 40.25 29.95 30.25])
grid;

subplot(2,2,3);
plot(xa_L,ya_L,lli.Y(1).Data,lli.Y(2).Data,xr,yr,'--g');
title('R3')
xlabel(' x (mm'); ylabel('y (mm)');
legend('Simulation','Experiment','Ref');
axis([59.85 60.3 29.6 30.05])
grid;

subplot(2,2,4);
plot(xa_L,ya_L,lli.Y(1).Data,lli.Y(2).Data,xr,yr,'--g');
title('R4')
xlabel(' x (mm');
axis([104.95 105.04 55.9 56.5])
grid;

%% Part G.2
var_x=[transpose(traj.t) transpose(traj.x)];
var_y=[transpose(traj.t) transpose(traj.y)];

contx_Z=LLI_L_x_Z;
conty_Z=LLI_L_y_Z;
stop_time=15;
out_L_s=sim('Part_C1.slx');
bdclose all;
tr_s = out_L_s.simout.time;
xr_s = out_L_s.simout.signals.values(:,2);
yr_s = out_L_s.simout.signals.values(:,4);
xa_s = out_L_s.simout.signals.values(:,3);
ya_s = out_L_s.simout.signals.values(:,5);

load('lli_mytraj_', 'lli');

%%% Toolpaths
figure(9)
plot(xa_s,ya_s,lli.Y(1).Data,lli.Y(2).Data,xr_s,yr_s,'--g');
title({'Figure 9: My Reference, Simulated and Measured Toolpath',
    'with Locations of Critical Regions (G.2)'});
legend('Simulation','Experiment','Ref','Location','SouthEast');
axis([0 100 0 100])
xlabel('x (mm)'); ylabel('y (mm)');
grid;
dim = [.49 .86 .05 .05];str = 'R6';
annotation('textbox',dim,'String',str)
dim = [.72 .53 .05 .05];str = 'R5';
annotation('textbox',dim,'String',str)
dim = [.68 .71 .05 .05];str = 'R4';
annotation('textbox',dim,'String',str)
dim = [.52 .2 .05 .05];str = 'R3';
annotation('textbox',dim,'String',str)
dim = [.56 .14 .05 .05];str = 'R2';
annotation('textbox',dim,'String',str)
dim = [.49 .1 .05 .05];str = 'R1';
annotation('textbox',dim,'String',str)

%%% Toolpaths (Zoomed views of critical regions)
figure(10);
sgtitle({'Figure 10: Tool Motion of My Trajectory for Reference, Simulated'
    ,' and Experimental Toolpaths using the LBW Controller (G.2)'});
subplot(3,2,1);
plot(xa_s,ya_s,lli.Y(1).Data,lli.Y(2).Data,xr_s,yr_s,'--g');
title('R1')
ylabel('y (mm)');
legend('Simulation','Experiment','Ref');
axis([48.5 51.5 -0.05 0.02])
grid;

subplot(3,2,2);
plot(xa_s,ya_s,lli.Y(1).Data,lli.Y(2).Data,xr_s,yr_s,'--g');
title('R2')
axis([57.4 57.65 6.5 9])
grid;

subplot(3,2,3);
plot(xa_s,ya_s,lli.Y(1).Data,lli.Y(2).Data,xr_s,yr_s,'--g');
title('R3')
ylabel('y (mm)');
axis([54.36 54.415 13.54 13.72])
grid;

subplot(3,2,4);
plot(xa_s,ya_s,lli.Y(1).Data,lli.Y(2).Data,xr_s,yr_s,'--g');
title('R4')
axis([74.999 75.001 77.86 78.04])
grid;

subplot(3,2,5);
plot(xa_s,ya_s,lli.Y(1).Data,lli.Y(2).Data,xr_s,yr_s,'--g');
title('R5')
xlabel(' x (mm'); ylabel('y (mm)');
axis([79.34 79.5 56.035 56.07])
grid;

subplot(3,2,6);
plot(xa_s,ya_s,lli.Y(1).Data,lli.Y(2).Data,xr_s,yr_s,'--g');
title('R6')
xlabel(' x (mm');
axis([49.9 50.01 98.52 98.64])
grid;

%%Contouring error plot
xe_s=xr_s-xa_s; ye_s=yr_s-ya_s;

t_exp=lli.X.Data;
xe_exp=lli.Y(3).Data-lli.Y(1).Data; ye_exp=lli.Y(4).Data-lli.Y(2).Data;

theta = atan(3/4);
for k=1:length(xe_s)
    leR1 = sqrt((xe_s(k))^2+(ye_s(k))^2);
    alpha = atan(ye_s(k)/xe_s(k));
    beta = alpha - theta;
    Ce_s(k) = leR1*sin(beta);
end

for k=1:length(xe_exp) 
    leR1 = sqrt((xe_exp(k))^2+(ye_exp(k))^2);
    alpha = atan(ye_exp(k)/xe_exp(k));
    beta = alpha - theta;
    Ce_exp(k) = leR1*sin(beta);
end

figure(11)
plot(tr_s,Ce_s,t_exp(17138:17138+length(xe_s))-1.7136, ...
    Ce_exp(17138:17138+length(xe_s)))
title({'Figure 11: Contouring Error as a Function of Time',
    'for the Experiment and Simulation using the LBW Controller (G.2)'});
xlabel(' Time (sec)'); ylabel('Contouring Error (mm)');
legend('Simulation','Experiment','Location','SouthEast');
grid();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part H - Pole Placement Controller
%%%Simulated Bandwidth, Rise Time, and Overshoot 
stepinfo(Gcl_P_x);
bandwidth(Gcl_P_x);

stepinfo(Gcl_P_x_Z);
bandwidth(Gcl_P_x_Z);

stepinfo(Gcl_P_y);
bandwidth(Gcl_P_y);

stepinfo(Gcl_P_y_Z);
bandwidth(Gcl_P_y_Z);

%%%Tracking Errors
xe_P=xr-xa_P; ye_P=yr-ya_P;

figure(12);
subplot(2,1,1);
plot(tr,xe_L,tr,xe_P);
title({'Figure 12:Tracking Errors in the X and Y Axes as Functions of Time'
    ,'for the LBW and PP Controllers (H)'});
ylabel('Tracking Error (x-axis) (mm)');
legend('LBW','PP','Location','SouthEast');
grid();

subplot(2,1,2);
plot(tr,ye_L,tr,ye_P);
xlabel(' Time (sec)'); ylabel('Tracking Error (y-axis)  (mm)');
grid();

%%%Contouring Performance
figure(13);
sgtitle({'Figure 13: Tool Motion for Reference Toolpath and',
    'Simulations using the LBW and PP Controllers (H)'});
subplot(2,2,1);
plot(xa_L,ya_L,'r',xa_P,ya_P,'b',xr,yr,'g--');
title('R1')
ylabel('y (mm)');
axis([19.9975 20.002 14.93 15.01])
grid;

subplot(2,2,2);
plot(xa_L,ya_L,'r',xa_P,ya_P,'b',xr,yr,'g--');
title('R2')
axis([39.95 40.25 29.95 30.25])
grid;

subplot(2,2,3);
plot(xa_L,ya_L,'r',xa_P,ya_P,'b',xr,yr,'g--');
title('R3')
xlabel(' x (mm'); ylabel('y (mm)');
legend('LBW','PP','Ref');
axis([59.85 60.3 29.6 30.05])
grid;

subplot(2,2,4);
plot(xa_L,ya_L,'r',xa_P,ya_P,'b',xr,yr,'g--');
title('R4')
xlabel(' x (mm');
axis([104.98 105.02 55.95 56.5])
grid;

%%%Experimental Line Circle Trajectory
load('pp_', 'pp');
t_exp=pp.X.Data;
xa_exp=pp.Y(1).Data; ya_exp=pp.Y(2).Data;

figure(14)
plot(xr,yr,'--g',xa_exp,ya_exp);
title('Figure14:The Reference and Experimental Line Circle Trajectory(H)');
legend('Ref','Experiment','Location','NorthEast');
ylim([0 90])
xlabel('x (mm)'); ylabel('y (mm)');
grid;

%%% Experimental vs Simulated Responses for my Designed PP Controller on my
%%% Generated Trajectory
var_x=[transpose(traj.t) transpose(traj.x)];
var_y=[transpose(traj.t) transpose(traj.y)];

t0_y_Z=t0_y_Z_mytraj;
R_y_Z=R_y_Z_mytraj;
s_y_Z=s_y_Z_mytraj;

stop_time=15;
out_P_s=sim('Part_H.slx');
bdclose all;
tr_s = out_P_s.simout.time;
xr_s = out_P_s.simout.signals.values(:,2);
yr_s = out_P_s.simout.signals.values(:,4);
xa_P_s = out_P_s.simout.signals.values(:,3);
ya_P_s = out_P_s.simout.signals.values(:,5);

load('pp_mytraj_', 'pp');
t_exp=pp.X.Data;
xa_exp=pp.Y(1).Data; ya_exp=pp.Y(2).Data;

figure(15)
plot(tr_s,xa_P_s,'--',t_exp(17431:17431+length(xr_s))-1.7430, ...
xa_exp(17431:17431+length(xr_s)),tr_s,ya_P_s,'--', ...
t_exp(17431:17431+length(xr_s))-1.7430,ya_exp(17431:17431+length(xr_s)))
title({'Figure 15: Simulated and Experimental Responses for My Trajectory',
    'using My Designed PP (H)'})
xlabel(' Time (sec)'); ylabel('Displacement (mm)');
legend('x(sim)','x(exp)','y(sim)','y(exp)','Location','SouthEast')
grid();