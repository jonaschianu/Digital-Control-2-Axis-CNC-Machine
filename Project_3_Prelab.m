%% Matlab Code for Project 3
% PRE-LAB
% MECH 541
% University of British Columbia
% Jonas Chianu
% 31298391
% jchianu@student.ubc.ca
% November 2020

close all; bdclose all; clear all;
Ts=0.1/1000; Ti=Ts;
Fs=0; Fe=0;
A=1000; D=-1000;
Px1=0;Py1=0;
Px2=40;Py2=30;
Px3=60;Py3=30;
Pcx4=90;Pcy4=30;
fraction_L_4=1; % Fraction of full circle circumference
fraction_theta0_4=1; % Fraction of starting point, pi()
Rotation_4=1; % 1: CCW, -1: CW 
t0=0; L0=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part A - Trajectory Generation
F=200;

[t1,x1,y1,dx1,dy1,dxdx1,dydy1,l1,f1,a1] =linTraj(Px1,Py1,Px2,Py2,Fs,Fe,A,D,F,Ti,t0,L0);
[t2,x2,y2,dx2,dy2,dxdx2,dydy2,l2,f2,a2] =linTraj(Px2,Py2,Px3,Py3,Fs,Fe,A,D,F,Ti,t1(end),l1(end));
[t3,x3,y3,dx3,dy3,dxdx3,dydy3,l3,f3,a3] =cirTraj(Px3,Py3,Pcx4,Pcy4,Fs,Fe,A,D,F,Ti,t2(end),l2(end),fraction_L_4,fraction_theta0_4,Rotation_4);

t=[0,t1,t2,t3];

l=[0,l1,l2,l3];
f=[0,f1,f2,f3];
a=[0,a1,a2,a3];

x=[0,x1,x2,x3];
y=[0,y1,y2,y3];

dx=[0,dx1,dx2,dx3];
dy=[0,dy1,dy2,dy3];

dxdx=[0,dxdx1,dxdx2,dxdx3];
dydy=[0,dydy1,dydy2,dydy3];

% First row is time and second row is displacement
var_x_1=[transpose(t) transpose(x)];
var_x=var_x_1;

var_y_1=[transpose(t) transpose(y)];
var_y=var_y_1;

%% Part A.1
figure(1);
plot(x,y);
title('Figure 1: The Generated Toolpath (A.1)');
ylim([0 90]);
xlabel(' Xr (mm)'); ylabel('Yr (mm)');
grid;

%% Part A.2
figure(2)
subplot(3,1,1)
plot(t,l)
title({'Figure 2: The Displacement, Feedrate, and Tangential Acceleration Profiles','as Functions of Time (A.2)'})
ylabel('Displacement (mm)');
grid();
subplot(3,1,2)
plot(t,f)
ylabel('Velocity (mm/s)');
grid();
subplot(3,1,3)
plot(t,a)
xlabel(' Time (sec)'); ylabel('Acceleration (mm/s^2)');
grid();

%% Part A.3
figure(3)
subplot(3,1,1)
plot(t,x,t,y)
title({'Figure 3: The Axis Position, Velocity, and Acceleration Commands','as Functions of Time (A.3)'})
ylabel('Axis Position (mm)');
legend('x','y','Location','NorthEast')
grid();
subplot(3,1,2)
plot(t,dx,t,dy)
ylabel('Axis Velocity (mm/s)');
grid();
subplot(3,1,3)
plot(t,dxdx,t,dydy)
xlabel(' Time (sec)'); ylabel('Axis Acceleration (mm/s^2)');
grid();


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part A (Fast)
F=250;

[t1,x1,y1,dx1,dy1,dxdx1,dydy1,l1,f1,a1] =linTraj(Px1,Py1,Px2,Py2,Fs,Fe,A,D,F,Ti,t0,L0);
[t2,x2,y2,dx2,dy2,dxdx2,dydy2,l2,f2,a2] =linTraj(Px2,Py2,Px3,Py3,Fs,Fe,A,D,F,Ti,t1(end),l1(end));
[t3,x3,y3,dx3,dy3,dxdx3,dydy3,l3,f3,a3] =cirTraj(Px3,Py3,Pcx4,Pcy4,Fs,Fe,A,D,F,Ti,t2(end),l2(end),fraction_L_4,fraction_theta0_4,Rotation_4);

t_fast=[0,t1,t2,t3];

l_fast=[0,l1,l2,l3];
f_fast=[0,f1,f2,f3];
a_fast=[0,a1,a2,a3];

x_fast=[0,x1,x2,x3];
y_fast=[0,y1,y2,y3];

dx_fast=[0,dx1,dx2,dx3];
dy_fast=[0,dy1,dy2,dy3];

dxdx_fast=[0,dxdx1,dxdx2,dxdx3];
dydy_fast=[0,dydy1,dydy2,dydy3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part B – Two-Axis Controller Design
Ka=1;Kt=0.49;Ke=1.59;
Je_x=4.36E-4;Je_y=3E-4;
B_x=0.0094;B_y=0.0091;

s=tf('s'); z = tf('z',Ts);

G_x=Ka*Kt*Ke/(s*(Je_x*s+B_x));
G_x_Z=c2d(G_x,Ts);

G_y=Ka*Kt*Ke/(s*(Je_y*s+B_y));
G_y_Z=c2d(G_y,Ts);

%% Part B.1
%%Low BW
wc=20*2*pi();
[~,phase] = bode(G_x, wc);

%Lead-Lag compensator
phim=(60-(180+phase))*(pi()/180); % added phase
wm=wc; % Add phase lead at this frequency
a=(1+sin(phim ))/(1-sin(phim )); % Design alpha
Tc=1/(wm*sqrt(a)); % design T
C=tf([a*Tc 1],[Tc 1]); % Compensator TF

% integral action
Ki=wc/10;
I=(Ki+s)/s;

%LLI_x
[mag,~] = bode(G_x*C, wc);
K_x=1/mag;
KC_x=K_x*C;

LLI_L_x=KC_x*I;
LLI_L_x_Z=c2d(LLI_L_x,Ts,'tustin');

G_L_x=LLI_L_x*G_x;
G_L_x_Z=LLI_L_x_Z*G_x_Z;

%LLI_y
[mag,~] = bode(G_y*C, wc);
K_y=1/mag;
KC_y=K_y*C;

LLI_L_y=KC_y*I;
LLI_L_y_Z=c2d(LLI_L_y,Ts,'tustin');

G_L_y=LLI_L_y*G_y;
G_L_y_Z=LLI_L_y_Z*G_y_Z;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%High BW
wc=40*2*pi();
[~,phase] = bode(G_x, wc);

%Lead-Lag compensator
phim=(60-(180+phase))*(pi()/180); % added phase
wm=wc; % Add phase lead at this frequency
a=(1+sin(phim ))/(1-sin(phim )); % Design alpha
Tc=1/(wm*sqrt(a)); % design T
C=tf([a*Tc 1],[Tc 1]); % Compensator TF

% integral action
Ki=wc/10;
I=(Ki+s)/s;

%LLI_x
[mag,~] = bode(G_x*C, wc);
K_x=1/mag;
KC_x=K_x*C;

LLI_H_x=KC_x*I;
LLI_H_x_Z=c2d(LLI_H_x,Ts,'tustin');

G_H_x=LLI_H_x*G_x;
G_H_x_Z=LLI_H_x_Z*G_x_Z;

%LLI_y
[mag,~] = bode(G_y*C, wc);
K_y=1/mag;
KC_y=K_y*C;

LLI_H_y=KC_y*I;
LLI_H_y_Z=c2d(LLI_H_y,Ts,'tustin');

G_H_y=LLI_H_y*G_y;
G_H_y_Z=LLI_H_y_Z*G_y_Z;

%% Part B.1 - Pole Placement
zeta_m=0.7; wn_m=20*2*pi ; % Desired damping ratio and natural frequency
s1m=-zeta_m*wn_m+i*wn_m*sqrt(1-zeta_m^2);
s2m=-zeta_m*wn_m-i*wn_m*sqrt(1-zeta_m^2);
z1_m=exp(s1m*Ts); z2_m=exp(s2m*Ts);
m1=-(z1_m+z2_m);
m2=z1_m*z2_m;
Am=[1 m1 m2]; % Desired characteristic eqn. of the closed loop controller

%%PP_x
b0=G_x_Z.num{(1)}(2);
b1=G_x_Z.num{(1)}(3);
a1=G_x_Z.den{(1)}(2);
a2=G_x_Z.den{(1)}(3);

% Design pole placement controller
bm0=(Am(1)+Am(2)+Am(3))/(b0+b1);
t0_x_Z=bm0;
A=[1 b0 0;a1 b1 b0;a2 0 b1];
B=[m1-a1;m2-a2;0];
teta=A^-1*B;
r1=teta(1);
s0=teta(2);
s1=teta(3);

R_x_Z = 1+r1*z^-1;
s_x_Z = s0+s1*z^-1;

t0_x=t0_x_Z;
R_x=d2c(R_x_Z,'tustin');
s_x=d2c(s_x_Z,'tustin');

G_P_x_Z=G_x_Z/R_x_Z;
G_P_x=G_x/R_x;

%%PP_y
b0=G_y_Z.num{(1)}(2);
b1=G_y_Z.num{(1)}(3);
a1=G_y_Z.den{(1)}(2);
a2=G_y_Z.den{(1)}(3);

% Design pole placement controller
bm0=(Am(1)+Am(2)+Am(3))/(b0+b1);
t0_y_Z=bm0;
A=[1 b0 0;a1 b1 b0;a2 0 b1];
B=[m1-a1;m2-a2;0];
teta=A^-1*B;
r1=teta(1);
s0=teta(2);
s1=teta(3);

R_y_Z = 1+r1*z^-1;
s_y_Z = s0+s1*z^-1;

t0_y=t0_y_Z;
R_y=d2c(R_y_Z,'tustin');
s_y=d2c(s_y_Z,'tustin');

G_P_y_Z=G_y_Z/R_y_Z;
G_P_y=G_y/R_y;

stop_time=2;
out_P=sim('Part_H.slx');
tr = out_P.simout.time;
xr = out_P.simout.signals.values(:,2);
yr = out_P.simout.signals.values(:,4);
xa_P = out_P.simout.signals.values(:,3);
ya_P = out_P.simout.signals.values(:,5);

%% Part B.1 - Pole Placement (My Traj. design)
zeta_m=0.7; wn_m=30*2*pi ; % Desired damping ratio and natural frequency
s1m=-zeta_m*wn_m+i*wn_m*sqrt(1-zeta_m^2);
s2m=-zeta_m*wn_m-i*wn_m*sqrt(1-zeta_m^2);
z1_m=exp(s1m*Ts); z2_m=exp(s2m*Ts);
m1=-(z1_m+z2_m);
m2=z1_m*z2_m;
Am=[1 m1 m2]; % Desired characteristic eqn. of the closed loop controller

%%PP_x
b0=G_x_Z.num{(1)}(2);
b1=G_x_Z.num{(1)}(3);
a1=G_x_Z.den{(1)}(2);
a2=G_x_Z.den{(1)}(3);

% Design pole placement controller
bm0=(Am(1)+Am(2)+Am(3))/(b0+b1);
t0_x_Z_mytraj=bm0;
A=[1 b0 0;a1 b1 b0;a2 0 b1];
B=[m1-a1;m2-a2;0];
teta=A^-1*B;
r1=teta(1);
s0=teta(2);
s1=teta(3);

R_x_Z_mytraj = 1+r1*z^-1;
s_x_Z_mytraj = s0+s1*z^-1;

%%PP_y
b0=G_y_Z.num{(1)}(2);
b1=G_y_Z.num{(1)}(3);
a1=G_y_Z.den{(1)}(2);
a2=G_y_Z.den{(1)}(3);

% Design pole placement controller
bm0=(Am(1)+Am(2)+Am(3))/(b0+b1);
t0_y_Z_mytraj=bm0;
A=[1 b0 0;a1 b1 b0;a2 0 b1];
B=[m1-a1;m2-a2;0];
teta=A^-1*B;
r1=teta(1);
s0=teta(2);
s1=teta(3);

R_y_Z_mytraj = 1+r1*z^-1;
s_y_Z_mytraj = s0+s1*z^-1;


%% Part B.2
figure(4);
bode(G_L_x,G_H_x);
title({'Figure 4: Open Loop Continuous System of X axis','with the LBW and HBW Controllers (B.2)'})
legend('LBW','HBW','Location','NorthEast')
grid;

Gcl_L_x=feedback(G_L_x,1);
Gcl_L_x_Z=feedback(G_L_x_Z,1);

Gcl_H_x=feedback(G_H_x,1);
Gcl_H_x_Z=feedback(G_H_x_Z,1);

figure(5)
bode(Gcl_L_x,Gcl_H_x);
title({'Figure 5: Closed Loop Continuous System of X axis','with the LBW and HBW Controllers (B.2)'})
legend('LBW','HBW','Location','NorthEast')
grid;

Gcl_L_y=feedback(G_L_y,1);
Gcl_L_y_Z=feedback(G_L_y_Z,1);

Gcl_H_y=feedback(G_H_y,1);
Gcl_H_y_Z=feedback(G_H_y_Z,1);

%% Part B.2 - Pole Placement
Gcl_P_x_Z=t0_x_Z*feedback(G_P_x_Z,s_x_Z);
Gcl_P_x=t0_x*feedback(G_P_x,s_x);

Gcl_P_y_Z=t0_y_Z*feedback(G_P_y_Z,s_y_Z);
Gcl_P_y=t0_y*feedback(G_P_y,s_y);

%% Part B.3
pole(Gcl_L_x);
zero(Gcl_L_x);
stepinfo(Gcl_L_x);
bandwidth(Gcl_L_x);

pole(Gcl_L_x_Z);
zero(Gcl_L_x_Z);
stepinfo(Gcl_L_x_Z);
bandwidth(Gcl_L_x_Z);

pole(Gcl_H_x);
zero(Gcl_H_x);
stepinfo(Gcl_H_x);
bandwidth(Gcl_H_x);

pole(Gcl_H_x_Z);
zero(Gcl_H_x_Z);
stepinfo(Gcl_H_x_Z);
bandwidth(Gcl_H_x_Z);

pole(Gcl_L_y);
zero(Gcl_L_y);
stepinfo(Gcl_L_y);
bandwidth(Gcl_L_y);

pole(Gcl_L_y_Z);
zero(Gcl_L_y_Z);
stepinfo(Gcl_L_y_Z);
bandwidth(Gcl_L_y_Z);

pole(Gcl_H_y);
zero(Gcl_H_y);
stepinfo(Gcl_H_y);
bandwidth(Gcl_H_y);

pole(Gcl_H_y_Z);
zero(Gcl_H_y_Z);
stepinfo(Gcl_H_y_Z);
bandwidth(Gcl_H_y_Z);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part C – Contouring Performance Simulation
%% Part C.1
%LBW Controllers
stop_time=2;
contx_Z=LLI_L_x_Z;
conty_Z=LLI_L_y_Z;
out_L=sim('Part_C1.slx');
tr = out_L.simout.time;
xr = out_L.simout.signals.values(:,2);
yr = out_L.simout.signals.values(:,4);
xa_L = out_L.simout.signals.values(:,3);
ya_L = out_L.simout.signals.values(:,5);

%HBW Controllers
contx_Z=LLI_H_x_Z;
conty_Z=LLI_H_y_Z;
out_H=sim('Part_C1.slx');
xa_H = out_H.simout.signals.values(:,3);
ya_H = out_H.simout.signals.values(:,5);

%Mismatched Dynamics Controllers
contx_Z=LLI_H_x_Z;
conty_Z=LLI_L_y_Z;
out_M=sim('Part_C1.slx');
xa_M = out_M.simout.signals.values(:,3);
ya_M = out_M.simout.signals.values(:,5);

figure(6)
subplot(3,1,1)
plot(tr,xr,'--',tr,xa_L,tr,yr,'--',tr,ya_L)
title({'Figure 7: Reference Trajectory vs Simulated Trajectory','()r: Reference, ()a: Simulated (C.1)'})
ylabel('Displacement (LBW) (mm)');
legend('xr','xa','yr','ya','Location','SouthEast')
grid();
subplot(3,1,2)
plot(tr,xr,'--',tr,xa_H,tr,yr,'--',tr,ya_H)
ylabel('Displacement (HBW) (mm)');
grid();
subplot(3,1,3)
plot(tr,xr,'--',tr,xa_M,tr,yr,'--',tr,ya_M)
xlabel(' Time (sec)'); ylabel('Displacement (MBW) (mm)');
grid();

%% Part C.2
F=100;Fs=0;Fe=0;
A=250;D=-250;
Px1=0;Py1=0;
Px2=50;Py2=0;
Pcx3=50;Pcy3=7.5;
fraction_L_3=0.4;fraction_theta0_3=3/2;Rotation_3=1;
t0=0;L0=0;

[t1,x1,y1,dx1,dy1,dxdx1,dydy1,l1,f1,a1] =linTraj(Px1,Py1,Px2,Py2,Fs,Fe,A,D,F,Ti,t0,L0);
[t2,x2,y2,dx2,dy2,dxdx2,dydy2,l2,f2,a2] =cirTraj(Px2,Py2,Pcx3,Pcy3,Fs,Fe,A,D,F,Ti,t1(end),l1(end),fraction_L_3,fraction_theta0_3,Rotation_3);

Px3=x2(end);Py3=y2(end);
Px4=Px3;Py4=Py3+10;
Px5=Px4+20;Py5=Py4;
Px6=Px5;Py6=Py5+7.5;
Px7=Px4;Py7=Py6;
Px8=Px7;Py8=Py7+55;
Pcx9=Px8;Pcy9=Py8-30;
fraction_L_9=0.25;fraction_theta0_9=1/2;Rotation_9=-1;

[t3,x3,y3,dx3,dy3,dxdx3,dydy3,l3,f3,a3] =linTraj(Px3,Py3,Px4,Py4,Fs,Fe,A,D,F,Ti,t2(end),l2(end));
[t4,x4,y4,dx4,dy4,dxdx4,dydy4,l4,f4,a4] =linTraj(Px4,Py4,Px5,Py5,Fs,Fe,A,D,F,Ti,t3(end),l3(end));
[t5,x5,y5,dx5,dy5,dxdx5,dydy5,l5,f5,a5] =linTraj(Px5,Py5,Px6,Py6,Fs,Fe,A,D,F,Ti,t4(end),l4(end));
[t6,x6,y6,dx6,dy6,dxdx6,dydy6,l6,f6,a6] =linTraj(Px6,Py6,Px7,Py7,Fs,Fe,A,D,F,Ti,t5(end),l5(end));
[t7,x7,y7,dx7,dy7,dxdx7,dydy7,l7,f7,a7] =linTraj(Px7,Py7,Px8,Py8,Fs,Fe,A,D,F,Ti,t6(end),l6(end));
[t8,x8,y8,dx8,dy8,dxdx8,dydy8,l8,f8,a8] =cirTraj(Px8,Py8,Pcx9,Pcy9,Fs,Fe,A,D,F,Ti,t7(end),l7(end),fraction_L_9,fraction_theta0_9,Rotation_9);

Px9=x8(end);Py9=y8(end);
Px10=Px9-5;Py10=Py9;
Px11=Px9+3.75;Py11=Py9-5;
Px12=Px9+12.5;Py12=Py9;
Px13=Px9+7.5;Py13=Py9;
Pcx14=Px8;Pcy14=Py13;
fraction_L_14=0.25;fraction_theta0_14=0;Rotation_14=1;

[t9,x9,y9,dx9,dy9,dxdx9,dydy9,l9,f9,a9] =linTraj(Px9,Py9,Px10,Py10,Fs,Fe,A,D,F,Ti,t8(end),l8(end));
[t10,x10,y10,dx10,dy10,dxdx10,dydy10,l10,f10,a10] =linTraj(Px10,Py10,Px11,Py11,Fs,Fe,A,D,F,Ti,t9(end),l9(end));
[t11,x11,y11,dx11,dy11,dxdx11,dydy11,l11,f11,a11] =linTraj(Px11,Py11,Px12,Py12,Fs,Fe,A,D,F,Ti,t10(end),l10(end));
[t12,x12,y12,dx12,dy12,dxdx12,dydy12,l12,f12,a12] =linTraj(Px12,Py12,Px13,Py13,Fs,Fe,A,D,F,Ti,t11(end),l11(end));
[t13,x13,y13,dx13,dy13,dxdx13,dydy13,l13,f13,a13] =cirTraj(Px13,Py13,Pcx14,Pcy14,Fs,Fe,A,D,F,Ti,t12(end),l12(end),fraction_L_14,fraction_theta0_14,Rotation_14);

Px14=x13(end);Py14=y13(end);
Px15=50;Py15=Py14+5;

%--------------------mirror
Px16=50-(Px14-50);Py16=Py14;
Pcx17=50-(Pcx14-50);Pcy17=Pcy14;
fraction_L_17=fraction_L_14;fraction_theta0_17=1/2;Rotation_17=1;
Px17=50-(Px13-50);Py17=Py13;
Px18=50-(Px12-50);Py18=Py12;
Px19=50-(Px11-50);Py19=Py11;
Px20=50-(Px10-50);Py20=Py10;
Px21=50-(Px9-50);Py21=Py9;
Pcx22=50-(Pcx9-50);Pcy22=Pcy9;
fraction_L_22=fraction_L_9;fraction_theta0_22=1;Rotation_22=-1;
Px22=50-(Px8-50);Py22=Py8;
Px23=50-(Px7-50);Py23=Py7;
Px24=50-(Px6-50);Py24=Py6;
Px25=50-(Px5-50);Py25=Py5;
Px26=50-(Px4-50);Py26=Py4;
Px27=50-(Px3-50);Py27=Py3;
Pcx28=50-(Pcx3-50);Pcy28=Pcy3;
fraction_L_28=fraction_L_3;fraction_theta0_28=(pi()-(2*pi()*fraction_L_3-(pi()/2)))/pi();Rotation_28=1;

[t14,x14,y14,dx14,dy14,dxdx14,dydy14,l14,f14,a14] =linTraj(Px14,Py14,Px15,Py15,Fs,Fe,A,D,F,Ti,t13(end),l13(end));
[t15,x15,y15,dx15,dy15,dxdx15,dydy15,l15,f15,a15] =linTraj(Px15,Py15,Px16,Py16,Fs,Fe,A,D,F,Ti,t14(end),l14(end));
[t16,x16,y16,dx16,dy16,dxdx16,dydy16,l16,f16,a16] =cirTraj(Px16,Py16,Pcx17,Pcy17,Fs,Fe,A,D,F,Ti,t15(end),l15(end),fraction_L_17,fraction_theta0_17,Rotation_17);
[t17,x17,y17,dx17,dy17,dxdx17,dydy17,l17,f17,a17] =linTraj(Px17,Py17,Px18,Py18,Fs,Fe,A,D,F,Ti,t16(end),l16(end));
[t18,x18,y18,dx18,dy18,dxdx18,dydy18,l18,f18,a18] =linTraj(Px18,Py18,Px19,Py19,Fs,Fe,A,D,F,Ti,t17(end),l17(end));
[t19,x19,y19,dx19,dy19,dxdx19,dydy19,l19,f19,a19] =linTraj(Px19,Py19,Px20,Py20,Fs,Fe,A,D,F,Ti,t18(end),l18(end));
[t20,x20,y20,dx20,dy20,dxdx20,dydy20,l20,f20,a20] =linTraj(Px20,Py20,Px21,Py21,Fs,Fe,A,D,F,Ti,t19(end),l19(end));
[t21,x21,y21,dx21,dy21,dxdx21,dydy21,l21,f21,a21] =cirTraj(Px21,Py21,Pcx22,Pcy22,Fs,Fe,A,D,F,Ti,t20(end),l20(end),fraction_L_22,fraction_theta0_22,Rotation_22);
[t22,x22,y22,dx22,dy22,dxdx22,dydy22,l22,f22,a22] =linTraj(Px22,Py22,Px23,Py23,Fs,Fe,A,D,F,Ti,t21(end),l21(end));
[t23,x23,y23,dx23,dy23,dxdx23,dydy23,l23,f23,a23] =linTraj(Px23,Py23,Px24,Py24,Fs,Fe,A,D,F,Ti,t22(end),l22(end));
[t24,x24,y24,dx24,dy24,dxdx24,dydy24,l24,f24,a24] =linTraj(Px24,Py24,Px25,Py25,Fs,Fe,A,D,F,Ti,t23(end),l23(end));
[t25,x25,y25,dx25,dy25,dxdx25,dydy25,l25,f25,a25] =linTraj(Px25,Py25,Px26,Py26,Fs,Fe,A,D,F,Ti,t24(end),l24(end));
[t26,x26,y26,dx26,dy26,dxdx26,dydy26,l26,f26,a26] =linTraj(Px26,Py26,Px27,Py27,Fs,Fe,A,D,F,Ti,t25(end),l25(end));
[t27,x27,y27,dx27,dy27,dxdx27,dydy27,l27,f27,a27] =cirTraj(Px27,Py27,Pcx28,Pcy28,Fs,Fe,A,D,F,Ti,t26(end),l26(end),fraction_L_28,fraction_theta0_28,Rotation_28);

t=[0,t1,t2,t3,t4,t5,t6,t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18,t19,t20,t21,t22,t23,t24,t25,t26,t27];
l=[0,l1,l2,l3,l4,l5,l6,l7,l8,l9,l10,l11,l12,l13,l14,l15,l16,l17,l18,l19,l20,l21,l22,l23,l24,l25,l26,l27];
f=[0,f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12,f13,f14,f15,f16,f17,f18,f19,f20,f21,f22,f23,f24,f25,f26,f27];
a=[0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,a24,a25,a26,a27];
x=[0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27];
y=[0,y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11,y12,y13,y14,y15,y16,y17,y18,y19,y20,y21,y22,y23,y24,y25,y26,y27];
dx=[0,dx1,dx2,dx3,dx4,dx5,dx6,dx7,dx8,dx9,dx10,dx11,dx12,dx13,dx14,dx15,dx16,dx17,dx18,dx19,dx20,dx21,dx22,dx23,dx24,dx25,dx26,dx27];
dy=[0,dy1,dy2,dy3,dy4,dy5,dy6,dy7,dy8,dy9,dy10,dy11,dy12,dy13,dy14,dy15,dy16,dy17,dy18,dy19,dy20,dy21,dy22,dy23,dy24,dy25,dy26,dy27];
dxdx=[0,dxdx1,dxdx2,dxdx3,dxdx4,dxdx5,dxdx6,dxdx7,dxdx8,dxdx9,dxdx10,dxdx11,dxdx12,dxdx13,dxdx14,dxdx15,dxdx16,dxdx17,dxdx18,dxdx19,dxdx20,dxdx21,dxdx22,dxdx23,dxdx24,dxdx25,dxdx26,dxdx27];
dydy=[0,dydy1,dydy2,dydy3,dydy4,dydy5,dydy6,dydy7,dydy8,dydy9,dydy10,dydy11,dydy12,dydy13,dydy14,dydy15,dydy16,dydy17,dydy18,dydy19,dydy20,dydy21,dydy22,dydy23,dydy24,dydy25,dydy26,dydy27];

figure(7)
plot(x,y);
title('Figure 8: My Generated Toolpath (C.2)')
xlabel(' Xr (mm)'); ylabel('Yr (mm)');
grid;

traj.t=t; traj.x=x; traj.y=y;

save MyTraj_Anchor_JonasChianu traj;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Trajectory Generation Functions
function [t,x,y,dx,dy,dxdx,dydy,l,f,a] =linTraj(Px1,Py1,Px2,Py2,Fs,Fe,A,D,F,Ti,t0,L0)
    L=sqrt((Px2-Px1)^2+(Py2-Py1)^2);

    T1=(F-Fs)/A;
    T3=(Fe-F)/D;
    T2=(L/F)-(((1/(2*A))-(1/(2*D)))*F-(((Fe^2)/(2*A))-((Fs^2)/(2*D)))*(1/F));

    if T2<0
        T2=0;
        F=sqrt((2*A*D*L-((Fe^2)*A-(Fs^2)*D))/(D-A));
        T1=(F-Fs)/A;
        T3=(Fe-F)/D;
    end

    N1=ceil(T1/Ti);
    N2=ceil(T2/Ti);
    N3=ceil(T3/Ti);

    if (T1/Ti) ~= N1 || (T2/Ti) ~= N2 || (T3/Ti) ~= N3
        if (T1/Ti) ~= N1
            T1=N1*Ti;
        end
        if (T2/Ti) ~= N2
            T2=N2*Ti;
        end
        if (T3/Ti) ~= N3
            T3=N3*Ti;
        end

        F=(2*L-Fs*T1-Fe*T3)/(T1+2*T2+T3);
        A=(F-Fs)/T1;
        D=(Fe-F)/T3;
    end

    for k=1:N1
        t(k) =k*Ti;
        l(k)=(1/2)*A*(k*Ti)^2+Fs*(k*Ti);
        f(k)=A*(k*Ti)+Fs;
        a(k)=A;

        x(k)=((Px2-Px1)/L)*l(k)+Px1;
        y(k)=((Py2-Py1)/L)*l(k)+Py1;

        dx(k)=((Px2-Px1)/L)*f(k);
        dy(k)=((Py2-Py1)/L)*f(k);

        dxdx(k)=((Px2-Px1)/L)*a(k);
        dydy(k)=((Py2-Py1)/L)*a(k);
    end

    for k = (N1+1):(N1+N2)
        t(k) = k*Ti;
        l(k) = l(N1) + F*(k-N1)*Ti;
        f(k)=F;
        a(k)=0;

        x(k)=((Px2-Px1)/L)*l(k)+Px1;
        y(k)=((Py2-Py1)/L)*l(k)+Py1;

        dx(k)=((Px2-Px1)/L)*f(k);
        dy(k)=((Py2-Py1)/L)*f(k);

        dxdx(k)=((Px2-Px1)/L)*a(k);
        dydy(k)=((Py2-Py1)/L)*a(k);
    end

    for k = (N1+N2+1):(N1+N2+N3)
        t(k) = k*Ti;
        l(k) = l(N1+N2) + (1/2)*D*((k-N1-N2)*Ti)^2+F*((k-N1-N2)*Ti);
        f(k)=F+D*(k-N1-N2)*Ti;
        a(k)=D;

        x(k)=((Px2-Px1)/L)*l(k)+Px1;
        y(k)=((Py2-Py1)/L)*l(k)+Py1;

        dx(k)=((Px2-Px1)/L)*f(k);
        dy(k)=((Py2-Py1)/L)*f(k);

        dxdx(k)=((Px2-Px1)/L)*a(k);
        dydy(k)=((Py2-Py1)/L)*a(k);
    end
    l=l+L0;
    t=t+t0;
end

function [t,x,y,dx,dy,dxdx,dydy,l,f,a] =cirTraj(Px1,Py1,Px2,Py2,Fs,Fe,A,D,F,Ti,t0,L0,fraction_L,fraction_theta0,Rot_dir)
    R=sqrt((Px2-Px1)^2+(Py2-Py1)^2);
    L=2*pi()*R*fraction_L;
    theta0=pi()*fraction_theta0;

    T1=(F-Fs)/A;
    T3=(Fe-F)/D;
    T2=(L/F)-(((1/(2*A))-(1/(2*D)))*F-(((Fe^2)/(2*A))-((Fs^2)/(2*D)))*(1/F));

    if T2<0
        T2=0;
        F=sqrt((2*A*D*L-((Fe^2)*A-(Fs^2)*D))/(D-A));
        T1=(F-Fs)/A;
        T3=(Fe-F)/D;
    end

    N1=ceil(T1/Ti);
    N2=ceil(T2/Ti);
    N3=ceil(T3/Ti);

    if (T1/Ti) ~= N1 || (T2/Ti) ~= N2 || (T3/Ti) ~= N3
        if (T1/Ti) ~= N1
            T1=N1*Ti;
        end
        if (T2/Ti) ~= N2
            T2=N2*Ti;
        end
        if (T3/Ti) ~= N3
            T3=N3*Ti;
        end
        F=(2*L-Fs*T1-Fe*T3)/(T1+2*T2+T3);
        A=(F-Fs)/T1;
        D=(Fe-F)/T3;
    end

    for k=1:N1
        t(k) =k*Ti;
        l(k)=(1/2)*A*(k*Ti)^2+Fs*(k*Ti);
        f(k)=A*(k*Ti)+Fs;
        a(k)=A;
        theta(k)=theta0+(l(k)/R)*Rot_dir;

        x(k)=Px2+R*cos(theta(k));
        y(k)=Py2+R*sin(theta(k));

        dx(k)=-f(k)*sin(theta(k));
        dy(k)=f(k)*cos(theta(k));

        dxdx(k)=-a(k)*sin(theta(k))-(1/R)*(f(k)^2)*cos(theta(k));
        dydy(k)=a(k)*cos(theta(k))-(1/R)*(f(k)^2)*sin(theta(k));
    end

    for k = (N1+1):(N1+N2)
        t(k) = k*Ti;
        l(k) = l(N1) + F*(k-N1)*Ti;
        f(k)=F;
        a(k)=0;
        theta(k)=theta0+(l(k)/R)*Rot_dir;

        x(k)=Px2+R*cos(theta(k));
        y(k)=Py2+R*sin(theta(k));

        dx(k)=-f(k)*sin(theta(k));
        dy(k)=f(k)*cos(theta(k));

        dxdx(k)=-a(k)*sin(theta(k))-(1/R)*(f(k)^2)*cos(theta(k));
        dydy(k)=a(k)*cos(theta(k))-(1/R)*(f(k)^2)*sin(theta(k));
    end

    for k = (N1+N2+1):(N1+N2+N3)
        t(k) = k*Ti;
        l(k) = l(N1+N2) + (1/2)*D*((k-N1-N2)*Ti)^2+F*((k-N1-N2)*Ti);
        f(k)=F+D*(k-N1-N2)*Ti;
        a(k)=D;
        theta(k)=theta0+(l(k)/R)*Rot_dir;

        x(k)=Px2+R*cos(theta(k));
        y(k)=Py2+R*sin(theta(k));

        dx(k)=-f(k)*sin(theta(k));
        dy(k)=f(k)*cos(theta(k));

        dxdx(k)=-a(k)*sin(theta(k))-(1/R)*(f(k)^2)*cos(theta(k));
        dydy(k)=a(k)*cos(theta(k))-(1/R)*(f(k)^2)*sin(theta(k));
    end
    l=l+L0;
    t=t+t0;
end