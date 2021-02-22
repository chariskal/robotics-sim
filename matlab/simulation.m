%% *********Simulation**********

%% *** Robot (kinematic) model parameters *** 
clear; 
close all;
clc;
l = [5 5 8 0 12 12];  %% in cm 

%   *** sampling period *** 
%   *** for the robot motion, kinematic simulation: 
dt = 0.001; %dt = 0.001; i.e. 1 msec)   

Tf=10.0; 	% 10 sec duration of motion 
t=0:dt:Tf;  

%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
%   initial/final end-point position --> desired task-space trajectory  

%   Case 1: 
% xd0 = 20.00; xd1 = 20.00;   yd0 = 10.00; yd1 = -10.00;    zd0=5.00; zd1=5.00;

%   Case 2:
% xd0 = 20.00; xd1 = 20.00;    yd0 = 10.00; yd1 = -10.00;    zd0=7.00; zd1=0.00;

%   Case 3:
% xd0 = 20.00; xd1 = 40.00;    yd0 = 15.00; yd1 = -15.00;    zd0=0.00; zd1=0.00;

%   Case 4:
% xd0 = 20.00; xd1 = 30.00;    yd0 = 15.00; yd1 = -15.00;    zd0=-2.00; zd1=6.00;

%   Case 35:
% xd0 = 20.00; xd1 = 25.00;    yd0 = 0.00; yd1 = 0.00;    zd0=6.00; zd1=0.00;

%You can change points A and B
%0-> start point, 1->end point

xd0 = 20.00;	
xd1 = 25.00; 
yd0 = -12.00; 
yd1 = 12.00;  
zd0=6.00;
zd1=0.00;

% Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %% 
disp(' ');   

kmax=Tf/dt + 1; 

xd=(xd1-xd0)/2*sin(linspace(-pi/2,pi/2,kmax))+(xd0+xd1)/2;
yd=(yd1-yd0)/2*sin(linspace(-pi/2,pi/2,kmax))+(yd0+yd1)/2;
zd=(zd1-zd0)/2*sin(linspace(-pi/2,pi/2,kmax))+(zd0+zd1)/2;
 
 num=max([max(abs(xd1))+l(2), max(abs(yd1))+10, max(abs(zd1))+l(1)]);

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %%  

%% ***** INVERSE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 
%   compute the reference joint-motion vectors: 
%   {qd(k,i), i=1,...,n (num of degrees of freedom), with k=1,..., kmax,} 
%   and reference joint (angular) velocities {qd_1(k,i)}

pp=sqrt((xd(:)-l(2)).^2+(zd(:)+l(1)).^2);
qd(:,1) = acos(l(3)./pp)+atan2(zd(:)+l(1),xd(:)-l(2));


K2sq=yd(:).^2;
W=(xd(:)-l(2)).^2+(zd(:)+l(1)).^2-l(3)^2-l(4)^2;
% c2q1=-cosd(2*qd(:,1));    % or use this one
c2q1=sin(qd(:,1)).^2-cos(qd(:,1)).^2;
qd(:,3) =acos((W(:)./c2q1+K2sq(:)-l(5)^2-l(6)^2)./(2*l(5)*l(6)));


A=l(5)+cos(qd(:,3))*l(6);
B=sin(qd(:,3))*l(6);
Q=B(:)+yd(:);
Q(Q==0)=4.934802024791907e-09;      % take into account dividing with zero

tau=(A(:)-sqrt(A(:).^2+B(:).^2-yd(:).^2))./Q;
qd(:,2)=2*atan(tau(:));



%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS ***** 

[xd_,yd_,zd_,xd1,yd1,zd1,xd2,yd2,zd2,xd3,yd3,zd3]=frwd_kinematics(qd,l);


%% *** SAVE and PLOT position of end effector and q's data *** %%** use functions plot(...)  
 % save;  %% --> save data to 'matlab.mat' file   

fig1 = figure;  

subplot(2,3,1); 
plot(t,xd); 
ylabel('xd (cm)'); 
xlabel('time t (sec)');  

subplot(2,3,2); 
plot(t,yd); 
ylabel('yd (cm)'); 
xlabel('time t (sec)');  

subplot(2,3,3); 
plot(t,zd); 
ylabel('zd (cm)'); 
xlabel('time t (sec)');  



subplot(2,3,4); 
plot(t,qd(:,1)); 
ylabel('qd1 (deg)'); 
xlabel('time t (sec)');  

subplot(2,3,5); 
plot(t,qd(:,2)); 
ylabel('qd2 (deg)'); 
xlabel('time t (sec)');    

subplot(2,3,6); 
plot(t,qd(:,3)); 
ylabel('qd3 (deg)'); 
xlabel('time t (sec)');  

%% *** PLOT velocities of end effector and q_dot*** %%** use functions plot(...)  

vx=diff(xd)./diff(t);           % find velocities of end effector using definition of velocity
vy=diff(yd)./diff(t);
vz=diff(zd)./diff(t);

qd1_dot=(qd(2:end,1)'-qd(1:end-1,1)')./(t(2:end)-t(1:end-1));       
qd2_dot=(qd(2:end,2)'-qd(1:end-1,2)')./(t(2:end)-t(1:end-1));
qd3_dot=(qd(2:end,3)'-qd(1:end-1,3)')./(t(2:end)-t(1:end-1));

% Plot all figures
fig2 = figure;  

subplot(2,3,1); 
plot(t(2:end),vx);                      %Vx
ylabel('vx (cm/s)'); 
xlabel('time t (sec)');  

subplot(2,3,2);     
plot(t(2:end),vy);                      %Vy
ylabel('vy (cm/s)'); 
xlabel('time t (sec)');  

subplot(2,3,3); 
plot(t(2:end),vz);                      %Vz
ylabel('vz (cm/s)'); 
xlabel('time t (sec)');  



subplot(2,3,4); 
plot(t(2:end),qd1_dot);                 %derivative of qd1
ylabel('qd1 dot (rad/s)'); 
xlabel('time t (sec)');  

subplot(2,3,5); 
plot(t(2:end),qd2_dot);                 %derivative of qd2
ylabel('qd2 dot (rad/s)'); 
xlabel('time t (sec)');    

subplot(2,3,6); 
plot(t(2:end),qd3_dot);                 %derivative of qd3
ylabel('qd3 dot (rad/s)'); 
xlabel('time t (sec)');  

%% *** stick diagram --> animate robot motion ...  
%% within a for (or while) loop, use periodic plot(...) functions to draw the geometry (current pos)  
%% of the robot, and thus animate its motion ...  

% Plot in 3D space

fig3 = figure; 

axis([-num num -num num -num num]) %%set xyz plot axes (caution: square axes) 
axis on 
hold on 
grid on

xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel('z (cm)')

plot3(xd,yd,zd,'r','LineWidth',2);      %desired trejectory
dtk=1000; %% plot robot position every dtk samples, to animate its motion 

plot3(0,0,0,'*');                           %O(0,0,0)

plot3([0 0],[0 0],[0,-l(1)],'k');           %Plot l0 and l1 that are fixed
plot3([0,l(2)],[0 0],[-l(1),-l(1)],'k');

plot3(l(2),0,-l(1),'b o');                  %position of q1


for tk=1:dtk:kmax    %%% 	
   pause(0.2);	%% pause motion to view successive robot configurations  
   
   plot3([l(2),xd_(tk)],[0,yd_(tk)],[-l(1),zd_(tk)],'b');	
   
   plot3([xd_(tk),xd1(tk)],[yd_(tk),yd1(tk)],[zd_(tk),zd1(tk)],'b');		%link1			
   plot3(xd1(tk),yd1(tk),zd1(tk),'b o');   
   
   plot3([xd1(tk),xd2(tk)],[yd1(tk),yd2(tk)],[zd1(tk),zd2(tk)],'g');        %link2		
   plot3(xd2(tk),yd2(tk),zd2(tk),'b o');   
   
   plot3([xd2(tk),xd3(tk)],[yd2(tk),yd3(tk)],[zd2(tk),zd3(tk)],'g');        %link3
   plot3(xd3(tk),yd3(tk),zd3(tk),'c *');                                    %actual position of end effector
   
   plot3(xd(tk),yd(tk),zd(tk),'k +');                                       %desired position of end effector
end       
