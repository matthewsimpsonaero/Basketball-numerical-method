% Matthew Simpson
% MAE 361 - Vibrations
% Dr. Silverberg
clear;clc; close all


%% declaring all variables

m = 15;          %mass
g = 9.81;        %gravity
mu = .3;         %friction coeficient
z(1) = 0;        %x position
z(2) = 13;       %x velocity
z(3) = 10;       %y position
z(4) = 0;        %y velcocity
z(5) = 0;        %inital angle
z(6) = -22;      %angular velocity
r = 1.5;         %radius
k = 15000;       %spring constant of ground
c = 190;         %damping constant for ground
dT = 0.01;       %timestep
maxtime = 1400;  %maxtime is the number of steps
kwall = 100;     %spring constant of walls
cwall = 40;      %damping constant of walls
vertcolision = 0;%checker for vertical collision
barLength = 10;  %length of the launcher bar
angleBar = 115;  %angle of the launcher bar
q(1,1) = 2;      %x position of bar
q(1,2) = 9.1;    %vx of bar
q(1,3) = -10;    %y position of bar
q(1,4) = 7.4;    %vy of bar
filename = 'hw4.gif'; %filename to save the movie to
%% Bar Calculations

for step = 2:maxtime
    if step<845 %before the motion of the ball
        q(step,2) =  q(1,2);
        q(step,4) =  q(1,4);
        q(step,1) =  q(1,1);
        q(step,3) =  q(1,3);
    elseif step > 970 %afer the motion of the ball
        q(step,2) =  q(969,2);
        q(step,4) =  q(969,4);
        q(step,1) =  q(969,1);
        q(step,3) =  q(969,3);
    else
        
        q(step,2) = q(step-1,2); %finding the position of the bar at the time
        q(step,4) = q(step-1,4);
        q(step,1) = q(step-1,1)+q(step,2)*dT;
        q(step,3) = q(step-1,3)+q(step,4)*dT;
    end
    barline(step,1) = q(step,1) + (barLength/2)*sind(angleBar); %finding the position of the endpoints of the bar
    barline(step,2) = q(step,1) - (barLength/2)*sind(angleBar);
    barline(step,3) = q(step,3) + (barLength/2)*cosd(angleBar);
    barline(step,4) = q(step,3) - (barLength/2)*cosd(angleBar);
end
%% force calculations to plot at the end
for timestep = 1:maxtime

    T = (timestep)*dT; %actual time
    if timestep >= 1 %determing if the bar and the ball are colliding
        x1 = barline(timestep,1);
        x2 = barline(timestep,2);
        y1 = barline(timestep,3);
        y2 = barline(timestep,4);
        x3 = z(1)-r*sin(180-angleBar);
        y3 = z(3)-r*cos(180-angleBar);
        alpha = (x3-x1)/(x2-x1);
        beta = (y3-y1)/(y2-y1);

    end
        qBegin = q(1,:); %inital velocitie of the bar (since it doesnt change with time)
        BarLineAtTime = barline(timestep,:); %position of the bar
        [z,vertcolision] = RungeKutta(z,dT,r,k,c,m,g,mu,kwall,cwall,vertcolision,BarLineAtTime,alpha,beta,angleBar,q);%numerical integration
    
        
        
%normal force in y direction from state space equation
if z(3)  < r && (z(1) > 12 || z(1) < 5) && vertcolision == 0 %if ball is in the floor
    N(timestep) = k*(r-z(3))-c*z(4); %apply normal force
else %if not
    N(timestep) = 0; %no normal force
end

if z(1) > 5 && z(1) < 12 && z(3)<-5+r % vertical normal force if the ball is in the hole
    N(timestep) = k*(-5-(z(3)-r))-c*z(4); 
end

% normal force in x direction conditions
if z(1) > 30-(r-0.4)   %hitting vertical wall at x = 30
    Nx(timestep) = -kwall*((r)-z(1))+cwall*z(2);
else
    Nx(timestep) = 0;
end

%normal force in x direction from state space equation
if ((z(1) < r+4.4) && z(3) < 0) %if the ball is hitting the left wall in the hole
    Nx(timestep) = -k*(5-(z(1)-r))+cwall*z(2);
    vertcolision= 1; % turns off normal force in y direction when it deforms into vertical walls
else
    vertcolision = 0;
end
if ((z(1) > 12-(r-.4)) && z(3) < 0) %if the ball is hitting the right wall
    Nx(timestep) = -1000*(5-(z(1)-r))+cwall*z(2);
    vertcolision= 1;
else
    vertcolision = 0;
end

%normal forces for ball hitting the launcher using the coeficient of
%proportionalty alpha and beta
if alpha > 0 && alpha < 1 && beta > 0 && beta < 1
    Nx(timestep) = 7*kwall*(z(1)-r*sind(angleBar)-(BarLineAtTime(2)+(alpha*(BarLineAtTime(1)-BarLineAtTime(2)))))-c*(q(2)-z(2));
    N(timestep) = 7*kwall*(z(3)-r*cosd(angleBar)-(BarLineAtTime(4)+(beta*(BarLineAtTime(3)-BarLineAtTime(4)))))-c*(q(4)-z(4));
end


    angle = z(5); %angle that the ball is currently at (0-360 degrees)
    
    
    time(timestep) = T; %time
    xposition(timestep) = z(1); %x position as it changes
    yposision(timestep) = z(3); %y position as it changes
    omega(timestep) = z(6);
    
    angle = z(5); %angle that the ball is currently at (0-360 degrees)
    rotationline(timestep,1) = z(1)+(1.45*r/2)*sin(angle); %calculating the endpoints of the rotating bar inside of the ball
    rotationline(timestep,2) = z(1)-(1.45*r/2)*sin(angle);
    rotationline(timestep,3) = z(3)+(1.45*r/2)*cos(angle);
    rotationline(timestep,4) = z(3)-(1.45*r/2)*cos(angle);
end
%% animation of the plot
for time = 1:5:maxtime %new for loop variable
    h = figure(1); %make new firgure
    plot(xposition(time),yposision(time),'ro','MarkerSize',20) %plot a point
    hold on
    plot([0;5],[0;0],'k');
    text(5,-8,' Ball Launcher')
    text(24,17,'Basketball Hoop')
    text(7,-1,'Hole In Floor')
    plot([12;30],[0;0],'k'); %plotting the walls and floor
    plot([30,30],[0,20],'k');
    plot([12;12;5;5],[0;-5;-5;0],'k')
    plot([rotationline(time,1),rotationline(time,2)],[rotationline(time,3),rotationline(time,4)],'r'); %plotting bar inside of the ball
    plot([barline(time,1),barline(time,2)],[barline(time,3),barline(time,4)],'LineWidth',6); %plotting the launcher bar
    plot([30,25],[15,15],'Color',[0.8500, 0.3250, 0.0980],'LineWidth',4) %plotting basketball hoop
    plot([0,25],[8.5,8.5],'k') %plotting the beginning of where the crowd is
    %person 1
    rectangle('Position',[1 14 2 4],'Curvature',[1,1]);
    rectangle('Position',[1.5 16.5 .2 .2],'Curvature',[1,1]);
    rectangle('Position',[2.5 16.5 .2 .2],'Curvature',[1,1]);
    plot([2,2],[14,8.5],'k')
    %person 2
    rectangle('Position',[6 14 2 4],'Curvature',[1,1]);
    rectangle('Position',[6.5 16.5 .2 .2],'Curvature',[1,1]);
    rectangle('Position',[7.5 16.5 .2 .2],'Curvature',[1,1]);
    plot([7,7],[14,8.5],'k')
    %person 3
    rectangle('Position',[11 14 2 4],'Curvature',[1,1]);
    rectangle('Position',[11.5 16.5 .2 .2],'Curvature',[1,1]);
    rectangle('Position',[12.5 16.5 .2 .2],'Curvature',[1,1]);
    plot([12,12],[14,8.5],'k')
    %person 4
    rectangle('Position',[16 14 2 4],'Curvature',[1,1]);
    rectangle('Position',[16.5 16.5 .2 .2],'Curvature',[1,1]);
    rectangle('Position',[17.5 16.5 .2 .2],'Curvature',[1,1]);
    plot([17,17],[14,8.5],'k')
    %person 5
    rectangle('Position',[21 14 2 4],'Curvature',[1,1]);
    rectangle('Position',[21.5 16.5 .2 .2],'Curvature',[1,1]);
    rectangle('Position',[22.5 16.5 .2 .2],'Curvature',[1,1]);
    plot([22,22],[14,8.5],'k')
    
    if time>1230 %if the ball goes in the hoop
        %person 1
        plot([2,0],[12,15],'k')
        plot([2,4],[12,15],'k')
        rectangle('Position',[1.5 15 .4 .4],'Curvature',[1,1]);
        %person 2
        plot([7,5],[12,15],'k')
        plot([7,9],[12,15],'k')
        rectangle('Position',[6.5 15 .4 .4],'Curvature',[1,1]);
        %person 3
        plot([12,10],[12,15],'k')
        plot([12,14],[12,15],'k')
        rectangle('Position',[11.5 15 .4 .4],'Curvature',[1,1]);
        %person 4
        plot([17,15],[12,15],'k')
        plot([17,19],[12,15],'k')
        rectangle('Position',[16.5 15 .4 .4],'Curvature',[1,1]);
        %person 5
        plot([22,20],[12,15],'k')
        plot([22,24],[12,15],'k')
        rectangle('Position',[21.5 15 .4 .4],'Curvature',[1,1]);
        
        
        if time>1230 && time<1430 %making the confetti cannon
            randomMatrix = randi([-30 30],2,410);
            plot(randomMatrix(1,1:60),randomMatrix(2,1:60),'r*','Markersize',1)
            plot(randomMatrix(1,61:120),randomMatrix(2,61:120),'g*','Markersize',1)
            plot(randomMatrix(1,121:180),randomMatrix(2,121:180),'b*','Markersize',1)
            plot(randomMatrix(1,181:240),randomMatrix(2,181:240),'c*','Markersize',1)
            plot(randomMatrix(1,241:300),randomMatrix(2,241:300),'m*','Markersize',1)
            plot(randomMatrix(1,301:360),randomMatrix(2,301:360),'y*','Markersize',1)
            plot(randomMatrix(1,361:410),randomMatrix(2,361:410),'k*','Markersize',1)
        end
    else
        %person 1
        plot([2,0],[12,9],'k')
        plot([2,4],[12,9],'k')
        plot([1.5,2.5],[15,15],'r')
        %person 2
        plot([7,5],[12,9],'k')
        plot([7,9],[12,9],'k')
        plot([6.5,7.5],[15,15],'r')
        %person 3
        plot([12,10],[12,9],'k')
        plot([12,14],[12,9],'k')
        plot([11.5,12.5],[15,15],'r')
        %person 4
        plot([17,15],[12,9],'k')
        plot([17,19],[12,9],'k')
        plot([16.5,17.5],[15,15],'r')
        %person 5
        plot([22,20],[12,9],'k')
        plot([22,24],[12,9],'k')
        plot([21.5,22.5],[15,15],'r')
    end
    
    xlim([-1 32]) %x limits
    ylim([-15,23])% y limits
    drawnow %draw function to animate
    title('Basketball Particle Motion')
    xlabel('X (m)')
    ylabel('Y (m)')
    legend('',sprintf('Time: %.2f sec',time/100),sprintf('X Position %.2f m',xposition(time)),sprintf('Y Position %.2f m',yposision(time)),sprintf('Angular Velocity %.2f rad/sec',omega(time)),'Location','southeast')
    hold off
    frame = getframe(h); %getting the frames to save into the movie

      im = frame2im(frame); 

      [imind,cm] = rgb2ind(im,256); 

      

      if time == 1  %create the gif file

          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 

      else %append to th gif file

          imwrite(imind,cm,filename,'gif','WriteMode','append'); 

      end 
end


%% plotting the other graphs

figure(2) %new figure
plot(xposition,yposision,'k') %plot entire curve of ball
title(' Planar Position of Ball Throughout Event')
xlabel('X-Position (m)')
ylabel('Y-Position (m)')
saveas(gcf,'Planar_Position.png');

figure(3)
plot(time,N,'b') %plot normal force versus time
title(' Vertical Normal Force Versus Time')
xlabel('Time (sec)')
ylabel('Normal Force (N)')
text(1,6000,'Floor Region')
text(6.5,7000,'Hole in Floor Region')
text(11.5,6000,'Launcher')
text(11.5,5700,'Force')
saveas(gcf,'Vertical_Normal.png');

figure(4)
plot(time,Nx,'b') %plot horizontal normal force versus time
title(' Horizontal Normal Force Versus Time')
xlabel('Time (sec)')
ylabel('Normal Force (N)')
text(3,4000,'Wall Collision')
text(7,5500,'Hole in Floor Region')
text(11.5,4000,'Laucher Force')
saveas(gcf,'Horizontal_Normal.png');

figure(5)
yyaxis left
p1 = plot(time,xposition,'g-');
p2 = plot(time,yposision,'k-') ;%plot vertical normal force versus time
hold on 
xlabel('time (sec)')
ylabel('position magnitude (m)')
yyaxis right
p3 = plot(time,omega,'r-');
title('Planar Motion Versus Time')
xlabel('time (sec)')
ylabel('omega (rad/sec)')
%legend( [p1;p2;p3] , {'X-Position','Y-Position','Omega'})% ,'Location','NorthWest');
saveas(gcf,'position_and_omega.png');


% function used 


function [state_var_dot,vertcolision] = state_space(z,r,k,c,m,g,mu,kwall,cwall,vertcolision,BarLineAtTime,alpha,beta,anglebar,q)

%normal force in y direction conditions
if z(3)  < r && (z(1) > 12 || z(1) < 5) && vertcolision == 0 %if ball is in the floor
    N = k*(r-z(3))-c*z(4); %apply normal force
else %if not
    N = 0; %no normal force
end

if z(1) > 5 && z(1) < 12 && z(3)<-5+r
    N = k*(-5-(z(3)-r))-c*z(4); 
end

% normal force in x direction conditions
if z(1) > 30-(r-0.4)   %hitting vertical wall at x = 30
    Nx = -kwall*((r)-z(1))+cwall*z(2);
else
    Nx = 0;
end


if ((z(1) < r+4.4) && z(3) < 0)
    Nx = -k*(5-(z(1)-r))+cwall*z(2);
    vertcolision= 1; % turns off normal force in y direction when it deforms into vertical walls
else
    vertcolision = 0;
end
if ((z(1) > 12-(r-.4)) && z(3) < 0)
    Nx = -1000*(5-(z(1)-r))+cwall*z(2);
    vertcolision= 1;
else
    vertcolision = 0;
end

%normal forces for ball hitting the launcher
if alpha > 0 && alpha < 1 && beta > 0 && beta < 1
    Nx = 7*kwall*(z(1)-r*sind(anglebar)-(BarLineAtTime(2)+(alpha*(BarLineAtTime(1)-BarLineAtTime(2)))))-c*(q(2)-z(2));
    N = 7*kwall*(z(3)-r*cosd(anglebar)-(BarLineAtTime(4)+(beta*(BarLineAtTime(3)-BarLineAtTime(4)))))-c*(q(4)-z(4));
end



%friction force with directions
if (z(2)-r*z(6)) < 0 %direction of x velocity compared to angular velocity
    F = mu*N; %direction of normal force
elseif (z(2)-r*z(6)) > 0 %direction of x velocity compared to angular velocity
    F = -mu*N;
else
    F = 0;
end

%equations from lecture that were derived in class
state_var_dot(1) = z(2);
state_var_dot(2) = (F-Nx)/m; %zdot
state_var_dot(3) = z(4); %derivative of values
state_var_dot(4) = (N/m)-g;
state_var_dot(5) = z(6);
state_var_dot(6) = (-2*F)/(m*r);


end

% runge kutta numerical integration
function [state_var,vertcolision] = RungeKutta(state_var,dT,r,k,c,m,g,mu,kwall,cwall,vertcolision,BarLineAtTime,alpha,beta,anglebar,q) % runga kutta
[state_var_dot,vertcolision] = state_space(state_var,r,k,c,m,g,mu,kwall,cwall,vertcolision,BarLineAtTime,alpha,beta,anglebar,q); %evaulate at first order
state_var_change = dT*state_var_dot; %find delta x
[state_var_dot2,vertcolision] = state_space(state_var + state_var_change,r,k,c,m,g,mu,kwall,cwall,vertcolision,BarLineAtTime,alpha,beta,anglebar,q); %second order
state_var  = state_var + 0.5*dT*(state_var_dot + state_var_dot2); %second order runga kutta
end

