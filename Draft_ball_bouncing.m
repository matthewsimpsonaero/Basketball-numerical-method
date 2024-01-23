clear;clc; close all


%% declare state space variables in an array

m = 10;      %mass
g = 9.81;    %gravity
mu = .2;     %friction coeficient
z(1) = 0;    %x position
z(2) = 8;    %x velocity
z(3) = 7;    %y position
z(4) = 0;    %y velcocity
z(5) = 0;    %inital angle
z(6) = -40;  %angular velocity
r = 0.7;     %radius
k = 10000;     %spring constant
c = 60;      %damping constant
dT = 0.01;   %timestep
i = 900;     %i is the number of steps


%% normal force calculation

for t = 1:i
    if z(3) < r % if it bounces 
        N(t) = -k*(r-z(3))-c*z(4); %apply a normal force ( -kx -zxdot)
    else
        N(t) = 0; %if not then dont apply normal force
    end
    T = (t-1)*dT; %actual time
    z = secondInt(z,dT,r,k,c,m,g,mu); %numerical integration
    time(t) = T; %time
    xGraph(t) = z(1); %x position as it changes
    yGraph(t) = z(3); %y position as it changes
    
end
%% animation of the plot
for j = 1:i %new for loop variable
    figure(1) %make new firgure
    plot(xGraph(j),yGraph(j),'ko','MarkerSize',16) %plot a point
    xlim([-1 20]) %x limits
    ylim([0,20])% y limits
    drawnow %draw function to animate
    hold off %dont replot
end
%% plotting the other graphs
figure(2) %new figure
plot(xGraph,yGraph,'b') %plot entire curve of ball
title('y vs. x') %plot styling
figure(3)
plot(time,N,'b') %plot normal force versus time

function d = namestate(z,r,k,c,m,g,mu)
%name of state variables
d(1) = z(2); %first condition is just x velocity
%normal force conditions
if z(3)  < r  %if ball is in the floor
    N = k*(r-z(3))-c*z(4); %apply normal force
elseif z(3) >= r %if not
    N = 0; %no normal force
end
%friction force
if (z(2)-r*z(6)) < 0 %direction of x velocity compared to angular velocity
    F = mu*N; %direction of normal force
elseif (z(2)-r*z(6)) > 0 %direction of x velocity compared to angular velocity
    F = -mu*N;
else 
   F = 0;
end
 d(2) = F/m; %zdot
 d(3) = z(4); %derivative of values
 d(4) = (N/m)-g;
 d(5) = z(6);
 d(6) = (-2*F)/(m*r);
    

end
    
function x = secondInt(x,dT,r,k,c,m,g,mu) % runga kutta 
f1 = namestate(x,r,k,c,m,g,mu); %evaulate at first order
deltax1 = dT*f1; %find delta x
f2 = namestate(x + deltax1,r,k,c,m,g,mu); %second order 
x  = x + 0.5*dT*(f1 + f2); %second order runga kutta
end
