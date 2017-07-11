%
% vehicle.m
%
% created on: 09.10.2015
%     author: rungger
%
% see readme file for more information on the vehicle example
%
% you need to run ./vehicle binary first 
%
% so that the files: vehicle_ss.bdd 
%                    vehicle_obst.bdd
%                    vehicle_target.bdd
%                    vehicle_controller.bdd 
% are created
%

function myRobot
clear set
close all



%% simulation
x0=[1920 1812 0.14];
controller=SymbolicSet('myRobot_controller.bdd','projection',[1 2 3]);
target=SymbolicSet('myRobot_ts.bdd');

y=x0;
v=[];
while(1)
 if (target.isElement(y(end,:))) 
    break;
  end 

  u=controller.getInputs(y(end,:))
  v=[v; u(1,:)];
  [t x]=ode45(@unicycle_ode,[0 .7], y(end,:), [],u(1,:));

  y=[y; x(end,:)];
end



%% plot the vehicle domain
% colors
colors=get(groot,'DefaultAxesColorOrder');


% load the symbolic set containig the abstract state space
set=SymbolicSet('myRobot_ss.bdd','projection',[1 2]);
plotCells(set,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
hold on

% load the symbolic set containig obstacles
set=SymbolicSet('myRobot_obst.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)

% plot the real obstacles and target set5
% plot_domain

% load the symbolic set containig target set
set=SymbolicSet('myRobot_ts.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

% plot initial state  and trajectory
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)


box on
axis([350 2050 450 2470]);


end

function dxdt = unicycle_ode(t,x,u)
 dxdt = zeros(3,1);
 dxdt(1) = (u(1)+u(2))*0.5*cos(x(3))*0.7459991;
 dxdt(2) = (u(1)+u(2))*0.5*sin(x(3))*0.7459991;
 dxdt(3) = (u(2)-u(1))/105.4*0.678181;
end
