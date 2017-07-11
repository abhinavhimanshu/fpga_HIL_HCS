% see readme file for more information on the vehicle example
%
% you need to run ./myRobot binary first 
%
% so that the files: myRobot_ss.bdd 
%                    myRobot_obst.bdd
%                    myRobot_target.bdd
%                    myRobot_controller.bdd 
% are created
% Make sure FPGA board is connected through UART and you are logged out of
% FPGA board
% If it gives unexpected errors then try connecting than
% run "instrreset" on matlab
% try login fpga thorugh minicom
% logout
% close minicom
% and run this code again
% 

function checkit
     %%
          clear all  % clear all previous varaibles
          instrreset % reset all the ports 
          clc        % clear the command window
     %%  
          comport = terminal_open(); % Open terminal
          flushinput(comport);       % Flush input
          flushoutput(comport);      % Flush Output
     %%    
          username='root';           % FPGA username to login
          passwd='root';             % FPGA passwd to login
          login(comport,username,passwd);% login in the FPGA via Matlab thorugh UART
          fprintf("-------Log In succesfull \n"); 
          flushinput(comport); % flush comport 
     %%   
          filename = './uarttest.elf'; %name of file to be run on FPGA  
          fprintf(comport,['./uarttest.elf' '\n']); % Run the uarttest file on FPGA
          java.lang.Thread.sleep(length(['./uarttest.elf' '\n']));
          pause(1);
          msg=terminal_read(comport); %read at port
          msg=terminal_read(comport); %read at port
          fprintf("%s\n",msg);        
     %%
           % target set
          target=SymbolicSet('myRobot_ts.bdd');%read target details from file
            
             
     %%  
          %simulation data for state space
          lbs=[450,600,-3.4000]; % As described in controller.bdd file after quantization 
          ubs=[1980,1890,pi+0.4];%Upper bound
          
          x0=[1920 1812 0.14]; % intial state
          etas=[30,30,.2]; %eta values
          smask=[6 6 6 ];  % number of BDD variable in each dim order [1st 2nd 3rd]
          y=x0;    % trajectory
          v=[];    % control inputs
          ci=[];   % saves bdd index of all control inputs     
     %%
          %simulation data for input space 
          %bdd varaiable to control input
            lbc=[-240,-240];% As described in controller.bdd file after quantization 
            etac=[120,120];%eta value in each dimension 
            cmask=[3 3];% number of BDD variable in each dim order [1st 2nd 3rd]
  
     %%   %% To check if program started correctly on FPGA
          terminal_write(comport,'ping'); % Ping FPGA
          msg=terminal_read(comport);     % read comport
          fprintf("%s\n",msg);            %if returns PONG that program started correctly
          indexX=[];
      %%
          while(1)
              
             if (target.isElement(y(end,:))) 
                 break;
             end 
             %%%%% convert physical state to bdd
                  indexX=[indexX;round((y(end,1)-lbs(1))/etas(1)),round((y(end,2)-lbs(2))/etas(2)),round((y(end,3)-lbs(3))/etas(3))];%% Get index value in each dimesnion
                  bddx=0;% store the final BDD index
                  bddx = indexX(end,3); %higest dimension bits(MSB)
                  bddx= bitshift(bddx, smask(3)); %Shfit the bits to put lower bits
                  bddx= bitor(bddx,indexX(end,2));%Or the 2nd index
                  bddx= bitshift(bddx, smask(2)); %Shift to put 1st dim index
                  bddx= bitor(bddx,indexX(end,1));%Put first dimesnion 
              %%
                  m=int2str(bddx) ; %integer to string
                  terminal_write(comport,m); % send to FPGA
                  msg=terminal_read(comport);% read from FPGA 
            %%fprintf("%s\n",msg);
                  msg=terminal_read(comport);% read from FPGA
            %%fprintf("%s\n",msg);
                  
                  bddc=str2num(msg); %convert string to integer 
                  ci=[ci;bddc];
              %% Convert control input index to real control inputs
                  c=[0 0]; % start with zero inputs
                  c(1)=bitand(bddc,2^cmask(1)-1);%get lower 3 bits of index 
                  c(1)=lbc(1)+etac(1)*c(1);%find real value from this index
                  bddc=bitshift(bddc,-cmask(1));%shift to get higher 3 bits
                  c(2)=bitand(bddc,2^cmask(2)-1);% get higher 3 bits 
                  c(2)=lbc(2)+etac(2)*c(2);%find the real value to this control inputs
                  u=c;
                  v=[v; u(1,:)];
                  [t x]=ode45(@unicycle_ode,[0 .7], y(end,:), [],u(1,:)); %compute the next state
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

% load the symbolic set containig target set
set=SymbolicSet('myRobot_ts.bdd','projection',[1 2]);
plotCells(set,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

% plot initial state  and trajectory
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)
box on
axis([350 2050 450 2470]);
%%
  logout(comport); %logout from the FPGA board 
  termina_close(comport); %close the terminal
          
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% To login the FPGA Board
function login(port,usrname,passwd)
    terminal_write(port,usrname);
    flushinput(port);
    fprintf(port,passwd);
    flushinput(port);     
    flushoutput(port);       
end

%% To logout the FPGA Board
function logout(port)
    flushoutput(port);
    fprintf(port,'exit'); 
    java.lang.Thread.sleep(length(['exit' '\n']));
    flushinput(port);       
end

%% To read the terminal
function out = terminal_read(port);
   out=[];
   out=fgets(port);
   if(length(out)>=2)
    out(end-1:end)=[];
   else
       out='blank';
   end
end
              
 %% To write the terminal         
function terminal_write(port, str)
    flushinput(port);
    flushoutput(port);
    fprintf(port, [str '\n']);
    java.lang.Thread.sleep(length([str '\n']));
    echo = fgets(port);
   
    echo(end-1:end)=[]; % removing the \n and \r
    if(strcmp(str, echo) == 0)
        termina_close(port);
        error('Failed to write to terminal');
    end
    
end   

%% closes the terminal  
function termina_close(port)
    fclose(port)
    delete(port)
    clear port
end
          
         
%% Opens port at given attributes           
 function port = terminal_open()
    COM_PORT = '/dev/ttyACM0';
    BAUD_RATE = 115200;
    try        
        port = serial(COM_PORT, 'Baudrate', BAUD_RATE, 'Parity', 'none', 'DataBits', 8, 'StopBits', 1, 'Terminator', 'LF', 'Timeout',10);
        fopen(port);
    catch
        termina_close(port);
        error('Failed to00000000000000000 open the terminal');
    end
    flushinput(port);
 end     
          
          
 %% Ode dynamics for the Robot 
function dxdt = unicycle_ode(t,x,u)
 dxdt = zeros(3,1);
 dxdt(1) = (u(1)+u(2))*0.5*cos(x(3))*0.7459991;
 dxdt(2) = (u(1)+u(2))*0.5*sin(x(3))*0.7459991;
 dxdt(3) = (u(2)-u(1))/105.4*0.678181;
end

          
          