%% The path tracking simulation is implemented by Cheng Liu in this source file
function twoWheeledTracking_test()
clear;clc;close all;

%% ENVIRONMENT AND SIMULATION SETUP
xBound=[-10 10];                                    % specify environment range (x)
yBound=[-10 10];                                    % specify environment range (y)
sampleDist=0.5;                                     % distance limit for specifying new trajectory waypoint
dT=0.05;                                            % delta T
steeringDirLimit=30*pi/180;                         % steering angle limit for specifying new trajectory waypoint
Time=0:0.1:20;                                      % sampling time
nodeNum=numel(Time);                                % number of specified waypoints

%% ERRORS AND REAL TRAJECTORY
Trajectory.positionError=zeros(2,nodeNum);          % position error
Trajectory.headingError=zeros(1,nodeNum);           % heading error
Trajectory.Path=zeros(2,nodeNum);                   % real vehicle's path
Trajectory.Orientation=zeros(1,nodeNum);            % real vehicle's orientation
Trajectory.LinearVelocity=zeros(1,nodeNum);         % linear velocity of the vehicle along the path
Trajectory.AngularVelocity=zeros(1,nodeNum);        % angular velocity of the vehicle along the path
Trajectory.WheelAngularVelocity=zeros(2,nodeNum);   % angular velocity of two wheels along the path (row 1: left; row 2: right)
Trajectory.TimeStamp=Time;                          % time stamps

%% CONTROLLER
% pid controller
PID.position.errors.previousError=0;                % previous position error                
PID.position.errors.errorSum=0;                     % historic cumulative position error
PID.position.K=[10;0.005;-0.1];                     % gains for position feedback
PID.orientation.errors.previousError=0;             % previous heading error
PID.orientation.errors.errorSum=0;                  % historic cumulative heading error
PID.orientation.K=[-20;-0.1;-0.01];                 % gains for orientation feedback
PID.linearVelocity.errors.previousError=0;          % previous linear velocity error
PID.linearVelocity.errors.errorSum=0;               % historic cumulative velocity error
PID.linearVelocity.K=[0;0;0];                       % gains for velocity feedback
PID.angularVelocity.errors.previousError=0;         % previous angular velocity error
PID.angularVelocity.errors.errorSum=0;              % historic cumulative velocity error
PID.angularVelocity.K=[0;0;0];                      % gains for angular velocity feedback

%% VEHICLE
vehicle.Mass=1;                                             % vehicle mass (kg)
vehicle.PendulumMass=0.1;                                   % pendulum mass (kg)
vehicle.PendulumLength=0.5;                                 % pendulum length (m)
vehicle.PendulumAngle=0.0;                                  % pendulum orientation (radian)
vehicle.Radius=1;                                           % effective radius of the vehicle (m)
vehicle.Position=[(xBound(2)-xBound(1))*rand+xBound(1);
    (yBound(2)-yBound(1))*rand+yBound(1)];                  % position of the center of mass of the vehicle (m)
vehicle.Theta=(360*rand)*pi/180;                            % orientation of the vehicle (radian)
vehicle.Heading=[cos(vehicle.Theta);sin(vehicle.Theta)];    % direction of the vehicle (unit vector)
vehicle.currentLinearVelocity=0;                            % current linear velocity of the vehicle (body frame)
vehicle.currentAngularVelocity=0;                           % current angular velocity of the vehicle (body frame)
vehicle.wheel.radius=0.75*vehicle.Radius;                   % wheel radius (m)
vehicle.wheel.leftRotSpd=0;                                 % rotational spd of the left wheel (rad/s)
vehicle.wheel.rightRotSpd=0;                                % rotational spd of the right wheel (rad/s)

%% SIMULATION
set(gcf, 'Position', get(0, 'Screensize'));                 % maximize the next figure to show
specifiedTraj=trajectoryGenerate(xBound,yBound,sampleDist,steeringDirLimit,nodeNum); % generate a random path
%specifiedTraj=[Time;10*sin(Time)];                         % generate a sinusoidal path
%specifiedTraj=[Time;2*Time];                               % generate a straight path

for iTraj=1:size(specifiedTraj,2)                           % traverse each trajectory waypoint
    
    % Update informtion of real trajectory
    Trajectory.positionError(:,iTraj)=specifiedTraj(:,iTraj)-vehicle.Position;  % append current position error
    Trajectory.Path(:,iTraj)=vehicle.Position;                                  % append current vehicle's position
    Trajectory.LinearVelocity(iTraj)=vehicle.currentLinearVelocity;             % append current linear velocity of the vehicle
    Trajectory.Orientation(iTraj)=vehicle.Theta;                                % append current vehicle's orientation
    Trajectory.AngularVelocity(iTraj)=vehicle.currentAngularVelocity;           % append current angular velocity of the vehicle
    Trajectory.WheelAngularVelocity(:,iTraj)=[vehicle.wheel.leftRotSpd;vehicle.wheel.rightRotSpd];% append current angular velocities of two wheels
    
    % Animate the simulation
    stateMonitoringAnimate(vehicle,specifiedTraj,Trajectory,iTraj);             % animate the simulation with the monitor to state variables
    %animate(vehicle,specifiedTraj,iTraj);                                      % animate the tracking process
    
    % Update the state variables of the vehicle
    [vehicle.currentLinearVelocity,vehicle.currentAngularVelocity,PID.position.errors,PID.orientation.errors]=PIDControl(PID,specifiedTraj(:,iTraj),vehicle);%update state variables by a PID controller
    [vehicle.Position,vehicle.Theta]=TwoWheeledVehicleDynamics(vehicle,dT);     % update vehicle position and heading through equations on motion
    [vehicle.Heading,vehicle.wheel.leftRotSpd,vehicle.wheel.rightRotSpd]=vehicleParamUpdate(vehicle);   % update other state variables
    
end

resultPlot(Trajectory,specifiedTraj);   % plot the history of all state variables.

end

% This function plots the recored trajectory and the specified trajectory
function resultPlot(realTraj,specifiedTraj)

    % plot position error
    figure;
    subplot(311);
    plot(realTraj.TimeStamp,sqrt(sum(realTraj.positionError.^2)),'LineWidth',2);% real transient position error
    ylabel('Magnitude (m)');
    title('Position Error');
    grid minor;
    subplot(312);
    plot(realTraj.TimeStamp,realTraj.positionError(1,:),'LineWidth',2);% real transient position error in x direction
    ylabel('Error_x (m)');
    grid minor;
    subplot(313);
    plot(realTraj.TimeStamp,realTraj.positionError(2,:),'LineWidth',2);% real transient position error in y direction
    ylabel('Error_y (m)');
    xlabel('Time Stamps (s)');
    grid minor;
    
    % plot real path and orientation
    figure;    
    subplot(311);
    fig=plot(realTraj.TimeStamp,realTraj.Path(1,:),realTraj.TimeStamp,specifiedTraj(1,:));% real transient path in x direction
    set(fig,{'LineWidth'},{2;2});
    ylabel('X (m)');
    legend('Real','Specified');
    title('Real Trajectory vs. Specified Trajectory');
    grid minor;
    subplot(312);
    fig=plot(realTraj.TimeStamp,realTraj.Path(2,:),realTraj.TimeStamp,specifiedTraj(2,:),'LineWidth',2);% real transient path in y direction
    set(fig,{'LineWidth'},{2;2});
    ylabel('Y (m)');
    legend('Real','Specified');
    grid minor;
    subplot(313);
    plot(realTraj.TimeStamp,realTraj.Orientation*180/pi,'LineWidth',2);% real transient orientation
    ylabel('\theta (\circ)');
    title('Orientation');
    xlabel('Time Stamps (s)');   
    grid minor;
    
    % plot transient linear velocity and angular velocity
    figure;
    subplot(311)
    plot(realTraj.TimeStamp,realTraj.LinearVelocity,'LineWidth',2);% real transient linear velocity
    ylabel('V (m/s)');
    legend('Linear Velocity in the Body Frame');
    grid minor;
    subplot(312)
    plot(realTraj.TimeStamp,realTraj.AngularVelocity,'LineWidth',2);% real transient angular velocity
    ylabel('\Omega (rad/s)');
    legend('Angular Velocity in the Body Frame');
    grid minor;
    subplot(313)
    fig=plot(realTraj.TimeStamp,realTraj.WheelAngularVelocity(1,:),realTraj.TimeStamp,realTraj.WheelAngularVelocity(2,:));% real transient wheel speed
    set(fig,{'LineWidth'},{2;2});
    legend('Left','Right');
    ylabel('\omega (rad/s)');
    xlabel('Time Stamps (s)');
    grid minor;
    
end

% This function animates the tracking simulation
function animate(vehicle,traj,idx)

    vehicleFig=drawVehicle(vehicle);                        % plot vehicle
    fig=plot(traj(1,idx),traj(2,idx),'g*','LineWidth',5);   % plot path

    pause(0.05);

    if(idx==1)
        drawTrajectory(traj);
        pause;
    end

    if(idx<size(traj,2))
        set(fig,'Visible','off');
        set(vehicleFig,'Visible','off');
    end

end

% This function draws the specified trajectory with boundary limit to the
% environment.
function drawTrajectory(traj,xBound,yBound)

    plot(traj(1,:),traj(2,:),'r-','LineWidth',2);
    if exist('xBound','var')
       xlim(xBound*1.5); 
    end
    if exist('yBound','var')
       ylim(yBound*1.5);
    end

    axis equal;
    hold on;

end

% This function generates random trajectory by sampling waypoints subject
% to sampling distance limit and steering angle limit.
function traj=trajectoryGenerate(xBound,yBound,sampleDist,steeringLimit,nodeNum)

    traj=zeros(2,nodeNum);
    traj(1,1)=(xBound(2)-xBound(1))*rand+xBound(1);
    traj(2,1)=(yBound(2)-yBound(1))*rand+yBound(1);
    
    initialSteeringLimit=steeringLimit;    
    tempAngle=0;
    
    for idx=2:nodeNum
        
        while(true)
            
            traj(1,idx)=(2*rand-1)*sampleDist+traj(1,idx-1);
            traj(2,idx)=(2*rand-1)*sampleDist+traj(2,idx-1);
             
            dr=traj(:,idx)-traj(:,idx-1);
            
            radiusFlag=sqrt(sum(dr.^2))<sampleDist;
            angle=atan2(dr(2),dr(1))-tempAngle;
            angleFlag=angle>=-steeringLimit&&angle<=steeringLimit;
            boundFlag=(traj(1,idx)>xBound(1)&&traj(1,idx)<xBound(2))&&(traj(2,idx)>yBound(1)&&traj(2,idx)<yBound(2));
            %boundFlag=true;
            
            if(radiusFlag && angleFlag && boundFlag)
                tempAngle=atan2(dr(2),dr(1));
                steeringLimit=initialSteeringLimit;
                break;
            end
            
            if ~boundFlag
                steeringLimit=pi;
            end 
            
        end   
        
    end

end

% This function draws the vehicle.
function vehicleFig=drawVehicle(vehicle)

    alpha=linspace(0,3*pi,74);
    pos=vehicle.Position;
    theta=vehicle.Theta;

    dir=vehicle.Heading;
    R=vehicle.Radius;
    r=vehicle.wheel.radius;
    rotMatrix=[cos(theta) -sin(theta);sin(theta) cos(theta)];
    leftWheelLine=[pos pos]+rotMatrix*[-r r;R R];
    rightWheelLine=[pos pos]+rotMatrix*[-r r;-R -R];

    plot(pos(1),pos(2),'b.','LineWidth',2);
    hold on;
    
    vehicleFig=plot(pos(1)*ones(1,numel(alpha))+cos(alpha),pos(2)*ones(1,numel(alpha))+sin(alpha),'b',...
        [pos(1),pos(1)+dir(1)*2*R],[pos(2),pos(2)+dir(2)*2*R],'r',...    
        leftWheelLine(1,:),leftWheelLine(2,:),'k',...
        rightWheelLine(1,:),rightWheelLine(2,:),'k');
    set(vehicleFig,{'LineWidth'},{5;5;8;8});

end

function [nextPosition,nextTheta]=TwoWheeledVehicleDynamics(vehicle,deltaT)

    dPosition=deltaT*[vehicle.currentLinearVelocity*cos(vehicle.Theta);
        vehicle.currentLinearVelocity*sin(vehicle.Theta)];
    
    nextPosition=vehicle.Position+dPosition;
    
    nextTheta=vehicle.Theta+deltaT*vehicle.currentAngularVelocity;
    
end

% This function implements the PID controller to calculate the desired control
% commands
function [linearVelocity,omega,positionErrors,orientationErrors]=PIDControl(pidParam,goal,vehicle)

    dr=goal-vehicle.Position;

    %PID module for orientation
    kp=pidParam.orientation.K(1);
    ki=pidParam.orientation.K(2);
    kd=pidParam.orientation.K(3);
    currentThetaError=vehicle.Theta-atan2(dr(2),dr(1));
    thetaErrorSum=pidParam.orientation.errors.errorSum+currentThetaError;
    omega=kp*currentThetaError+ki*thetaErrorSum+kd*(currentThetaError-pidParam.orientation.errors.previousError);
    orientationErrors.previousError=currentThetaError;
    orientationErrors.errorSum=thetaErrorSum;

    %PID module for position
    kp=pidParam.position.K(1);
    ki=pidParam.position.K(2);
    kd=pidParam.position.K(3);
    currentPositionError=norm(dr);
    positionErrorSum=pidParam.position.errors.errorSum+currentPositionError;
    linearVelocity=kp*currentPositionError+ki*positionErrorSum+kd*(currentPositionError-pidParam.position.errors.previousError);
    linearVelocity=linearVelocity*cos(currentThetaError);
    positionErrors.previousError=currentPositionError;
    positionErrors.errorSum=positionErrorSum;    

end

% This function updates other state variables
function [Heading,leftRotSpd,rightRotSpd]=vehicleParamUpdate(vehicle)

    Heading=[cos(vehicle.Theta);sin(vehicle.Theta)];
    rightWheelVelocity=vehicle.currentLinearVelocity+vehicle.currentAngularVelocity*vehicle.Radius;
    leftWheelVelocity=vehicle.currentLinearVelocity-vehicle.currentAngularVelocity*vehicle.Radius;
    leftRotSpd=leftWheelVelocity/vehicle.wheel.radius;
    rightRotSpd=rightWheelVelocity/vehicle.wheel.radius;
    
end

% This function animates the tracking simulation while monitoring changing
% state variables
function stateMonitoringAnimate(vehicle,specifiedTraj,realTraj,idx)
    
    timeDomain=[0 realTraj.TimeStamp(end)];
    
    subplot(4,2,[1 3 5 7]);
    
    if(idx==1)
        
        drawTrajectory(specifiedTraj);
        
        xboundLimit=[min([specifiedTraj(1,:) vehicle.Position(1)]) max([specifiedTraj(1,:) vehicle.Position(1)])];
        yboundLimit=[min([specifiedTraj(2,:) vehicle.Position(2)]) max([specifiedTraj(2,:) vehicle.Position(2)])];
        Mid=0.5*(xboundLimit(1)+xboundLimit(2));
        Wid=0.5*(xboundLimit(2)-xboundLimit(1));
        xboundLimit=[Mid-1.2*Wid Mid+1.2*Wid];
        
        Mid=0.5*(yboundLimit(1)+yboundLimit(2));
        Wid=0.5*(yboundLimit(2)-yboundLimit(1));
        yboundLimit=[Mid-1.2*Wid Mid+1.2*Wid];
        
        xlim(xboundLimit);
        ylim(yboundLimit);
        
    end
    
    vehicleFig=drawVehicle(vehicle);
    pointFig=plot(specifiedTraj(1,idx),specifiedTraj(2,idx),'g*','LineWidth',5);
    
    subplot(422);
    plot(realTraj.TimeStamp(1:idx),sqrt(sum(realTraj.positionError(:,1:idx).^2)),'LineWidth',2);
    ylabel('Error (m)');
    title('Position Error');
    xlim(timeDomain);
    grid minor;
    
    subplot(424);
    plot(realTraj.TimeStamp(1:idx),realTraj.LinearVelocity(1:idx),'LineWidth',2);
    ylabel('V (m/s)');
    title('Linear Velocity in the Body Frame');
    xlim(timeDomain);
    grid minor;
    
    subplot(426);
    plot(realTraj.TimeStamp(1:idx),realTraj.AngularVelocity(1:idx),'LineWidth',2);
    ylabel('\Omega (rad/s)');
    title('Angular Velocity in the Body Frame');
    xlim(timeDomain);
    grid minor;
    
    subplot(428);
    fig=plot(realTraj.TimeStamp(1:idx),realTraj.WheelAngularVelocity(1,1:idx),realTraj.TimeStamp(1:idx),realTraj.WheelAngularVelocity(2,1:idx));
    set(fig,{'LineWidth'},{2;2});
    legend('Left','Right');
    title('Angular Velocities of Two Wheels')
    ylabel('\omega (rad/s)');
    xlabel('Time Stamps (s)');
    xlim(timeDomain);
    grid minor;
    
    if(idx~=1)
        pause(1e-9);
    else
        pause;
    end
    
    if(idx<size(specifiedTraj,2))
        set(vehicleFig,'Visible','off');
        set(pointFig,'Visible','off');
        clf(pointFig);
        clf(vehicleFig);
    end
    
end