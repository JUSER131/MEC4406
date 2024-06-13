% % MEC4406 Robotic Arm Control
% % Author: Jack Halpin - U1119161
% % Date: 09/06/2024

% clear all
% close all

% % Section 1: Interpolating the trajectory.

% Define start and end point of the linear trajectory:
p_0 = [30; 30]; % Start
p_1 = [600; 600]; % End

% Define base point of the arm (within +30x, +30y of origin).

ox = 0;
oy = 0;i

% Calculate position vector
path = p_1 - p_0;

% Derive y = mx + c
gradient = path(2,1) ./ path(1,1);
y_intercept = p_1(2,1) - gradient*p_1(1,1);

% Calculate length of the line, this will allow for consistent 1mm
% increments for interpolation.

length_path_mm = sqrt(path(1,1)^2 + path(2,1)^2);

% Store x and y coordinates for the line in arrays x and y. 
x = linspace(p_0(1,1), p_1(1,1), length_path_mm/3);
y = gradient .* x + y_intercept;

% Add filler to trajectory so matrix operations lower are easier (less
% indexing)

fill = zeros(2,length(x));

trajectory = [x; y; fill];

% % Section 2: Inverse Kinematics

% Define symbolic variables;

% th1 = angle of link 1
% th2 = angle of link 2

syms th1 th2

% Define arm length, each arm will be equal length.

L = sqrt(600^2 + 600^2)/2;

% Random initial arm angles, not necessarily same as the start point of the trajectory.

init_pos_arm = [pi/4; pi/3]; 

% Calculate the transformation matrix for the arm (forward kinematics).

A1 = [cos(th1), -sin(th1), 0, L*cos(th1);
      sin(th1), cos(th1), 0, L*sin(th1);
      0, 0, 1, 0;
      0, 0, 0, 1];

A2 = [cos(th2), -sin(th2), 0, L*cos(th2);
      sin(th2), cos(th2), 0, L*sin(th2);
      0, 0, 1, 0;
      0, 0, 0, 1];

T = A1 * A2;

pos_function = T(:,4); % Stacked D-H Matrix for position of the end-effector.

% Define equation for the current position of the end effector given
% initial position of the arm.

effector_pos = subs(pos_function, [th1; th2],init_pos_arm);

% Define the loop for calculating the jacobian.
for R = 1:length(x);

    % Calculate symbolic jacobian matrix:
    J = jacobian(pos_function,[th1; th2]);
    % Substitute initial arm angles into the Jacobian:
    J_val = subs(J,[th1; th2], init_pos_arm);
    % Pseudo-inverse of the Jacobian:
    J_val_inv = pinv(J_val);

    delta_pos = trajectory(:,R) - effector_pos;
    delta_pos = 0.5*delta_pos;

    delta_vel = J_val_inv * delta_pos;
    
    new_pos_arm = vpa(init_pos_arm + [ delta_vel(1,1); delta_vel(2,1)]);

    init_pos_arm = new_pos_arm;

    effector_pos = subs(pos_function, [th1; th2], new_pos_arm);

    clf;

    % Plot the trajectory:
    
    hold on

    plot(x,y,'Marker','.','LineStyle','-');
    plot(p_0(1,1), p_0(2,1),'Marker','.','Linewidth',5,'Color','Blue');
    plot(p_1(1,1), p_1(2,1),'Marker','.','Linewidth',5,'Color','Red');

    grid on

    xlim([-350 600])
    ylim([-350 600])

    xticks(-350:30:600)
    yticks(-350:30:600)

    line([ox L*cos(init_pos_arm(1,1))], [oy L*sin(init_pos_arm(1,1))],[0 0],'Marker','o','Linestyle','-','Linewidth',5,'Color',[0 0.5 1])

    line([L*cos(init_pos_arm(1,1)) effector_pos(1,1)], [L*sin(init_pos_arm(1,1)) effector_pos(2,1)], [0 0],'Marker','o','Linestyle','-','Linewidth',5,'Color',[0 0 1])
       
    pause(0.01);

end
