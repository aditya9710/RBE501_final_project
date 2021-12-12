clear, clc, close all
addpath('utils');

%% model based on PSMFK.py

robot_psmfk = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...       %q(1)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(2)
                    PrismaticMDH('a', 0, 'theta', 0, 'alpha', pi/2, 'offset', -4.389), ...      %q(3)
                    RevoluteMDH('a', 0, 'd', 4.16, 'alpha', 0, 'offset', 0), ...                %q(4)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(5)
                    RevoluteMDH('a', 0.09, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...        %q(6)
                    RevoluteMDH('a', 0, 'd', 0.106, 'alpha', -pi/2, 'offset', pi/2)], ...       %q(7)
                    'name', '6 DoF dVRK');

qlim = [-pi/2  pi/2;  % q(1)
        -pi/3  pi/3;  % q(2)
     30*0.0254 150*0.0254;    % q(3)
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/3  pi/3;  % q(6)
           0     0];    % q(7)
       
for i = 1:size(qlim,1)
    robot_psmfk.links(i).qlim = qlim(i,:);
end
close(figure(1))
figure(1)
q = zeros(1,7);
robot_psmfk.teach(q);

%% model based on user guide  

%%% unit: m

% robot_userguide = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...             %q(1)
%                     RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(2)
%                     PrismaticMDH('a', 0, 'theta', 0, 'alpha', pi/2, 'offset', -0.4318), ...     %q(3)
%                     RevoluteMDH('a', 0, 'd', 0.416, 'alpha', 0, 'offset', 0), ...               %q(4)
%                     RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(5)
%                     RevoluteMDH('a', 0.0091, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...       %q(6)
%                     RevoluteMDH('a', 0, 'd', 0.0102, 'alpha', -pi/2, 'offset', 0)], ...      %q(7)
%                     'name', '6 DoF dVRK');
% 
% qlim = [-pi/2  pi/2;  % q(1)
%         -pi/2  pi/2;  % q(2)
%      1.2*0.0254 5.0*0.0254;    % q(3)
%         -pi/2  pi/2;  % q(4)
%         -pi/2  pi/3;  % q(5)
%         -pi/3  pi/3;  % q(6)
%            0     0];    % q(7)
%        
% for i = 1:size(qlim,1)
%     robot_userguide.links(i).qlim = qlim(i,:);
% end
% close(figure(1))
% figure(1)
% q = zeros(1,7);
% % q(2) = -pi/3;
% robot_userguide.teach(q);


%% model based on wpi paper, full

%%% can be used for visualize, play with it

%%% unit:m

robot_wpi = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...             %q(1)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(2)
                    RevoluteMDH('a', 0.04009, 'd', 0, 'alpha', 0, 'offset', pi/2), ...           %q(2')
                    RevoluteMDH('a', 0.14454, 'd', 0, 'alpha', 0, 'offset', pi/2), ...           %q(2'')
                    RevoluteMDH('a', 0.516, 'd', 0, 'alpha', 0, 'offset', 0), ...           %q(2'''')
                    PrismaticMDH('a', 0.04009, 'theta', 0, 'alpha', -pi/2, 'offset', -0.28726), ...     %q(3)
                    RevoluteMDH('a', 0, 'd', 0.4162, 'alpha', 0, 'offset', 0), ...               %q(4)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...           %q(5)
                    RevoluteMDH('a', 0.0091, 'd', 0, 'alpha', -pi/2, 'offset', pi/2), ...       %q(6)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', 0, 'offset', 0)], ...      %q(7)
                    'name', '6 DoF dVRK');

qlim = [ -pi/2  pi/2;  % q(1)
        -pi/3  pi/3;  % q(2)
            0  0;       % q(2')
         -pi/3 pi/3;     % q(2'')
         -pi/3 pi/3;     % q(2'''')
     3*0.0254 15.0*0.0254;    % q(3)
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/3  pi/3;  % q(6)
           0     0];    % q(7)
       
% RevoluteMDH('a', 0.18262, 'd', 0, 'alpha', 0, 'offset', pi/2), ...           %q(2''')
% -pi/2 pi/2;     % q(2''')    
for i = 1:size(qlim,1)
    robot_wpi.links(i).qlim = qlim(i,:);
end

q = zeros(1,10);
% robot_wpi.teach(q);

%%% test
close(figure(1))
figure(1)
q_test = q;
% q_test(2) = pi/3;
% q_test(4) = -pi/3;
% q_test(5) = pi/3;
robot_wpi.teach(q_test);

%% model based on wpi paper, simplified

%%% unit:m

robot_wpisimplified = SerialLink([RevoluteMDH('a', 0, 'd', 0, 'alpha', pi/2, 'offset', pi/2), ...             %q(1)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(2)
                    PrismaticMDH('a', 0, 'theta', 0, 'alpha', pi/2, 'offset', -0.4318), ...     %q(3)
                    RevoluteMDH('a', 0, 'd', 0.4162, 'alpha', 0, 'offset', 0), ...               %q(4)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...           %q(5)
                    RevoluteMDH('a', 0.0091, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...       %q(6)
                    RevoluteMDH('a', 0, 'd', 0, 'alpha', 0, 'offset', pi)], ...      %q(7)
                    'name', '6 DoF dVRK');

qlim = [-pi/2  pi/2;  % q(1)
        -pi/3  pi/3;  % q(2)
     3*0.0254 15.0*0.0254;    % q(3)
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/3  pi/3;  % q(6)
           0     0];    % q(7)
       
for i = 1:size(qlim,1)
    robot_wpisimplified.links(i).qlim = qlim(i,:);
end
close(figure(1))
figure(1)
q = zeros(1,7);
% q(2) = pi/3
% q(2) = -pi/3;
robot_wpisimplified.teach(q);
% robot_wpisimplified.teach(q,'zoom',0.4);





