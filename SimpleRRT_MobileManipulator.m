clc
clear
close all
%syms L_1 L_2 theta_1 theta_2 XE YE startx starty

%% Manipulator 2R planar Dimensions
L1 = 1;
L2 = 0.5;

%% Map
% https://www.mathworks.com/matlabcentral/fileexchange/95998-autonomous-navigation-for-mobile-robots-and-ugv
size_map = 10;
res = 5;
myMap = binaryOccupancyMap(size_map,size_map,res);
ncel = size_map*res;

% Walls
walls = zeros(ncel,ncel);
walls(1,:) = 1; % Top wall
walls(end,:) = 1; % Bottom wall
walls(:,1) = 1; % Left wall
walls(:,end) = 1; % Right wall
walls(1:30,15) = 1; % Left division
walls(25:end,25) = 1; % Middle division
%walls(1:30,35) = 1; % Right division

setOccupancy(myMap,[1 1],walls,"grid");
inflate(myMap,0.3)
%show(myMap)

% 2D map to 3D
my3DMap = occupancyMap3D(5);
x = linspace(0,size_map,ncel)';
y = linspace(0,size_map,ncel)';
z = linspace(0,5,25);

% Arrange coordinates in XYZ format with increasing height
mapXY = [repelem(x,ncel) repmat(y,ncel,1)];
mapXYZ=[repmat(mapXY,25,1) repelem(z,2500)'];

% Get occupied space from 2D map and extrude in Z
occ3D = checkOccupancy(myMap,mapXY);
occ3D = repmat(occ3D,25,1);

% Set occupied space in the map
setOccupancy(my3DMap, mapXYZ, occ3D);

% Create map visualization
%show(my3DMap)

%% Geometric Objects - Manipulator Links 
width = 0.05;
link1_g = collisionBox(L1,width,1.5); % Link 1
link2_g = collisionBox(L2,width,1.5); % Link 2

%% RRT Algorithm

% Define the start and goal positions
start_tt1 = 0;
start_tt2 = 0;
start_base = [1, 1];
start_end_effector = [2.5, 1];
goal_end_effector = [8, 8];

% Define the size of the map
map_size = [10, 10];

% Number of nodes in the tree
num_nodes = 1000;

% Maximum step size
step_size = 1;

% Min distance to goal to consider that it reached
goal_distance = 0.5;

% Define the radius for checking nearby nodes
radius = 0.5;

% Save Link states
v_states = {};

% Initialize the tree with the start node
tree = start_base;
tree_angle1 = [0];
tree_angle2 = [0];
parent = 1;

% Plot the start and goal positions
figure(1)
plot(start_base(1), start_base(2), '.', 'MarkerSize', 20, 'LineWidth', 2);
hold on;
plot(goal_end_effector(1), goal_end_effector(2), '.', 'MarkerSize', 20, 'LineWidth', 2);
xlim([0 10])
ylim([0,10])

% Variables
path = [];
link1_path = {};
link2_path = {};
link1_path_aux={};
link2_path_aux={};
cnt = 1;

for i = 1:num_nodes

    % Generate a state
    rand_tt1 = 2*pi*rand() + pi;
    rand_tt2 = 2*pi*rand() + pi;
    random_point = [rand()*10, rand()*10];
    base = [random_point(1), random_point(2)];

    % Find the nearest node in the tree
    distances = vecnorm(tree - base, 2, 2);
    [min_dist, nearest_idx] = min(distances);
    nearest_node = tree(nearest_idx, :);
    nearest_angle1 = tree_angle1(nearest_idx);
    nearest_angle2 = tree_angle2(nearest_idx);

    % Compute a new node towards the random point
    direction = (base - nearest_node) / min_dist;
    new_node = nearest_node + direction * min(step_size, min_dist);   

    % Check if the new node is in collision with obstacles
    [lk1, lk2] = att_link(link1_g,link2_g,new_node(1),new_node(2),rand_tt1,rand_tt2);
    collisionStatus1 = checkMapCollision(my3DMap,lk1);
    collisionStatus2 = checkMapCollision(my3DMap,lk2);
    
    % Initial and target status
    x = nearest_node(1);
    y = nearest_node(2);
    x1 = new_node(1);
    y1 = new_node(2);
    angle1 = nearest_angle1;
    angle2 = nearest_angle2;
    new_angle1 = rand_tt1;
    new_angle2 = rand_tt2;

    % Interpolation between two status
    num_points = 10;    
    x_values = linspace(x, x1, num_points);
    y_values = linspace(y, y1, num_points);
    line_vector = [x_values; y_values];
    line_vector = line_vector';     
    theta1_values = linspace(angle1, new_angle1, num_points);
    theta2_values = linspace(angle2, new_angle2, num_points);

    % Check whole interpolation for collision
    collide = 0; 
    if collisionStatus1 == 0 && collisionStatus2 == 0
        for j = 1:1:num_points
                [lk1, lk2] = att_link(link1_g,link2_g,x_values(j),y_values(j),theta1_values(j),theta2_values(j));
                collisionStatus1 = checkMapCollision(my3DMap,lk1);
                collisionStatus2 = checkMapCollision(my3DMap,lk2);
                if collisionStatus2 == 1 || collisionStatus1 == 1
                    collide = 1;
                    break                
                end
        end
        
        % If doesn't collide, add new node to the tree
        if collide == 0
            cnt = cnt + 1;
            % Add the new node to the tree
            tree = [tree; new_node];
            tree_angle1 = [tree_angle1,new_angle1];
            tree_angle2 = [tree_angle2,new_angle2];
            parent = [parent; nearest_idx]; % Update parent index
            %link1_path_aux{cnt} = copy(lk1); % Save state link1
            %link2_path_aux{cnt} = copy(lk2); % Save state link2

            % Plot the new node
            plot(new_node(1), new_node(2), 'g.', 'MarkerSize', 10);
            hold on
            show(myMap)
            hold on
            show(lk1)
            show(lk2)        
            drawnow;

            % Check if we have reached the goal
            if norm(new_node - goal_end_effector) < goal_distance                
                disp('Goal reached!');
                % Print the path
                path = [goal_end_effector];
                node_idx = size(tree, 1);
                cnt2 = 0;
                while node_idx ~= 1
                    cnt2 = cnt2+1;
                    path = [tree(node_idx, :); path];
                    % Save states
                    v_states{cnt2} = [tree(node_idx,1),tree(node_idx,2),tree_angle1(node_idx),tree_angle2(node_idx)];
                    node_idx = parent(node_idx);
                end
                path = [start_base; path];
                disp('Path:');
                disp(path);                
                break;
            end
        end
    end
end

%% Plot States Only
figure(3)
show(myMap)
hold on
plot(path(2:end-1,1),path(2:end-1,2),'.','MarkerSize',20);
hold on
v = flip(v_states);
for k = 1:1:length(v)
    [lk1, lk2] = att_link(link1_g,link2_g,v{k}(1),v{k}(2),v{k}(3),v{k}(4));
    show(lk1);
    show(lk2);
    hold on
end

%% Plot All Path
figure(4)
show(myMap)
hold on
plot(path(2:end-1,1),path(2:end-1,2),'.','MarkerSize',20);
hold on
v = flip(v_states);
for k = 1:1:length(v)-1
    [lk1, lk2] = att_link(link1_g,link2_g,v{k}(1),v{k}(2),v{k}(3),v{k}(4));
    show(lk1);
    show(lk2);

    num_points = 20;
    x1 = v{k+1}(1);
    y1 = v{k+1}(2);
    x1a = v{k}(1);
    y1a = v{k}(2);
    angle1 = v{k+1}(3);
    angle2 = v{k+1}(4);
    angle1a = v{k}(3);
    angle2a = v{k}(4);    

    x_values = linspace(x1a, x1, num_points);
    y_values = linspace(y1a, y1, num_points);
    theta1_values = linspace(angle1a, angle1, num_points);
    theta2_values = linspace(angle2a, angle2, num_points);
    line_vector = [x_values; y_values];
    line_vector = line_vector';

    for j = 1:1:num_points
        [lk1, lk2] = att_link(link1_g,link2_g,x_values(j),y_values(j),theta1_values(j),theta2_values(j));
        collisionStatus1 = checkMapCollision(my3DMap,lk1);
        collisionStatus2 = checkMapCollision(my3DMap,lk2);
        plot(x_values(j), y_values(j), 'g.', 'MarkerSize', 10);
        hold on
        show(myMap)
        hold on
        show(lk1)
        show(lk2)        
        %drawnow;
    end

end

%% Navigation Animation
animation = 0;
if animation == 1    
    saveGif = 0;
    figure(4)
    show(myMap)
    hold on
    plot(path(2:end-1,1),path(2:end-1,2),'.','MarkerSize',20);
    hold on
    v = flip(v_states);
    for k = 1:1:length(v)-1
        [lk1, lk2] = att_link(link1_g,link2_g,v{k}(1),v{k}(2),v{k}(3),v{k}(4));
        show(lk1);
        show(lk2);
    
        num_points = 20;
        x1 = v{k+1}(1);
        y1 = v{k+1}(2);
        x1a = v{k}(1);
        y1a = v{k}(2);
        angle1 = v{k+1}(3);
        angle2 = v{k+1}(4);
        angle1a = v{k}(3);
        angle2a = v{k}(4);    
    
        x_values = linspace(x1a, x1, num_points);
        y_values = linspace(y1a, y1, num_points);
        theta1_values = linspace(angle1a, angle1, num_points);
        theta2_values = linspace(angle2a, angle2, num_points);
        line_vector = [x_values; y_values];
        line_vector = line_vector';
    
        for j = 1:1:num_points
            [lk1, lk2] = att_link(link1_g,link2_g,x_values(j),y_values(j),theta1_values(j),theta2_values(j));
            collisionStatus1 = checkMapCollision(my3DMap,lk1);
            collisionStatus2 = checkMapCollision(my3DMap,lk2);
            plot(x_values(j), y_values(j), 'g.', 'MarkerSize', 10);
            hold on
            show(myMap)
            hold on
            show(lk1)
            show(lk2)        
            drawnow;
            if saveGif == 1
                exportgraphics(gcf,'testAnimated.gif','Append',true);
            end
            hold off
    
        end
    
    end
end

%% Function to set links based on configuration
function [link_1, link_2] = att_link(link1,link2,x,y,theta1,theta2)
    
    % Initialize links
    link1_aux = copy(link1);
    link2_aux = copy(link2);
    axis = [0 0 1];
    L1 = link1_aux.X;
    L2 = link2_aux.X;
    
    % Joint position (only one between them)
    xj = L1*cos(theta1) + x;
    yj = L1*sin(theta1) + y;

    % Homogenous Transformation Link1
    translation = [xj-(L1/2)*cos(theta1), yj-(L1/2)*sin(theta1),0];
    rotation = [axis theta1];
    H_translation = trvec2tform(translation);
    H_rotation = axang2tform(rotation);
    H = H_translation * H_rotation;
    link1_aux.Pose = H;
    
    % Homogenous Transformation Link2         
    translation = [xj+(L2/2)*cos(theta2+theta1),yj+(L2/2)*sin(theta2+theta1),0];
    rotation = [axis theta2+theta1];
    H_translation = trvec2tform(translation);
    H_rotation = axang2tform(rotation);
    H = H_translation * H_rotation;
    link2_aux.Pose = H;
    
    % Return
    link_1 = link1_aux;
    link_2 = link2_aux;
end