function Optimal_path = path_from_A_star(map)
    Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    MAX_Z=10;
    
    %Define the 3D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    zval=floor(map(size_map, 3));
    
    xTarget=xval;
    yTarget=yval;
    zTarget=zval;
    MAP(xval,yval,zval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        zval=floor(map(i, 3));
        MAP(xval,yval,zval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    zval=floor(map(1, 3));
    xStart=xval;
    yStart=yval;
    zStart=zval;
    MAP(xval,yval,zval)=1;
    
    % Main structure in the A* search =====================================================

    % Container storing nodes to be expanded, along with the f score (f=g+h)
    % Each node's (x,y,z) coordinate and its f score is stored in a row
    % For example, queue = [x1, y1, z1, f1; x2, y2, z2, f2; ...; xn, yn, zn, fn]
    queue = [];  

    % Arrays for storing the g score of each node, g score of undiscovered nodes is inf
    g = inf(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording whether a node is expanded (popped from the queue) or not
    % expanded: 1, not expanded: 0
    expanded = zeros(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording the parent of each node
    parents = zeros(MAX_X,MAX_Y,MAX_Z, 3);
    
    
    %Start your code here ==================================================================
    
    % Define heuristic type ('euclidean' or 'manhattan')
    heuristic_type = 'manhattan';
    
    % init g for starting node
    g(xStart, yStart, zStart) = 0;

    % h & f for starting node 
    h_start = calculate_heuristic(xStart, yStart, zStart, xTarget, yTarget, zTarget, heuristic_type);
    f_start = g(xStart, yStart, zStart) + h_start;

    % add start node to queue
    queue = [xStart, yStart, zStart, f_start];

    % 6 movements (up, down, left, right, fd, bd)
    movements = [0, 0, 1; ...   % up
                 0, 0, -1; ...  % down
                 0, 1, 0; ...   % right
                 0, -1, 0; ...  % left
                 1, 0, 0; ...   % forward
                 -1, 0, 0];   % backward

    % main loop for A*
    while ~isempty(queue)
        % min f node
        [~, min_f_idx] = min(queue(:, 4));
        current_node = queue(min_f_idx, 1:3);

        % remove current node from queue
        queue(min_f_idx, :) = [];

        x_cur_node = current_node(1);
        y_cur_node = current_node(2);
        z_cur_node = current_node(3);

        % check visited
        if expanded(x_cur_node, y_cur_node, z_cur_node) == 1
            continue;
        end

        % if not visited -> mark visited
        expanded(x_cur_node, y_cur_node, z_cur_node) = 1;

        % check if current node is target
        if x_cur_node == xTarget && y_cur_node == yTarget && z_cur_node == zTarget
            break;
        end

        % expand current node
        for i = 1 : size(movements, 1)  % all neighbors
            % neighbor node
            x_new_node = x_cur_node + movements(i, 1);
            y_new_node = y_cur_node + movements(i, 2);
            z_new_node = z_cur_node + movements(i, 3);

            % check if neighbor node is within the map
            if x_new_node < 1 || x_new_node > MAX_X || y_new_node < 1 || y_new_node > MAX_Y || z_new_node < 1 || z_new_node > MAX_Z
                continue;
            end

            % check obstacle
            if MAP(x_new_node, y_new_node, z_new_node) == -1
                continue;
            end

            % check visited
            if expanded(x_new_node, y_new_node, z_new_node) == 1
                continue;
            end

            % calculate tentative g -> g(m)
            tentative_g = g(x_cur_node, y_cur_node, z_cur_node) + 1;

            % if g(m) is inf -> push to queue
            if g(x_new_node, y_new_node, z_new_node) == inf
                % update parent
                parents(x_new_node, y_new_node, z_new_node, :) = [x_cur_node, y_cur_node, z_cur_node];
                % calculate h
                h = calculate_heuristic(x_new_node, y_new_node, z_new_node, xTarget, yTarget, zTarget, heuristic_type);
                % calculate f
                f = tentative_g + h;
                % add to queue
                queue = [queue; x_new_node, y_new_node, z_new_node, f];

            % if new path is better -> if g(m) > g(n) + Cnm
            if tentative_g < g(x_new_node, y_new_node, z_new_node)
                % update g
                g(x_new_node, y_new_node, z_new_node) = tentative_g;
                
            end
        end
    end

    % reconstruct path
    if expanded(xTarget, yTarget, zTarget) == 1 % reach target
        % backtrack from target to start
        current_node = [xTarget, yTarget, zTarget];
        path = current_node;

        % follow parents
        while ~(current_node(1) == xStart && current_node(2) == yStart && current_node(3) == zStart)
            current_node = squeeze(parents(current_node(1), current_node(2), current_node(3), :))'; % before squeeze, (1, 1, 1, 3) -> after squeeze (3, 1) -> need transpose
            path = [current_node; path];
        end

        % return path
        Optimal_path = path;
        disp('Path found');
        %disp(Optimal_path);
    else % no path
        Optimal_path = [];
        disp('No path found');
    end

end

function h = calculate_heuristic(x1, y1, z1, x2, y2, z2, type)
    % Calculate heuristic distance between two 3D points
    % Inputs:
    %   x1, y1, z1: coordinates of first point
    %   x2, y2, z2: coordinates of second point
    %   type: string specifying heuristic type ('euclidean' or 'manhattan')
    % Output:
    %   h: heuristic distance value
    
    switch lower(type)
        case 'euclidean'
            % Euclidean distance
            h = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
        case 'manhattan'
            % Manhattan distance
            h = abs(x1-x2) + abs(y1-y2) + abs(z1-z2);
        otherwise
            error('Unknown heuristic type. Use either ''euclidean'' or ''manhattan''');
    end
end
