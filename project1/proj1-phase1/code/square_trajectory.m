function s_des = square_trajectory(t, true_s)
    s_des = zeros(11,1);
% Given yaw, DO NOT CHANGE
    yaw_des = mod(0.2 * pi * t,2 * pi);
    dyaw_des = 0.2 * pi;
    
% TODO: Implement square trajectory here
    omega=25;
    
    waypoints = [
        0, 0, 0;
        1, 2, 0;
        2, 2, 2;
        3, 0, 2;
        4, 0, 0;
    ];

    % Calculate the time for each segment
    total_duration = omega;
    segment_duration = total_duration / (size(waypoints, 1) - 1);

    % check in which segment
    segment_index = floor(t / segment_duration) + 1;
    if segment_index > size(waypoints, 1)   % bound check
        segment_index = size(waypoints, 1);
    end

    % linear interpolation ratio
    segment_start_t = (segment_index - 1) * segment_duration;
    segment_end_t = segment_index * segment_duration;
    ratio = (t -segment_start_t) / segment_duration;

    % if in the last segment, no interpolation & ratio is 1
    if segment_index == size(waypoints, 1)
        ratio = 1;
    end
    % interpolation
    if segment_index < size(waypoints, 1)
        x_des = (1 - ratio) * waypoints(segment_index, 1) + ratio * waypoints(segment_index + 1, 1);
        y_des = (1 - ratio) * waypoints(segment_index, 2) + ratio * waypoints(segment_index + 1, 2);
        z_des = (1 - ratio) * waypoints(segment_index, 3) + ratio * waypoints(segment_index + 1, 3);

        x_vdes = (waypoints(segment_index + 1, 1) - waypoints(segment_index, 1)) / segment_duration;
        y_vdes = (waypoints(segment_index + 1, 2) - waypoints(segment_index, 2)) / segment_duration;
        z_vdes = (waypoints(segment_index + 1, 3) - waypoints(segment_index, 3)) / segment_duration;

        % no acceleration
        x_ades = 0;
        y_ades = 0;
        z_ades = 0;
    else    % at the last waypoint
        x_des = waypoints(end, 1);
        y_des = waypoints(end, 2);
        z_des = waypoints(end, 3);
        x_vdes = 0;
        y_vdes = 0;
        z_vdes = 0;
        x_ades = 0;
        y_ades = 0;
        z_ades = 0;
    end
   
    

    s_des(1)=x_des; 
    s_des(2)=y_des; 
    s_des(3)=z_des; 
    s_des(4)=x_vdes; 
    s_des(5)=y_vdes; 
    s_des(6)=z_vdes;
    s_des(7)=x_ades; 
    s_des(8)=y_ades; 
    s_des(9)=z_ades;
    s_des(10)=yaw_des; 
    s_des(11)=dyaw_des; 
    
end
