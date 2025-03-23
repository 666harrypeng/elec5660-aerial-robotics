function s_des = trajectory_generator(t, path, h)

    persistent P;
    persistent N;
    persistent total_time;
    persistent A;
    persistent d;
    persistent Q_k;
    persistent time_stamp;
    persistent time_intervals;
    persistent segments_num;
    persistent des_positions;

    if nargin > 1 % pre-process can be done here (given waypoints)

        N = 8; % Minimum snap
        total_time = 20.0; % total time for the trajectory
        waypoints_num = length(path); % number of waypoints
        segments_num = waypoints_num - 1; % number of segments

        % time allocation based on distance
        seg_length = vecnorm(diff(path), 2, 2);
        time_stamp = [0; cumsum(seg_length) / sum(seg_length) * total_time]';
        time_intervals = time_stamp(2:end) - time_stamp(1:end-1); 

        % disp(size(time_stamp))
        % disp(size(time_intervals))
        % disp(time_stamp)
        % disp(time_intervals)
        
        % cost function matrix Q_k
        Q_k = zeros(N * segments_num, N * segments_num);
        for k = 1:segments_num
            for i = 4:N
                for j = 4:N
                    Q_k((k-1)*N + i, (k-1)*N + j) = i * (i-1) * (i-2) * (i-3) * j * (j-1) * (j-2) * (j-3) / (i + j - 7) * (time_intervals(k)^(i + j - 7));
                end
            end
        end

        % disp(size(Q_k))

        %%% derivative constriant A & d
        P = zeros(N * segments_num, 3);
        % derivative constraints -> start & end -> 2 * segments_num
        % continuity constraints -> position, velocity, acceleration, jerk -> 4 * (segments_num - 1) -> applied to (segments_num - 1) waypoints
        % start & end conditions -> 3 * 2 -> applied to the first and last waypoints
        % total constraints -> 6 * segments_num + 2
        A = zeros(6 * segments_num + 2, N * segments_num);
        d = zeros(6 * segments_num + 2, 3);
        d(1 : 2 * segments_num, :) = reshape([path(1:end-1, :) path(2:end, :)]', 3, [])';

        % position constraint
        for i = 1:segments_num
            A(2*i-1 : 2*i, N*i-7 : N*i) = [ ...
                1, 0, 0, 0, 0, 0, 0, 0; ...
                1, time_intervals(i), time_intervals(i)^2, time_intervals(i)^3, time_intervals(i)^4, time_intervals(i)^5, time_intervals(i)^6, time_intervals(i)^7
            ];
        end

        % continuity constraint
        for i = 1 : (segments_num - 1)
            A((2*segments_num + 4*i - 3):(2*segments_num + 4*i), (8*i - 7):(8*i + 8)) = [ ...
                1, time_intervals(i), time_intervals(i)^2, time_intervals(i)^3, time_intervals(i)^4, time_intervals(i)^5, time_intervals(i)^6, time_intervals(i)^7, -1, 0, 0, 0, 0, 0, 0, 0; ...
                0, 1, 2*time_intervals(i), 3*time_intervals(i)^2, 4*time_intervals(i)^3, 5*time_intervals(i)^4, 6*time_intervals(i)^5, 7*time_intervals(i)^6, 0, -1, 0, 0, 0, 0, 0, 0;  ...
                0, 0, 2, 6*time_intervals(i), 12*time_intervals(i)^2, 20*time_intervals(i)^3, 30*time_intervals(i)^4, 42*time_intervals(i)^5, 0, 0, -2, 0, 0, 0, 0, 0; ...
                0, 0, 0, 6, 24*time_intervals(i), 60*time_intervals(i)^2, 120*time_intervals(i)^3, 210*time_intervals(i)^4, 0, 0, 0, -6, 0, 0, 0, 0
            ];
        end
        
        % start & end conditions(velocity, acceleration, jerk)
        A(6 * segments_num - 3 : 6 * segments_num - 1, 1 : N) = [ ...
            0, 1, 0, 0, 0, 0, 0, 0; ...
            0, 0, 2, 0, 0, 0, 0, 0; ...
            0, 0, 0, 6, 0, 0, 0, 0
        ];
        A(6 * segments_num : 6 * segments_num + 2, N * segments_num - 7 : N * segments_num) = [ ...
            0, 1, 2*time_intervals(segments_num), 3*time_intervals(segments_num)^2, 4*time_intervals(segments_num)^3, 5*time_intervals(segments_num)^4, 6*time_intervals(segments_num)^5, 7*time_intervals(segments_num)^6; ...
            0, 0, 2, 6*time_intervals(segments_num), 12*time_intervals(segments_num)^2, 20*time_intervals(segments_num)^3, 30*time_intervals(segments_num)^4, 42*time_intervals(segments_num)^5; ...
            0, 0, 0, 6, 24*time_intervals(segments_num), 60*time_intervals(segments_num)^2, 120*time_intervals(segments_num)^3, 210*time_intervals(segments_num)^4
        ];

        % quadratic programming
        f = zeros(N * segments_num, 1);
        P(:, 1) = quadprog(Q_k, f, [], [], A, d(:, 1));
        P(:, 2) = quadprog(Q_k, f, [], [], A, d(:, 2));
        P(:, 3) = quadprog(Q_k, f, [], [], A, d(:, 3));

        % disp(size(P))
        % disp(size(Q_k))
        % disp(size(f))
        % disp(size(A))
        % disp(size(d))
    else % output desired trajectory here (given time)
        
        % s_des(1:3) desire position
        % s_des(4:6) desire velocity
        % s_des(7:9) desire acceleration
        % s_des(10) desire yaw
        % s_des(11) desire yaw rate
    
        s_des = zeros(13, 1);

        % if time exceeds last segment, hover at the last waypoint
        if t > time_stamp(end)
            s_des(1:3) = des_positions;
            s_des(7:10) = [1; 0; 0; 0];
            return;
        end
        
        % determine active segment
        seg_idx = find(t >= time_stamp(1:end-1) & t < time_stamp(2:end), 1, "first");
        if isempty(seg_idx)
            disp("error: time exceeds last segment");
            seg_idx = segments_num;
        end

        % normalize time within segment
        % T_norm = (t - time_stamp(seg_idx)) / time_intervals(seg_idx);
        T_norm = t - time_stamp(seg_idx);
        % disp(T_norm)
        
        % compute desired state
        T_vec = [1, T_norm, T_norm^2, T_norm^3, T_norm^4, T_norm^5, T_norm^6, T_norm^7];
        dT_vec = [0, 1, 2*T_norm, 3*T_norm^2, 4*T_norm^3, 5*T_norm^4, 6*T_norm^5, 7*T_norm^6];
        ddT_vec = [0, 0, 2, 6*T_norm, 12*T_norm^2, 20*T_norm^3, 30*T_norm^4, 42*T_norm^5];

        % s_des(1:3) = T_vec * P((seg_idx-1)*N + (1:N), :);
        % s_des(4:6) = dT_vec * P((seg_idx-1)*N + (1:N), :);
        % s_des(7:9) = ddT_vec * P((seg_idx-1)*N + (1:N), :);
        s_des(1:3) = T_vec * P((seg_idx-1)*N + (1:N), :);
        s_des(4:6) = dT_vec * P((seg_idx-1)*N + (1:N), :);
        % s_des(7:9) = ddT_vec * P((seg_idx-1)*N + (1:N), :);
        s_des(7:10) =[1;0;0;0];


        des_positions = s_des(1:3);

        % s_des(10) = 0; % yaw
        % s_des(11) = 0; % yaw rate
    end


end



