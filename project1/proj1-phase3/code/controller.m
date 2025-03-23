function [F, M] = controller(t, s, s_des)
    % s(1:3) current position
    % s(4:6) current velocity
    % s(7:10) current attitude quaternion
    % s(11:13) current body angular velocity
    
    % s_des(1:3) desire position
    % s_des(4:6) desire velocity
    % s_des(7:9) desire acceleration
    % s_des(10) desire yaw
    % s_des(11) desire yaw rate

    global params
    persistent int_e_p int_e_angle prev_t

    % define position control gains (PID)
    Kp_pos = diag([10, 10, 10]); % position p gain
    Kd_pos = diag([8, 8, 8]); % position d gain
    Ki_pos = diag([1, 1, 1]); % position i gain

    % define attitude control gains (PID)
    Kp_att = diag([3000, 3000, 3000]); % attitude p gain
    Kd_att = diag([100, 100, 100]); % attitude d gain
    Ki_att = diag([1, 1, 1]); % attitude i gain

    % extract paras
    m = params.mass;
    g = params.grav;
    I = params.I;

    % extract current state
    p = s(1:3); % position
    v = s(4:6); % velocity
    q = s(7:10);    % attitude quaternion
    w = s(11:13);   % angular velocity
    
    % extract desired state
    p_des = s_des(1:3); % position
    v_des = s_des(4:6); % velocity
    a_des = s_des(7:9); % acceleration
    yaw_des = s_des(10);    % yaw
    yaw_rate_des = s_des(11); % yaw rate
    
    % Initialize integral errors
    if isempty(int_e_p)
        int_e_p = zeros(3,1);
        int_e_angle = zeros(3,1);
        prev_t = t;
    end

    % Time step
    dt = t - prev_t;
    prev_t = t;

    % errors
    e_p = p_des - p;    % position error
    e_v = v_des - v;    % velocity error

    % Integral error update
    int_e_p = int_e_p + e_p * dt;

    %%% position control -> Thrust F
    a_cmd = a_des + Kd_pos * e_v + Kp_pos * e_p + Ki_pos * int_e_p; % commanded acceleration
    F = m * (a_cmd(3) + g); % F as scalar

    %%% attitude control -> Moment M
    % current euler angles
    Rot_matrix = quaternion_to_R(q);
    [phi_cur, theta_cur, psi_cur] = RotToRPY_ZXY(Rot_matrix);
    
    % desired orientation (theta, phi, psi) -> (x_roll, y_pitch, z_yaw)
    phi_c = 1/g * (a_cmd(1) * sin(psi_cur) - a_cmd(2) * cos(psi_cur));
    theta_c = 1/g * (a_cmd(1) * cos(psi_cur) + a_cmd(2) * sin(psi_cur));
    psi_c = yaw_des;

    % desired angular velocity
    phi_dot_c = 0;
    theta_dot_c = 0;
    psi_dot_c = yaw_rate_des;

    

    % angle errors (need clamp)
    e_angle = [atan2(sin(phi_c - phi_cur), cos(phi_c - phi_cur)); 
                atan2(sin(theta_c - theta_cur), cos(theta_c - theta_cur)); 
                atan2(sin(psi_c - psi_cur), cos(psi_c - psi_cur))];
    
    % Integral error update for attitude
    int_e_angle = int_e_angle + e_angle * dt;

    % angular velocity errors (no need to clamp)
    e_w = [phi_dot_c; theta_dot_c; psi_dot_c] - w;

    % desired angular acceleration
    w_dot_c = Kp_att * e_angle + Kd_att * e_w + Ki_att * int_e_angle;

    % Moment M
    M = I * w_dot_c + cross(w, I * w);

end
