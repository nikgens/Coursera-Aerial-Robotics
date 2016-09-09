function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

%u = params.mass*params.gravity;

% FILL IN YOUR CODE HERE

Kp = 99;
Kv = 18;
u = params.mass*(Kp *(s_des(1)-s(1))+ Kv *(s_des(2)-s(2)) + params.gravity);

end

