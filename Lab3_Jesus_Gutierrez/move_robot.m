function move_robot(image_count,bot,botjoints)

persistent tau

if isempty(tau)
	tau=0.15;
end

if image_count>20
	%'undo' previous input
% 	jointTorque(bot, botjoints{2}, 1, -tau);
	% get new torque
	tau = -tau;
	% apply torque
	jointTorque(bot, botjoints{2}, 1, tau)
end