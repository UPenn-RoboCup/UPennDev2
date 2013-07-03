function ret = arm_move(targetrelpos)
  global CONTROL 
  disp(sprintf('Pickup: target %.2f %.2f %.2f',...
							targetrelpos(1),targetrelpos(2),targetrelpos(3) ));
  CONTROL.send_control_packet('arm','pickup',targetrelpos);
end
