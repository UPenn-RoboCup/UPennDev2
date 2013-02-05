function fid = dynamixelOpen(port, baud);

global DYNAMIXEL

if nargin < 2,
    port = '/dev/tty.usbserial-A7007xPV';
%  port = '/dev/tty.usbserial-A10019XW';
end
if nargin < 3,
    baud = 1000000; % Default 1Mbps
end

if isempty(DYNAMIXEL),
  DYNAMIXEL.fid = serialopen(port, baud);
end
