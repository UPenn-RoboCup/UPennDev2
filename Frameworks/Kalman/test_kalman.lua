local kalman = require 'kalman'

for i=1,10 do
  print( kalman.get_ball( i/30, 10, 1 ) );
end
