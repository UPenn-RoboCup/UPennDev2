dofile('init.lua')
require('unix')
require('Body')
require('SoundComm')

SoundComm.set_transmitter_volume(50)

symbols = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '#', '*', 'A', 'B', 'C', 'D'};

lastPress = unix.time();

-- turn off recevier
SoundComm.pause_receiver();

-- continuous
continuous = false;
lastTx = unix.time();

while (1) do

   if (continuous) then
      if (Body.get_sensor_bumperLeft()[1] == 1 
            and Body.get_sensro_bumperRight()[1] == 1
            and unix.time() - lastPress > 1) then
         continuous = false;
         lastPress = unix.time();
      else
         if (unix.time() - lastTx > 2) then
            -- pick random tone symbol
            ind = math.random(#symbols);
            symbol = symbols[ind];

            -- play sequence
            SoundComm.play_pnsequence(symbol);

            lastTx = unix.time();
         end
      end 
   else
     -- button pressed?
     if (Body.get_sensor_button()[1] == 1 and unix.time() - lastPress > 1) then
        -- pick random tone symbol
        ind = math.random(#symbols);
        symbol = symbols[ind];

        -- play sequence
        SoundComm.play_pnsequence(symbol);

        lastPress = unix.time();
     elseif (Body.get_sensor_bumperLeft()[1] == 1 and Body.get_sensro_bumperRight()[1] == 1) then
        -- enable continuous transmittion
        continuous = true;
        lastPress = unix.time();
     end
   end
   

   unix.usleep(100000);
end
