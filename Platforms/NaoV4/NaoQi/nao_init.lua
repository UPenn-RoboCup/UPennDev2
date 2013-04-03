package.cpath = 'HOME/?.so;'..package.cpath
package.path = 'HOME/../?.lua;'..package.path

require('unix')

print("Starting DCM lua initialization");

require('player');

print("setting post process...");

postProcess = player.update;

