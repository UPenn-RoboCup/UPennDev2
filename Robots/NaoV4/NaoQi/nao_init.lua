package.cpath = 'HOME/?.so;'..package.cpath
package.path = 'HOME/../?.lua;'..package.path

local unix = require('unix')

print("Starting DCM lua initialization");

local player = require('player');

print("setting post process...");

postProcess = player.update;

