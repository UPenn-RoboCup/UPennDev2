local Body = require('Body');
local motion = require('motion');

print("DCM motion starting...");
motion.entry();
postProcess = motion.update;
