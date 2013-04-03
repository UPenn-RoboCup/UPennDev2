-- Hokuyo Communication Manager

module(... or '', package.seeall)

require("Config")
require("vector")
require("util")
require("shm")

shared = {}
shsize = {}

shared.hokuyo = {};
shared.hokuyo.counter = vector.zeros(1);
shared.hokuyo.id = vector.zeros(1);

-- Default use HOKUYO_UTM with max Point 1081
shared.hokuyo.ranges = vector.zeros(1081);
--shared.hokuyo.intensities ;
shared.hokuyo.startAngle = vector.zeros(1); -- radians
shared.hokuyo.stopAngle = vector.zeros(1);  -- radians
shared.hokuyo.angleStep = vector.zeros(1);  -- radians
shared.hokuyo.startTime = vector.zeros(1);  -- seconds
shared.hokuyo.stopTime = vector.zeros(1);   -- seconds

util.init_shm_segment(getfenv(), _NAME, shared, shsize);
