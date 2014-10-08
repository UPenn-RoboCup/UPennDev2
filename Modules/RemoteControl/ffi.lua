local libRC = {}
local si = require'simple_ipc'
local RC_TIMEOUT = 1 / 50
local ffi = require'ffi'
local Body = require'Body'

local RC_constants = [[
#define REMOTE_CONTROL_PORT 10101
#define REMOTE_CONTROL_STRUCT_ID 0x0A
]]

local RC_structs = [[
typedef struct ThorUdpPacket
{
	unsigned char id;

	double neckZ;
	double neckY;

	double leftShoulderY;
	double leftShoulderX;
	double leftShoulderZ;
	double leftElbowY;
	double leftElbowZ;
	double leftWristX;
	double leftWristZ;
	double leftHipZ;
	double leftHipX;
	double leftHipY;
	double leftKnee;
	double leftAnkleY;
	double leftAnkleX;

	double rightShoulderY;
	double rightShoulderX;
	double rightShoulderZ;
	double rightElbowY;
	double rightElbowZ;
	double rightWristX;
	double rightWristZ;
	double rightHipZ;
	double rightHipX;
	double rightHipY;
	double rightKnee;
	double rightAnkleY;
	double rightAnkleX;

	double angleX;
	double angleY;
	double angleZ;
	double gyroX;
	double gyroY;
	double gyroZ;
	double accX;
	double accY;
	double accZ;
} ThorUdpPacket;

]]

local function define_to_vars(define, tbl)
  local define_iterator = "#define (%S+)%s+(%S+)"
  local defines = tbl or {}
  for name, value in define:gmatch(define_iterator) do
    defines[name] = tonumber(value) or value:match('"(%S+)"')
  end
  return defines
end

local function def(structs, defines, tbl)
  local constants
  if defines then
    constants = define_to_vars(defines, tbl)
    for k, v in pairs(constants) do
      structs = structs:gsub(k, v)
    end
  end
  ffi.cdef(structs)
  return constants
end

def(RC_structs, RC_constants, libRC)
local RC_data = ffi.new('struct ThorUdpPacket')
RC_data.id = libRC.REMOTE_CONTROL_STRUCT_ID

local function send_feedback(self)
  -- Left Arm
  local larm = Body.get_larm_position()
  RC_data.leftShoulderY, RC_data.leftShoulderX, RC_data.leftShoulderZ,
  RC_data.leftElbowY,
  RC_data.leftElbowZ, RC_data.leftWristX, RC_data.leftWristZ = unpack(larm)
  
  -- Left Leg
  local lleg = Body.get_lleg_position()
  RC_data.leftHipZ, RC_data.leftHipZ, RC_data.leftHipX,
  RC_data.leftHipY, RC_data.leftKnee, RC_data.leftAnkleY = unpack(lleg)
  
  -- Right Arm
  local rarm = Body.get_rarm_position()
  RC_data.rightShoulderY, RC_data.rightShoulderX, RC_data.rightShoulderZ,
  RC_data.rightElbowY,
  RC_data.rightElbowZ, RC_data.rightWristX, RC_data.rightWristZ = unpack(rarm)
  
  -- Right Leg
  local rleg = Body.get_rleg_position()
  RC_data.rightHipZ, RC_data.rightHipZ, RC_data.rightHipX,
  RC_data.rightHipY, RC_data.rightKnee, RC_data.rightAnkleY = unpack(rleg)
  
  -- Angle
  local rpy = Body.get_rpy()
  RC_data.angleX, RC_data.angleY, RC_data.angleZ = unpack(rpy)
  
  -- Gyro
  local gyro = Body.get_gyro()
  RC_data.gyroX, RC_data.gyroY, RC_data.gyroZ = unpack(gyro)
  
  -- Accelerometer
  local rpy = Body.get_accelerometer()
  RC_data.accX, RC_data.accY, RC_data.accZ = unpack(rpy)
  
  return self.sender:send(ffi.string(RC_data, ffi.sizeof(RC_data)))
end

local RC_cmd = ffi.new('struct ThorUdpPacket')
local function receive_commands(self, buf)
  if not buf then return end
  local sz = #buf
  if sz ~= ffi.sizeof(RC_cmd) then return end
  if RC_data.id ~= libRC.REMOTE_CONTROL_STRUCT_ID then return end
  ffi.copy(RC_cmd, buf, sz)
  return RC_cmd
end

local function wait_for_commands(self)
  local fd = self.receiver:descriptor()
  local status, ready = unix.select({fd}, RC_TIMEOUT)
end

function libRC.init(team, player, RC_addr)
  local udp_sender = si.new_sender(RC_addr or 'localhost', libRC.REMOTE_CONTROL_PORT)
  local udp_receiver = si.new_receiver(libRC.REMOTE_CONTROL_PORT)
  return {
    send = send_feedback,
    wait = wait_for_commands,
    receive = receive_commands,
    receiver = udp_receiver,
    sender = udp_sender,
  }
end

return libRC
