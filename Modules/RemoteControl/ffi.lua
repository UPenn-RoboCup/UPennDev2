local libRC = {}
local udp = require'udp'
local RC_TIMEOUT = 1 / 50
local ffi = require'ffi'
local Body = require'Body'

local RC_constants = [[
#define REMOTE_CONTROL_PORT 54321
#define REMOTE_CONTROL_ROBOT_ID 0x0A
#define REMOTE_CONTROL_REMOTE_ID 0x0B
]]

local RC_structs = [[
typedef struct ThorUdpPacket
{
	char header[8];

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

local function send_feedback(self)
  local RC_data = self.feedback_data
  -- Head
  local head = Body.get_head_position()
  RC_data.neckZ, RC_data.neckY = unpack(head)

  -- Left Arm
  local larm = Body.get_larm_position()
  RC_data.leftShoulderY, RC_data.leftShoulderX, RC_data.leftShoulderZ,
  RC_data.leftElbowY,
  RC_data.leftElbowZ, RC_data.leftWristX, RC_data.leftWristZ = unpack(larm)
  
  -- Left Leg
  local lleg = Body.get_lleg_position()
  RC_data.leftHipZ, RC_data.leftHipX, RC_data.leftHipY, RC_data.leftKnee, RC_data.leftAnkleY, RC_data.leftAnkleX = unpack(lleg)
  
  -- Right Arm
  local rarm = Body.get_rarm_position()
  RC_data.rightShoulderY, RC_data.rightShoulderX, RC_data.rightShoulderZ,
  RC_data.rightElbowY,
  RC_data.rightElbowZ, RC_data.rightWristX, RC_data.rightWristZ = unpack(rarm)
  
  -- Right Leg
  local rleg = Body.get_rleg_position()
  RC_data.rightHipZ, RC_data.rightHipX, RC_data.rightHipY, RC_data.rightKnee, RC_data.rightAnkleY, RC_data.rightAnkleX = unpack(rleg)
  
  -- Angle
  local rpy = Body.get_rpy()
  RC_data.angleX, RC_data.angleY, RC_data.angleZ = unpack(rpy)
  
  -- Gyro
  local gyro = Body.get_gyro()
  RC_data.gyroX, RC_data.gyroY, RC_data.gyroZ = unpack(gyro)
  
  -- Accelerometer
  local rpy = Body.get_accelerometer()
  RC_data.accX, RC_data.accY, RC_data.accZ = unpack(rpy)
  
  self.send_ok = self.sender:send(ffi.string(RC_data, ffi.sizeof(RC_data))) > 0
  --print('SEND', self.send_ok)
  return self
end

local function wait_for_commands(self)
  self.ready = unix.select({self.recv_fd}, RC_TIMEOUT) > 0
  --print('READY', self.ready)
  return self
end

local function receive_commands(self)
  if not self.ready then return self end
  local buf = self.receiver:receive()
  if not buf then
    self.cmds = nil
    return self
  end
  local sz = #buf
  if sz ~= ffi.sizeof'struct ThorUdpPacket' then
    self.cmds = nil
    return self
  end
  self.cmds = ffi.new('struct ThorUdpPacket')
  ffi.copy(self.cmds, buf, sz)
  return self
end

local function process_commands(self)
  local cmds = self.cmds
  if not cmds then return self end
  if cmds.header[0] ~= libRC.REMOTE_CONTROL_REMOTE_ID then return self end
  self.n = self.n + 1
  
  Body.set_head_command_position{cmds.neckZ, cmds.neckY}
  Body.set_larm_command_position{
    cmds.leftShoulderY, cmds.leftShoulderX, cmds.leftShoulderZ,
    cmds.leftElbowY,
    cmds.leftElbowZ, cmds.leftWristX, cmds.leftWristZ
  }
  Body.set_lleg_command_position{
    cmds.leftHipZ, cmds.leftHipX, cmds.leftHipY, cmds.leftKnee, cmds.leftAnkleY, cmds.leftAnkleX
  }
  Body.set_rarm_command_position{
    cmds.rightShoulderY, cmds.rightShoulderX, cmds.rightShoulderZ,
    cmds.rightElbowY,
    cmds.rightElbowZ, cmds.rightWristX, cmds.rightWristZ
  }
  Body.set_rleg_command_position{
    cmds.rightHipZ, cmds.rightHipX, cmds.rightHipY, cmds.rightKnee, cmds.rightAnkleY, cmds.rightAnkleX
  }
  return self
end

local function rc_tostring(self, formatstr)
  return string.format('Remote Controller | %s', self.address)
end

local __mt = {__tostring = rc_tostring}

function libRC.init(RC_addr)
  local address = RC_addr or 'localhost'
  local sender = udp.new_sender(address, libRC.REMOTE_CONTROL_PORT)
  local receiver = udp.new_receiver(libRC.REMOTE_CONTROL_PORT)
  local RC_data = ffi.new('struct ThorUdpPacket')
  RC_data.header[0] = libRC.REMOTE_CONTROL_ROBOT_ID
  return setmetatable({
    send = send_feedback,
    wait = wait_for_commands,
    receive = receive_commands,
    process = process_commands,
    receiver = receiver,
    sender = sender,
    recv_fd = receiver.fd or receiver:descriptor(),
    feedback_data = RC_data,
    address = address,
    n = 0
  },
  __mt
  )
end

return libRC
