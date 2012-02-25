module(..., package.seeall);

require('Body')
require('keyframe')
require('walk')
require('vector')
require('Config')

local cwd = unix.getcwd();
if string.find(cwd, "WebotsController") then
	cwd = cwd.."/Player";
end
cwd = cwd.."/Motion/keyframes"

keyframe.load_motion_file(cwd.."/"..Config.km.walk_forward,
"walkForward");
keyframe.load_motion_file(cwd.."/"..Config.km.walk_backward,
"walkBackward");

-- default kick type
walkDir = "walkForward";

function entry()
	print(_NAME.." entry");
	walk.stop();
	print('Initializing walking keyframe...')
	keyframe.entry();
	keyframe.do_motion(walkDir);
	Body.set_body_hardness(0.8);
end

function update()

	keyframe.update();
	if (keyframe.get_queue_len() == 0) then
		return "done"
	end
end

function exit()
	print("Walking exit");
	keyframe.exit();
	walk.active=true;
end

function set_walk_dir(wd)
	-- set the swing foot (left/right)
	walkDir = wd;
end

