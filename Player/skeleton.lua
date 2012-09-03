module(... or "",package.seeall)

function entry( mylogs )

	if(mylogs) then
		logs = mylogs
		nPlayers = #logs
		timestamp0 = 0
		timestamp_last = timestamp0
		t_update = unix.time()
	else
		require 'primesense'
		nPlayers = 2
		timestamp0 = unix.time();
	end
	
	-- Require the primecm modules
	pc = {};
	for i=1,nPlayers do
		pc[i] = require('primecm'..i)
	end
	
end

function update()
	if(logs) then
		update_logs()
	else
		update_primesense()
	end
end

function update_logs()

	for pl=1,nPlayers do
		log = logs[pl];
		timestamp_last = timestamp
		timestamp = log[count].t;
		timestamp_diff = timestamp - timestamp_last
		pos = { log[count].x[i],log[count].y[i],log[count].z[i] };
		confidence = { log[count].posconf[i],log[count].rotconf[i] };
		primecm = pc[pl];
		primecm['set_position_'..v]( pos );
		primecm['set_confidence_'..v]( confidence );
		primecm.set_skeleton_found( 1 );
		primecm.set_skeleton_timestamp( timestamp-timestamp0 );
	end
	t_last_update = t_update;
	t_update = unix.time();
	t_update_diff = t_update - t_last_update;
	unix.usleep( math.max(0,timestamp_diff-t_update_diff) );

end

function update_primesense()
	local ret = primesense.update_joints();
	timestamp = unix.time();

	-- Grab torso positions
	center = {};
	for pl=1,nPlayers do
		local pos, rot, confidence, active = primesense.get_jointtables(pl,3);
		center[pl] = pos[1];
	end
	
	-- Grab raw positions
	for pl=1,nPlayers do
		-- Choose the right player's SHM
		primecm = pc[pl];
		for i,v in ipairs(pc[playerID].jointNames) do
			pos, rot, confidence, active = primesense.get_jointtables(pl,i);
			-- Convert Positions to meters
			pos = vector.new(pos)/1000;
			-- Set in memory
			primecm['set_position_'..v]( pos );
			primecm['set_orientation_'..v]( rot );
			primecm['set_confidence_'..v]( confidence );
			primecm.set_skeleton_found( active );
			primecm.set_skeleton_timestamp( timestamp-timestamp0 );
		end
	end
	
end

function exit()
	for pl=1,#pc do
		print('Disabling user '..pl)
		primecm = pc[pl];
		primecm.set_skeleton_found( 0 );  
	end
end