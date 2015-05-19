local use_telekinesis = false
local telekinesis = {}
if use_telekinesis then require'tkcm' end
    -- Grab the box and move it around
    -- TODO: Check if null or so, since this needs a PRO license
    if use_telekinesis then
      -- Table (for placing objects on?)
      telekinesis.table = {}
      telekinesis.table.tag = webots.wb_supervisor_node_get_from_def("MY_TABLE")
      telekinesis.table.translation = 
      webots.wb_supervisor_node_get_field(telekinesis.table.tag, "translation")
      telekinesis.table.get_position = function(self)
      return tkcm.get_table_position(), webots.wb_supervisor_field_get_sf_vec3f(self.translation)
  end
  telekinesis.table.set_position = function( self, new_position )
  webots.wb_supervisor_field_set_sf_vec3f( self.translation, carray.double( new_position ) )
end
telekinesis.table.update = function(self)
local table_pos_shm, table_pos_wbt = telekinesis.table:get_position()
        -- Webots xyz is not the same as the Body xyz residing in shm
        local table_new_pos={table_pos_shm[2],table_pos_wbt[2],table_pos_shm[1]}
        telekinesis.table:set_position(table_new_pos)
    end
      -- Drill
      telekinesis.drill = {}
      telekinesis.drill.tag = webots.wb_supervisor_node_get_from_def("MY_DRILL")
      telekinesis.drill.translation = 
      webots.wb_supervisor_node_get_field(telekinesis.drill.tag, "translation")
      telekinesis.drill.rotation = 
      webots.wb_supervisor_node_get_field(telekinesis.drill.tag, "rotation")
      telekinesis.drill.get_position = function(self)
      local p_wbt = webots.wb_supervisor_field_get_sf_vec3f(self.translation)
        -- Webots x is our y, Webots y is our z, Webots z is our x, 
        -- Our x is Webots z, Our y is Webots x, Our z is Webots y
        return tkcm.get_drill_position(), vector.new({p_wbt[3],p_wbt[1],p_wbt[2]})
    end
    telekinesis.drill.set_position = function( self, new_position )
    assert(#new_position==3,'Bad new_position!')
        -- Webots x is our y, Webots y is our z, Webots z is our x, 
        -- Our x is Webots z, Our y is Webots x, Our z is Webots y
        local p_wbt = carray.double({
        	new_position[2],
        	new_position[3],
        	new_position[1]
        	})
        webots.wb_supervisor_field_set_sf_vec3f( self.translation, p_wbt )
    end
    telekinesis.drill.get_orientation = function(self)
    local q = quaternion.new(tkcm.get_drill_orientation())
    local aa_wbt = webots.wb_supervisor_field_get_sf_rotation(self.rotation)
        -- Webots x is our y, Webots y is our z, Webots z is our x, 
        -- Our x is Webots z, Our y is Webots x, Our z is Webots y
        local q_wbt = quaternion.from_angle_axis(aa_wbt[4],{aa_wbt[3],aa_wbt[1],aa_wbt[2]})
        return q, quaternion.unit(q_wbt)
    end
    telekinesis.drill.set_orientation = function( self, new_orientation )
    assert(#new_orientation==4,'Bad new_orientation!')
    local angle, axis = quaternion.angle_axis(new_orientation)
        -- Webots x is our y, Webots y is our z, Webots z is our x, 
        -- Our x is Webots z, Our y is Webots x, Our z is Webots y
        local q_wbt_new = carray.double({
        	axis[2],
        	axis[3],
        	axis[1],
        	angle
        	})        
        webots.wb_supervisor_field_set_sf_rotation( self.rotation, q_wbt_new )
    end
    telekinesis.drill.update = function(self)
    local q_drill_shm, q_drill_wbt = telekinesis.drill:get_orientation()
    local drill_pos_shm, drill_pos_wbt = telekinesis.drill:get_position()
    self:set_orientation(q_drill_shm)
    telekinesis.drill:set_position(drill_pos_shm)
end
      -- Associate the table with the body
      Body.telekinesis = telekinesis
  end

    -- Update the telekinesis
    if use_telekinesis then
    	telekinesis.table:update()
    	telekinesis.drill:update()
    end