function err = dynamixelReset(id);

inst = 6;
[id_ret, err] = dynamixelCommand(id, inst);
