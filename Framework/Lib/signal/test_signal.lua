local signal = require('signal')

function ShutDownFN()
	print("shutdown")
	os.exit(1);
end

signal.signal("SIGINT", ShutDownFN);
signal.signal("SIGTERM", ShutDownFN);

while (true) do
	print('annoying!')
end
