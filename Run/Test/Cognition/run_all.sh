LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 lua replay_logs.lua 1>/dev/null &
LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 luajit test_slam.lua
killall lua luajit
