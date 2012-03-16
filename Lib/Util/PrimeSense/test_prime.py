from openni import *

try:

    # Start OpenNI
    print "Initializing..."
    ctx = Context()
    ctx.init()
    user = UserGenerator()
    user.create(ctx)

    # Add some gestures to look for
#    print "Registering listeners..."
#    user.add_gesture("Wave")
#    user.add_gesture("Click")

    # Register callbacks
    def user_found(src, user, cookie):
        print "Found user:", user

    def user_lost(src, gesture, cookie): 
        print "Lost user:", user

    print "Registering callbacks"
    gest.register_gesture_cb(gesture_detected, gesture_progress)

    # Start generating
    print "Ready! Starting to detect gestures."
    ctx.start_generating_all()

    # Main loop:
    # process every frame until the user interrupts
    try:
        print "Press Control-C to quit.\n"
        while True: ctx.wait_any_update_all()
    except KeyboardInterrupt: print

except OpenNIError, error:
    # When an error occurs:
    print "OpenNI raised an error:", error
