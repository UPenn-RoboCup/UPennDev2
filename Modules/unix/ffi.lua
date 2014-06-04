local unix = {}

ffi.cdef[[

typedef __darwin_time_t   time_t;

typedef struct {
  time_t       tv_sec;   -- seconds since Jan. 1, 1970
  suseconds_t  tv_usec;  -- and microseconds
} timeval;

int usleep(useconds_t useconds);
int gettimeofday(struct timeval *restrict tp, void *restrict tzp);

]]

function unix.usleep(usec)
  return usleep(usec)
end

local tval = ffi.new("timeval")
function unix.time()
  gettimeofday(tval, nil)
end

return unix
