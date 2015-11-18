# Set the listener properly
using ZMQ
using MAT

ctx=Context()
s1=Socket(ctx, REP)

ZMQ.bind(s1, "ipc:///tmp/armopt")
println("Waiting for a message")
msg = ZMQ.recv(s1)
#out=convert(IOStream, msg)
#seek(out,0)
#read out::MemIO as usual, eg. read(out,...) or takebuf_string(out)
#or, conveniently, use bytestring(msg) to retrieve a string

println("Received a message!")

matfile = bytestring(msg)
println("I received ", matfile)

file = matopen(matfile)
varnames = names(file)
println(varnames)
close(file)

# This works fine
vars = matread(matfile)
#println(vars)

# Original Path
qw0 = vars["qwPath"]
# Task Space target
vw0 = vars["vwPath"]
# Task Space effective
vw0 = vars["vwEffective"]


ZMQ.send(s1, Message(matfile))
ZMQ.close(s1)

ZMQ.close(ctx)


#
# using ECOS, SCS, Gurobi
# using Convex
#
# mysolver = ECOSSolver()
# #mysolver = SCSSolver()
# #mysolver = GurobiSolver()
#
# items = [:Gold, :Silver, :Bronze]
# values = [5.0, 3.0, 1.0]
# weights = [2.0, 1.5, 0.3]
#
# # Define a variable of size 3, each index representing an item
# x = Convex.Variable(3)
# p = maximize(x' * values, 0 <= x, x <= 1, x' * weights <= 3)
# @time Convex.solve!(p, mysolver)
#
# println("Optimal objective: ", p.optval)
# items = Dict{Symbol,Float64}(zip(items, x.value))
#
# # [:Gold 0.9999971880377178
# #  :Silver 0.46667637765641057
# #  :Bronze 0.9999998036351865]
