using ECOS, SCS, Gurobi
using JuMP

mysolver = ECOSSolver()
#mysolver = SCSSolver()
#mysolver = GurobiSolver()

items  = [:Gold, :Silver, :Bronze]
values = Dict(:Gold => 5.0,  :Silver => 3.0,  :Bronze => 1.0)
weight = Dict(:Gold => 2.0,  :Silver => 1.5,  :Bronze => 0.3)

m = Model(solver=mysolver)
@defVar(m, 0 <= take_var[items] <= 1)  # Define a variable for each item
@setObjective(m, Max, sum{ values[item] * take_var[item], item in items})
@addConstraint(m, sum{ weight[item] * take_var[item], item in items} <= 3)
@time JuMP.solve(m)

println("Optimal objective: ",getObjectiveValue(m))
println("Optimal vars: ",getValue(take_var))

# take
# [  Gold] = 0.9999999680446406
# [Silver] = 0.46666670881026834
# [Bronze] = 0.9999999633898735
