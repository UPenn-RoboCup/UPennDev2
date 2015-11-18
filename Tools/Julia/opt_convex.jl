
## Tuning
# Relative weight of acceleration (vs null space accuracy)
alpha = 10 * 1e2; # Accel
beta = 1e1; # Vel

# Out
#c_tight = 0*1e-3;
#c_usage = 2*1e-3;

# Tight
c_tight = 2*1e-3;
c_usage = 0*1e-3;

# Closeness to previous trajectory
epsilon = deg2rad(10);
# Constraint the joints to be close on how many iterations...
# More skips makes the formulation of the problem easier
# Only works with proper acceleration weight
nSkip = 0; # Default on all
#nSkip = 10; # One constraint per second
#nSkip = 3;
nSkip = max(floor(nSkip), 0) + 1;
# TODO: Truncate the path if possible, once the difference is small
# Then further optimization steps just make the path smaller

# Optimization tools
#using ECOS; mysolver = ECOSSolver()
#using SCS;mysolver = SCSSolver()
using Gurobi; mysolver = GurobiSolver()
using Convex

# Set the listener properly
using ZMQ
using MAT

ctx=Context()
s1=Socket(ctx, REP)

ZMQ.bind(s1, "ipc:///tmp/armopt")

while true

  println("Waiting for a message")
  msg = ZMQ.recv(s1)
  println("Received a message!")
  matfile = bytestring(msg);
  println("I received: ", matfile)

  file = matopen(matfile)
  varnames = names(file)
  println(varnames);
  close(file);

  # This works fine
  vars = matread(matfile);

  ## Convert the variables

  # Task Space target
  vw0 = vars["vwPath"]
  vwa = Array{Float64}(size(vw0, 1), size(vw0[1], 1))
  for iter in eachindex(vw0)
  #    @show qw0[iter]
  #    @show iter
      vwa[iter, :] = vw0[iter]
  end
  vw0 = vwa
  vwa = 0;
  # Original Path
  qw0 = vars["qwPath"]
  qw0a = Array{Float64}(size(qw0, 1), size(qw0[1], 1))
  for iter in eachindex(qw0)
  #    @show qw0[iter]
  #    @show iter
      qw0a[iter, :] = qw0[iter]
  end
  qwPath0 = qw0a
  qw0a = 0;qw0 = 0;

  # Task Space effective
  vw = vars["vwEffective"]
  vwa = Array{Float64}(size(vw, 1), size(vw[1], 1))
  for iter in eachindex(vw)
  #    @show qw0[iter]
  #    @show iter
      vwa[iter, :] = vw[iter]
  end
  vw = vwa
  vwa = 0;

  # Number of trajectory points
  np = size(qwPath0, 1)
  # Number of joints
  nq = size(qwPath0, 2)
  # Number of optimization points
  n = length(qwPath0)
  println(n,'=',np, 'x', nq)

  ## Velocity matrix
  v0 = ones(n-nq);
  v0[1:nq, :] = 2;
  v1 = zeros(n);
  v1[1:nq] = -2;
  v1[end-nq+1:end] = 2;
  V = diagm(v0, nq) + diagm(v1);
  V = V + diagm(-flipdim(v0, 1), -nq);
  V = 0.5*V;

  ## Acceleration matrix
  d2 = 2 * ones(n);
  # Proper doundary condition (Central difference 2nd order):
  # http://www.mathematik.uni-dortmund.de/~kuzmin/cfdintro/lecture4.pdf
  d2[1:nq] = 1;
  d2[end-nq+1:end] = 1;
  d1 = ones(n - nq);

  A0 = diagm(-d2);
  A1 = diagm(d1, nq);
  A = A0 + A1 + A1';
  A = sparse(A);
  ATA = A' * A;

  # Null Space effective
  nulls = vars["nulls"]
  nulls_sparse = Array{Any}(size(vw, 1),)
  for iter in eachindex(nulls)
      nulls_sparse[iter] = sparse(nulls[iter]);
  end

  nulls = 0;
  N = blkdiag(nulls_sparse...)';
  nulls_sparse = 0;
  NTN = N' * N;

  # Jacobian effective
  Js = vars["Js"]
  Js_sparse = Array{Any}(size(vw, 1),)
  for iter in eachindex(Js)
      Js_sparse[iter] = sparse(Js[iter]);
  end

  Js = 0;
  J = blkdiag(Js_sparse...)';
  Js_sparse = 0;
  JV = J * V;

  # Constant Constraints
  qMid = [0, 0.759218, 0, -1.39626, 0, 0, 0];
  qMid = repmat(qMid[:], np, 1);
  elbow_diag = ones(n);
  elbow_diag[3:7:n] = 1;
  N_elbow = diagm(elbow_diag) * N;
  NTN_elbow = N_elbow' * N_elbow;


  # Solve the problem
  q0 = reshape(qwPath0', n, 1)
  q = Convex.Variable(n)
  p = minimize(
  alpha * sumsquares(A * q)
  + beta * sumsquares(JV * q)
  + c_tight * sumsquares(N_elbow * q)
  #+ c_usage * quad_form(q - qMid, NTN)
  + c_usage * sumsquares(N * (q-qMid))
  )

  p.constraints += q[1:nq]==q0[1:nq]
  p.constraints += q[n-nq+1:n]==q0[n-nq+1:n]
  for k = nq+1 : nSkip*nq : n-nq
      p.constraints += Base.norm(q[k:k+nq-1] - q0[k:k+nq-1], 2) <= epsilon;
  end

  #@time Convex.solve!(p, mysolver);
  Convex.solve!(p, mysolver);
  #println("Optimal objective: ", p.optval);
  #println(q.value);

  q_opt = reshape(q.value, nq, np)';

  println("Solution size: ", size(q_opt));
  file = matopen(matfile, "w")
  write(file, "qw", q_opt)
  close(file)

  ZMQ.send(s1, Message(matfile))

end

# Cleanup
ZMQ.close(s1)
ZMQ.close(ctx)
