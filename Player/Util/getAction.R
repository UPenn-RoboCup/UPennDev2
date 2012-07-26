### Global Parameter ### 
gamma = 0.5

rData.file = ".RData"
a.file = "action.data"

### Main ###
# Parse Argument
args = commandArgs(TRUE)
K = as.numeric(args[1])
t = as.numeric(args[2])
r = as.numeric(args[3])

# Load weights
if (t == 1) {
	w = rep(1,K)
} else {
	load(rData.file)
	xhat = rep(0,K)
	xhat[a] = r/p[a]
	w = w * exp(gamma * xhat/K)
}



# Compute p vectors
p = (1 - gamma) * w / (sum(w)) + gamma / K

# Make an action
a = sample(x=1:K,size=1,prob=p)
#cat("Weights:",p,"\n")
#cat("We choose action #",a,"\n")
cat(a)
write.table(a,file=a.file,row.names=FALSE,col.names=FALSE)

# Save for later computation
rm(r)
save.image(file=rData.file)
