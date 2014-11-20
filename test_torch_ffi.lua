ffi.load'libTH'
ffi.cdef"void THDoubleTensor_addmv(THDoubleTensor *r_, double beta, THDoubleTensor *t, double alpha, THDoubleTensor *mat, THDoubleTensor *vec);"
C=ffi.C
mat = torch.eye(4)
vec = torch.Tensor{1,2,3,4}
beta = 0
alpha = 1
res = torch.Tensor(4)
C.THDoubleTensor_addmv(res:cdata(), beta, res:cdata(), alpha, mat:cdata(), vec:cdata())
print(res)
