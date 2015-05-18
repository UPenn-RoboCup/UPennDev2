#!/usr/local/bin/luajit -i
dofile'../../../include.lua'
local vector = require'vector'
local util   = require'util'
local matrix = require('matrix_zmp')
require'mcm'

--[[
local function save_param(self)

  local data=''
  data=data..string.format("-- tZmp: %.2f\n",self.tZmp)
  data=data..string.format("zmpstep.params = true;\n")
  data=data..string.format("zmpstep.param_k1_px={%f,%f,%f}\n",
   self.param_k1_px[1][1],self.param_k1_px[1][2],self.param_k1_px[1][3]);
  data=data..string.format("zmpstep.param_a={\n");
  for i=1,3 do
    data=data..string.format("  {%f,%f,%f},\n",
  self.param_a[i][1],self.param_a[i][2],self.param_a[i][3]);
  end
  data=data..string.format("}\n");
  data=data..string.format("zmpstep.param_b={%f,%f,%f,%f}\n",
    self.param_b[1][1],self.param_b[2][1],self.param_b[3][1],self.param_b[4][1]);
  data=data..string.format("zmpstep.param_k1={\n    ");

  for i=1,self.preview_steps do
    data=data..string.format("%f,",self.param_k1[1][i]);
    if i%10==0 then   data=data.."\n    " ;end
  end
  data=data..string.format("}\n");

end
--]]

local function precompute(tZmp,outfile)
  ------------------------------------
  --We only need following parameters
  -- param_k1_px : 1x3
  -- param_k1 : 1xnPreview 
  -- param_a : 3x3
  -- param_b : 4x1
  ------------------------------------

  local nPreview = preview_interval / preview_tStep  

  param_a=matrix {{1,timeStep,timeStep^2/2},{0,1,timeStep},{0,0,1}}
  param_b=matrix.transpose({{timeStep^3/6, timeStep^2/2, timeStep,timeStep}})  

  local px,pu0,pu = {}, {}, {}
  for i=1, nPreview do
    px[i]={1, i*timeStep, i*i*timeStep*timeStep/2 - tZmp*tZmp}
    pu0[i]=(1+3*(i-1)+3*(i-1)^2)/6 *timeStep^3 - timeStep*tZmp*tZmp
    pu[i]={}
    for j=1, nPreview do pu[i][j]=0 end
    for j0=1,i do
      j = i+1-j0
      pu[i][j]=pu0[i-j+1]
    end
  end
  local param_pu = matrix:new(pu)
  local param_px = matrix:new(px)
  local param_pu_trans = matrix.transpose(param_pu)
  local param_a=matrix {{1,timeStep,timeStep^2/2},{0,1,timeStep},{0,0,1}}
  local param_b=matrix.transpose({{timeStep^3/6, timeStep^2/2, timeStep,timeStep}})
  local param_eye = matrix:new(nPreview,"I")
  local param_k=-matrix.invert(
      (param_pu_trans * param_pu) + (r_q*param_eye)
      )* param_pu_trans
  local k1={};
  k1[1]={};
  for i=1,nPreview do k1[1][i]=param_k[1][i] end
  local param_k1 = matrix:new(k1)
  local param_k1_px = param_k1 * param_px

  param_k1_px = param_k1_px
  param_k1 = param_k1


  local data=''



  data=data..string.format("zmpparam[%.3f] = {}\n",tZmp)  
  data=data..string.format("zmpparam[%.3f].param_k1_px={%f,%f,%f}\n",
   tZmp, param_k1_px[1][1],param_k1_px[1][2],param_k1_px[1][3]);


--[[
  data=data..string.format("zmpparam[%.3f].param_a={\n",tZmp);
  for i=1,3 do
    data=data..string.format("  {%f,%f,%f},\n",
  param_a[i][1],param_a[i][2],param_a[i][3]);
  end
  data=data..string.format("}\n");
  data=data..string.format("zmpparam[%.3f].param_b={%f,%f,%f,%f}\n",
    tZmp, param_b[1][1],param_b[2][1],param_b[3][1],param_b[4][1]);
--]]


  data=data..string.format("zmpparam[%.3f].param_k1={\n    ",tZmp);
  for i=1,nPreview do
    data=data..string.format("%f,",param_k1[1][i]);
    if i%5==0 then   data=data.."\n    " ;end
  end
  data=data..string.format("}\n\n\n");
  outfile:write(data);
  outfile:flush();

  print("Zmp parma added for tZmp",tZmp)
end



timeStep = 0.010
preview_tStep = 0.010
r_q = 10^-6
preview_interval = 3.0

local outfile=assert(io.open(HOME.."/Config/zmpparam.lua","w")); 

outfile:write("module(..., package.seeall)\n\nzmpparam={}\n");
outfile:write(string.format("zmpparam.preview_interval = %f\n",preview_interval))  
outfile:write(string.format("zmpparam.preview_tStep = %f\n",preview_tStep))  
outfile:write(string.format("zmpparam.param_r_q = %f\n\n\n",r_q))  

precompute(0.27,outfile)
precompute(0.28,outfile)
precompute(0.29,outfile)
precompute(0.30,outfile)
precompute(0.31,outfile)
precompute(0.32,outfile)
precompute(0.33,outfile)
precompute(0.34,outfile)
precompute(0.345,outfile)
precompute(0.35,outfile)
precompute(0.36,outfile)
precompute(0.37,outfile)
outfile:close();
