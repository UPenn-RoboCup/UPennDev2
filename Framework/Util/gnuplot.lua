--[[
Copyright © 2011-2012 Idiap Research Institute (Ronan Collobert)

Copyright © 2011-2012 NEC Laboratories America (Koray Kavukcuoglu)

Copyright © 2011-2012 NYU (Clement Farabet)

Copyright © 2006-2010 NEC Laboratories America (Ronan Collobert, Leon Bottou, Iain Melvin, Jason Weston)

Copyright © 2006 Idiap Research Institute (Samy Bengio)

Copyright © 2001-2004 Idiap Research Institute (Ronan Collobert, Samy Bengio, Johnny Mariethoz)

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    Neither the names of Idiap Research Institute, NEC Laboratories American and New York University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
--]]

---------------------------------------------------------------------------
-- gnuplot : modified plotting module from Torch7 (torch.ch)
---------------------------------------------------------------------------

gnuplot = {}

local _gptable = {}
_gptable.current = nil
_gptable.defaultterm = nil
_gptable.exe = nil
_gptable.hasrefresh = true

local function getexec()
   if not _gptable.exe then
      error('gnuplot executable is not set')
   end
   return _gptable.exe
end

local function findos()
   local ff = io.popen('uname -a','r')
   local s = ff:read('*all')
   ff:close()
   if s:match('Darwin') then
      return 'mac'
   elseif s:match('Linux') then
      return 'linux'
   elseif s:match('FreeBSD') then
      return 'freebsd'
   else
      --error('I don\'t know your operating system')
      return '?'
   end
end

local function getfigure(n)
   local n = n
   if not n or n == nil then
      n = #_gptable+1
   end
   if _gptable[n] == nil then
      _gptable[n] = {}
      if _gptable.defaultterm == nil then
         error('Gnuplot terminal is not set')
      end
      _gptable[n].term = _gptable.defaultterm
      _gptable[n].pipe = io.popen(getexec() .. ' -persist > /dev/null 2>&1 ','w')
   end
   _gptable.current = n
   return _gptable[n]
end

local function gnuplothasterm(term)
   if not _gptable.exe then
      return false--error('gnuplot exe is not found, can not chcek terminal')
   end
   local tfni = os.tmpname()
   local tfno = os.tmpname()
   local fi = io.open(tfni,'w')
   fi:write('set terminal\n\n')
   fi:close()
   os.execute(getexec() .. ' < ' .. tfni .. ' > ' .. tfno .. ' 2>&1 ')
   os.remove(tfni)
   local tf = io.open(tfno,'r')
   local s = tf:read('*l')
   while s do
      if s:match('^.*%s+  '.. term .. ' ') then
	 tf:close()
	 os.remove(tfno)
         return true
      end
      s = tf:read('*l')
   end
   tf:close()
   os.remove(tfno)
   return false
end

local function findgnuplotversion(exe)
   local ff = io.popen(exe .. '  --version','r')
   local ss = ff:read('*l')
   ff:close()
   local v,vv = ss:match('(%d).(%d)')
   v=tonumber(v)
   vv=tonumber(vv)
   return v,vv
end

local function findgnuplotexe()
   local os = findos()
   if os == 'windows' then
      return 'gnuplot.exe' -- I don't know how to find executables in Windows
   else
      _gptable.hasrefresh = true
      local ff = io.popen('which gnuplot','r')
      local s=ff:read('*l')
      ff:close()
      if s and s:len() > 0 and s:match('gnuplot') then
	 local v,vv = findgnuplotversion(s)
	 if  v < 4 then
	    error('gnuplot version 4 is required')
	 end
	 if vv < 4 then
	    -- try to find gnuplot44
	    if os == 'linux' and os.execute('test -e /usr/bin/gnuplot44') == 0 then
	       local ss = '/usr/bin/gnuplot44'
	       v,vv = findgnuplotversion(ss)
	       if v == 4 and vv == 4 then
		  return ss
	       end
	    end
	    _gptable.hasrefresh = false
	    print('Some functionality like adding title, labels, ... will be disabled install gnuplot version 4.4')
	 end
	 return s
      else
	 return nil
      end
   end
end

local function getgnuplotdefaultterm(os)
   if os == 'windows' and gnuplothasterm('windows') then
      return  'windows'
   elseif os == 'linux' and gnuplothasterm('wxt') then
      return  'wxt'
   elseif os == 'linux' and gnuplothasterm('x11') then
      return  'x11'
   elseif os == 'freebsd' and gnuplothasterm('wxt') then
      return  'wxt'
   elseif os == 'freebsd' and gnuplothasterm('x11') then
      return  'x11'
   elseif os == 'mac' and gnuplothasterm('aqua') then
      return  'aqua'
   elseif os == 'mac' and gnuplothasterm('x11') then
      return  'x11'
   else
      print('Can not find any of the default terminals for ' .. os .. ' you can manually set terminal by gnuplot.setterm("terminal-name")')
      return nil
   end
end

local function findgnuplot()
   local exe = findgnuplotexe()
   local os = findos()
   if not exe then
      return nil--error('I could not find gnuplot exe')
   end
   _gptable.exe = exe
   _gptable.defaultterm = getgnuplotdefaultterm(os)
end


function gnuplot.setgnuplotexe(exe)
   local oldexe = _gptable.exe

   if not os.execute('test -e '..exe) == 0 then
      error(exe .. ' does not exist')
   end

   _gptable.exe = exe
   local v,vv = findgnuplotversion(exe)
   if v < 4 then error('gnuplot version 4 is required') end
   if vv < 4 then 
      _gptable.hasrefresh = false
      print('Some functionality like adding title, labels, ... will be disabled, it is better to install gnuplot version 4.4')
   else
      _gptable.hasrefresh = true
   end
   
   local os = findos()
   local term = getgnuplotdefaultterm(os)
   if term == nil then
      print('You have manually set the gnuplot exe and I can not find default terminals, run gnuplot.setterm("terminal-name") to set term type')
   end
end

function gnuplot.setterm(term)
   if gnuplothasterm(term) then
      _gptable.defaultterm = term
   else
      error('gnuplot does not seem to have this term')
   end
end

local function getCurrentPlot()
   if _gptable.current == nil then
      gnuplot.figure()
   end
   return _gptable[_gptable.current]
end

local function writeToPlot(gp,str)
   local pipe = gp.pipe
   pipe:write(str .. '\n\n\n')
   pipe:flush()
   --pipe:writeString(str .. '\n\n\n')
   --pipe:synchronize()
end
local function refreshPlot(gp)
   if gp.fname then
      writeToPlot(gp,'set output "' .. gp.fname .. '"')
   end
   writeToPlot(gp,'refresh')
   if gp.fname then
      writeToPlot(gp,'unset output')
   end
end
local function writeToCurrent(str)
   writeToPlot(getCurrentPlot(),str)
end
local function refreshCurrent()
   refreshPlot(getCurrentPlot())
end

-- t is the arguments for one plot at a time
local function getvars(t)
   local legend = nil
   local x = nil
   local y = nil
   local format = nil

   local function isvector(v)
      return (type(v) == 'table' and type(v[1]) == 'number')
          or (type(v) == 'userdata' and type(v[1]) == 'number')
   end

   local function isstring(v)
      return type(v) == 'string'
   end

   if #t == 0 then
      error('empty argument list')
   end

   if #t >= 1 then
      if isstring(t[1]) then
	 legend = t[1]
      elseif isvector(t[1]) then
	 x = t[1]
      else
         print(type(v))
	 error('expecting [string,] vector [,vector] [,string]')
      end
   end
   if #t >= 2 then
      if x and isstring(t[2]) then
	 format = t[2]
      elseif x and isvector(t[2]) then
	 y = t[2]
      elseif legend and isvector(t[2]) then
	 x = t[2]
      else
	 error('expecting [string,] vector [,vector] [,string]')
      end
   end
   if #t >= 3 then
      if legend and x and isvector(t[3]) then
	 y = t[3]
      elseif legend and x and isstring(t[3]) then
	 format = t[3]
      elseif x and y and isstring(t[3]) then
	 format = t[3]
      else
	 error('expecting [string,] vector [,vector] [,string]')
      end
   end
   if #t == 4 then
      if legend and x and y and isstring(t[4]) then
	 format = t[4]
      else
	 error('expecting [string,] vector [,vector] [,string]')
      end
   end
   legend = legend or ''
   format = format or ''
   if not x then
      error('expecting [string,] vector [,vector] [,string]')
   end
   if not y then
      y = x
      x = {}
      for i = 1,#y do 
        x[i] = i
      end
   end
   if #x == 0 or #y == 0 then
      error('x and y are expected to be vectors #x = ' .. #x .. 'D #y = ' .. #y .. 'D')
   end
   return legend,x,y,format
end

-- t is the arguments for one plot at a time
local function getsplotvars(t)
   local legend = nil
   local x = nil
   local y = nil
   local z = nil
   local format = nil

   local function ismatrix(v)
      return (type(v) == 'table' and type(v[1]) == 'table' and type(v[1][1]) == 'number')
          or (type(v) == 'userdata' and type(v[1]) == 'userdata' and type(v[1][1]) == 'number')
   end

   local function isstring(v)
      return type(v) == 'string'
   end

   if #t == 0 then
      error('empty argument list')
   end

   if #t >= 1 then
      if isstring(t[1]) then
	 legend = t[1]
      elseif ismatrix(t[1]) then
	 x = t[1]
      else
	 error('expecting [string,] matrix [,matrix] [,matrix] [,string]')
      end
   end
   if #t >= 2 and #t <= 5 then
      if x and isstring(t[2]) then
        format = t[2]
      elseif x and ismatrix(t[2]) and ismatrix(t[3]) then
	 y = t[2]
	 z = t[3]
      elseif legend and ismatrix(t[2]) and ismatrix(t[3]) and ismatrix(t[4]) then
	 x = t[2]
	 y = t[3]
	 z = t[4]
      elseif legend and ismatrix(t[2]) then
	 x = t[2]
      else
	 error('expecting [string,] matrix [,matrix] [,matrix] [,string]')
      end
      if isstring(t[#t]) then
        format = t[#t]
      end
   elseif #t > 5 then
      error('expecting [string,] matrix [,matrix] [,matrix] [,string]')
   end
   legend = legend or ''
   format = format or '-'
   if not x then
      error('expecting [string,] matrix [,matrix] [,matrix] [,string]')
   end
   if not z then
      z = x
      x = {} 
      y = {} 
      for i=1,#z do
        x[i] = {}
        y[i] = {}
        for j=1,#z[1] do
          x[i][j] = i 
          y[i][j] = j
        end
      end
   end
   if #x[1] == 0 or #y[1] == 0 or #z[1] == 0 then
      error('x and y and z are expected to be matrices' )
   end
   return legend,x,y,z,format
end

-- t is the arguments for one plot at a time
local function getplot3dvars(t)
   local legend = nil
   local x = nil
   local y = nil
   local z = nil
   local palette = nil

   local function ismatrix(v)
      return (type(v) == 'table' and type(v[1]) == 'table' and type(v[1][1]) == 'number')
          or (type(v) == 'userdata' and type(v[1]) == 'userdata' and type(v[1][1]) == 'number')
   end

   local function isstring(v)
      return type(v) == 'string'
   end

   if #t == 0 then
      error('empty argument list')
   end

   if #t >= 1 then
      if isstring(t[1]) then
	 legend = t[1]
      elseif ismatrix(t[1]) then
	 x = t[1]
      else
	 error('expecting [string,] matrix [,matrix] [,matrix] [,string]')
      end
   end
   if #t >= 2 and #t <= 5 then
      if x and isstring(t[2]) then
        palette = t[2]
      elseif x and ismatrix(t[2]) and ismatrix(t[3]) then
	 y = t[2]
	 z = t[3]
      elseif legend and ismatrix(t[2]) and ismatrix(t[3]) and ismatrix(t[4]) then
	 x = t[2]
	 y = t[3]
	 z = t[4]
      elseif legend and ismatrix(t[2]) then
	 x = t[2]
      else
	 error('expecting [string,] matrix [,matrix] [,matrix] [,string]')
      end
      if isstring(t[#t]) then
        palette = t[#t]
      end
   elseif #t > 5 then
      error('expecting [string,] matrix [,matrix] [,matrix] [,string]')
   end
   legend = legend or ''
   palette = palette or 'gray'
   if not x then
      error('expecting [string,] matrix [,matrix] [,matrix] [,string]')
   end
   if not z then
      z = x
      x = {} 
      y = {} 
      for i=1,#z do
        x[i] = {}
        y[i] = {}
        for j=1,#z[1] do
          x[i][j] = i 
          y[i][j] = j
        end
      end
   end
   if #x[1] == 0 or #y[1] == 0 or #z[1] == 0 then
      error('x and y and z are expected to be matrices' )
   end
   return legend,x,y,z,palette
end

local function getimagescvars(t)
   local palette  = nil
   local x = nil

   local function ismatrix(v)
      return (type(v) == 'table' and type(v[1]) == 'table' and type(v[1][1]) == 'number')
          or (type(v) == 'userdata' and type(v[1]) == 'userdata' and type(v[1][1]) == 'number')
   end

   local function isstring(v)
      return type(v) == 'string'
   end

   if #t == 0 then
      error('empty argument list')
   end

   if #t >= 1 then
      if ismatrix(t[1]) then
	 x = t[1]
      else
	 error('expecting matrix [,string]')
      end
   end
   if #t == 2 then
      if x and isstring(t[2]) then
	 palette = t[2]
      else
	 error('expecting matrix [,string]' )
      end
   elseif #t > 2 then
      error('expecting matrix [,string]')
   end
   legend = legend or ''
   if not x then
      error('expecting matrix [,string]')
   end
   if not palette then
      palette = 'gray'
   end
   if #x[1] == 0 then
      error('x is expected to be a matrix')
   end
   return x,palette
end

local function gformat(f)
   if f ~= '~' and f:find('~') or f:find('acsplines') then
      coef = f:gsub('~',''):gsub('acsplines','')
      coef = tonumber(coef)
      f = 'acsplines'
   end
   if f == ''  or f == '' then return ''
   elseif f == '+'  or f == 'points' then return 'with points'
   elseif f == '.' or f == 'dots' then return 'with dots'
   elseif f == '-' or f == 'lines' then return 'with lines'
   elseif f == '+-' or f == 'linespoints' then return 'with linespoints' 
   elseif f == '|' or f == 'boxes' then return 'with boxes'
   elseif f == '~' or f == 'csplines' then return 'smooth csplines'
   elseif f == 'acsplines' then return 'smooth acsplines'
   end
   error("format string accepted: '.' or '-' or '+' or '+-' or '~' or '~ COEF'")
end

local function gnuplot_string(legend,x,y,format)
   local hstr = 'plot '
   local dstr = {''}
   local coef
   for i=1,#legend do
      if i > 1 then hstr = hstr .. ' , ' end
      hstr = hstr .. " '-' title '" .. legend[i] .. "' " .. gformat(format[i])
   end
   hstr = hstr .. '\n'
   for i=1,#legend do
      local xi = x[i]
      local yi = y[i]
      for j=1,#xi do
         if coef then
            table.insert(dstr,string.format('%g %g %g\n',xi[j],yi[j],coef))
         else
            table.insert(dstr,string.format('%g %g\n',xi[j],yi[j]))
         end
      end
      table.insert(dstr,'e\n')
   end
   return hstr,table.concat(dstr)
end

local function gnu_splot_string(legend,x,y,z,format)
   local hstr = 'splot '
   local dstr = {''}
   local coef
   for i=1,#legend do
      if i > 1 then hstr = hstr .. ' , ' end
      hstr = hstr .. " '-'title '" .. legend[i] .. "' " .. gformat(format[i])
   end
   hstr = hstr .. '\n'
   for i=1,#legend do
      local xi = x[i]
      local yi = y[i]
      local zi = z[i]
      for j=1,#xi do
         local xij = xi[j]
         local yij = yi[j]
         local zij = zi[j]
	 for k=1,#xi[1] do
            table.insert(dstr, string.format('%g %g %g\n',xij[k],yij[k],zij[k]))
         end
	 table.insert(dstr,'\n')
      end
      table.insert(dstr,'e\n')
   end
   return hstr,table.concat(dstr)
end

local function gnu_plot3d_string(legend,x,y,z,palette)
   local hstr = 'set pm3d\n unset surf\n'
   hstr = hstr..string.format('%s %s\n','set palette',palette[1])
   hstr = hstr..'splot '
   local dstr = {''}
   local coef
   for i=1,#legend do
      if i > 1 then hstr = hstr .. ' , ' end
      hstr = hstr .. " '-'title '" .. legend[i] .. "' "
   end
   hstr = hstr .. '\n'
   for i=1,#legend do
      local xi = x[i]
      local yi = y[i]
      local zi = z[i]
      for j=1,#xi do
         local xij = xi[j]
         local yij = yi[j]
         local zij = zi[j]
	 for k=1,#xi[1] do
            table.insert(dstr, string.format('%g %g %g\n',xij[k],yij[k],zij[k]))
         end
	 table.insert(dstr,'\n')
      end
      table.insert(dstr,'e\n')
   end
   return hstr,table.concat(dstr)
end

local function gnu_imagesc_string(x,palette)
   local hstr = string.format('%s\n','set view map')
   hstr = string.format('%s%s %s\n',hstr,'set palette',palette)
   hstr = string.format('%s%s\n',hstr,'set style data linespoints')
   hstr = string.format('%s%s%g%s\n',hstr,"set xrange [ -0.5 : ",#x[1]-0.5,"] noreverse nowriteback")
   hstr = string.format('%s%s%g%s\n',hstr,"set yrange [ -0.5 : ",#x-0.5,"] reverse nowriteback")
   hstr = string.format('%s%s\n',hstr,"splot '-' matrix with image")
   local dstr = {''}
   for i=1,#x do
      local xi = x[i];
      for j=1,#x[1] do
	 table.insert(dstr,string.format('%g ',xi[j]))
      end
      table.insert(dstr, string.format('\n'))
   end
   table.insert(dstr,string.format('e\ne\n'))
   return hstr,table.concat(dstr)
end

function gnuplot.close(n)
   if not n then return end
   local gp = _gptable[n]
   if gp == nil then return end
   if type(n) ==  number and gp.pipe then
      _gptable.current = n
      gnuplot.plotflush(i)
      writeToPlot(gp, 'quit')
      gp.pipe:close()
      gp.pipe=nil
      gp = nil
   end
   collectgarbage()
end

function gnuplot.closeall()
   for i,v in pairs(_gptable) do
      gnuplot.close(i)
   end
   _gptable = {}
   collectgarbage()
   findgnuplot()
   _gptable.current = nil
end

local function filefigure(fname,term,n)
   if not _gptable.hasrefresh then
      print('Plotting to files is disabled in gnuplot 4.2, install gnuplot 4.4')
   end
   local gp = getfigure(n)
   gp.fname = fname
   gp.term = term
   writeToCurrent('set term '.. gp.term)
   --writeToCurrent('set output \'' .. gp.fname .. '\'')
end
function gnuplot.epsfigure(fname,n)
   filefigure(fname,'postscript eps enhanced color',n)
end

function gnuplot.pngfigure(fname,n)
   filefigure(fname,'png',n)
end

function gnuplot.figprint(fname)
   local suffix = fname:match('.+%.(.+)')
   local term = nil
   if suffix == 'eps' then
      term = 'postscript eps enhanced color'
   elseif suffix == 'png' then
      term = 'png'
   else
      error('only eps and png for figprint')
   end
   writeToCurrent('set term ' .. term)
   writeToCurrent('set output \''.. fname .. '\'')
   refreshCurrent()
   writeToCurrent('unset output')
   writeToCurrent('set term ' .. _gptable[_gptable.current].term .. ' ' .. _gptable.current .. '\n')
end

function gnuplot.figure(n)
   local gp = getfigure(n)
   writeToCurrent('set term ' .. _gptable[_gptable.current].term .. ' ' .. _gptable.current .. '\n')
   writeToCurrent('raise')
   return _gptable.current
end

function gnuplot.plotflush(n)
   if not n then
      n = _gptable.current
   end
   if not n or _gptable[n] == nil then
      print('no figure ' ..  tostring(n))
      return
   end
   local gp = _gptable[n]
   --xprint(gp)
   if gp.fname then
      writeToPlot(gp,'set output "' .. gp.fname .. '"')
      writeToPlot(gp,'refresh')
      writeToPlot(gp,'unset output')
   end
end

local function gnulplot(legend,x,y,format)
   local hdr,data = gnuplot_string(legend,x,y,format)
   writeToCurrent(hdr)
   writeToCurrent(data)
end
local function gnusplot(legend,x,y,z,format)
   local hdr,data = gnu_splot_string(legend,x,y,z,format)
   writeToCurrent(hdr)
   writeToCurrent(data)
end
local function gnuplot3d(legend,x,y,z,palette)
   local hdr,data = gnu_plot3d_string(legend,x,y,z,palette)
   writeToCurrent(hdr)
   writeToCurrent(data)
end
local function gnuimagesc(x,palette)
   local hdr,data = gnu_imagesc_string(x,palette)
   writeToCurrent(hdr)
   writeToCurrent(data)
end

function gnuplot.xlabel(label)
   if not _gptable.hasrefresh then
      print('gnuplot.xlabel disabled')
      return
   end
   writeToCurrent('set xlabel "' .. label .. '"')
   refreshCurrent()
end
function gnuplot.ylabel(label)
   if not _gptable.hasrefresh then
      print('gnuplot.ylabel disabled')
      return
   end
   writeToCurrent('set ylabel "' .. label .. '"')
   refreshCurrent()
end
function gnuplot.zlabel(label)
   if not _gptable.hasrefresh then
      print('gnuplot.zlabel disabled')
      return
   end
   writeToCurrent('set zlabel "' .. label .. '"')
   refreshCurrent()
end
function gnuplot.title(label)
   if not _gptable.hasrefresh then
      print('gnuplot.title disabled')
      return
   end
   writeToCurrent('set title "' .. label .. '"')
   refreshCurrent()
end
function gnuplot.grid(toggle)
   if not _gptable.hasrefresh then
      print('gnuplot.grid disabled')
      return
   end
   if toggle then
      writeToCurrent('set grid')
      refreshCurrent()
   else
      writeToCurrent('unset grid')
      refreshCurrent()
   end
end

function gnuplot.movelegend(hloc,vloc)
   if not _gptable.hasrefresh then
      print('gnuplot.movelegend disabled')
      return
   end
   if hloc ~= 'left' and hloc ~= 'right' and hloc ~= 'center' then
      error('horizontal location is unknown : plot.movelegend expects 2 strings as location {left|right|center}{bottom|top|middle}')
   end
   if vloc ~= 'bottom' and vloc ~= 'top' and vloc ~= 'middle' then
      error('horizontal location is unknown : plot.movelegend expects 2 strings as location {left|right|center}{bottom|top|middle}')
   end
   writeToCurrent('set key ' .. hloc .. ' ' .. vloc)
   refreshCurrent()
end

function gnuplot.axis(axis)
   if not _gptable.hasrefresh then
      print('gnuplot.axis disabled')
      return
   end
   if axis == 'auto' then
      writeToCurrent('set size nosquare')
      writeToCurrent('set autoscale')
      refreshCurrent()
   elseif axis == 'image' or axis == 'equal' then
      writeToCurrent('set size ratio -1')
      refreshCurrent()
   elseif axis == 'fill' then
      writeToCurrent('set size ratio 1,1')
      refreshCurrent()
   elseif type(axis) == 'table' then
      if #axis ~= 4 then print('axis should have 4 componets {xmin,xmax,ymin,ymax}'); return end
      writeToCurrent('set xrange [' .. axis[1] .. ':' .. axis[2] .. ']')
      writeToCurrent('set yrange [' .. axis[3] .. ':' .. axis[4] .. ']')
      refreshCurrent()
   end
end

function gnuplot.raw(str)
   writeToCurrent(str)
end

-- plot(x)
-- plot(x,'.'), plot(x,'.-')
-- plot(x,y,'.'), plot(x,y,'.-')
-- plot({x1,y1,'.'},{x2,y2,'.-'})
function gnuplot.plot(...)
   local arg = {...}
   if select('#',...) == 0 then
      error('no inputs, expecting at least a vector')
   end

   local function isvector(v)
      return (type(v) == 'table' and type(v[1]) == 'number')
          or (type(v) == 'userdata' and type(v[1]) == 'number')
   end

   local formats = {}
   local xdata = {}
   local ydata = {}
   local legends = {}

   if not isvector(arg[1]) and type(arg[1]) == "table" then
      for i,v in ipairs(arg) do
         local l,x,y,f = getvars(v)
         legends[#legends+1] = l
         formats[#formats+1] = f
         xdata[#xdata+1] = x
         ydata[#ydata+1] = y
      end
   else
      local l,x,y,f = getvars(arg)
      legends[#legends+1] = l
      formats[#formats+1] = f
      xdata[#xdata+1] = x
      ydata[#ydata+1] = y
   end

   gnulplot(legends,xdata,ydata,formats)
end

-- splot(z)
-- splot(x,y,z,format)
-- splot({x1,y1,z1},{x2,y2,z2})
function gnuplot.splot(...)
   local arg = {...}
   if select('#',...) == 0 then
      error('no inputs, expecting at least a matrix')
   end

   local function ismatrix(v)
      return (type(v) == 'table' and type(v[1]) == 'table' and type(v[1][1]) == 'number')
          or (type(v) == 'userdata' and type(v[1]) == 'userdata' and type(v[1][1]) == 'number')
   end

   local formats = {}
   local xdata = {}
   local ydata = {}
   local zdata = {}
   local legends = {}

   if not ismatrix(arg[1]) and type(arg[1]) == "table" then
      for i,v in ipairs(arg) do
         local l,x,y,z,f = getsplotvars(v)
         legends[#legends+1] = l
         formats[#formats+1] = f
         xdata[#xdata+1] = x
         ydata[#ydata+1] = y
         zdata[#zdata+1] = z
      end
   else
      local l,x,y,z,f = getsplotvars(arg)
      legends[#legends+1] = l
      formats[#formats+1] = f
      xdata[#xdata+1] = x
      ydata[#ydata+1] = y
      zdata[#zdata+1] = z
   end
   gnusplot(legends,xdata,ydata,zdata,formats)
end

-- plot3d(z)
-- plot3d(x,y,z,palette)
-- plot3d({x1,y1,z1},{x2,y2,z2})
function gnuplot.plot3d(...)
   local arg = {...}
   if select('#',...) == 0 then
      error('no inputs, expecting at least a matrix')
   end

   local function ismatrix(v)
      return type(v) == 'table' and type(v[1]) == 'table' and type(v[1][1]) == 'number'
   end

   local palettes = {}
   local xdata = {}
   local ydata = {}
   local zdata = {}
   local legends = {}

   if not ismatrix(arg[1]) and type(arg[1]) == "table" then
      for i,v in ipairs(arg) do
         local l,x,y,z,p = getplot3dvars(v)
         legends[#legends+1] = l
         palettes[#palettes+1] = p 
         xdata[#xdata+1] = x
         ydata[#ydata+1] = y
         zdata[#zdata+1] = z
      end
   else
      local l,x,y,z,p = getplot3dvars(arg)
      legends[#legends+1] = l
      palettes[#palettes+1] = p 
      xdata[#xdata+1] = x
      ydata[#ydata+1] = y
      zdata[#zdata+1] = z
   end
   gnuplot3d(legends,xdata,ydata,zdata,palettes)
end

-- imagesc(x) -- x 2D matrix [0 .. 1]
function gnuplot.imagesc(...)
   local arg = {...}
   if select('#',...) == 0 then
      error('no inputs, expecting at least a matrix')
   end
   gnuimagesc(getimagescvars(arg))
end

-- bar(y)
-- bar(x,y)
function gnuplot.bar(...)
   local arg = {...}
   local nargs = {}
   for i = 1,select('#',...) do
      table.insert(nargs,arg[i])
   end
   table.insert(nargs, '|')
   gnuplot.plot(nargs)
end

findgnuplot()

return gnuplot
