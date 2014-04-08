function [varargout] = dijkstra_path2(A, C, istart, jstart)

ipath = istart;
jpath = jstart;

r = 0.5;
nth = 64;
th = 2*pi*[0:nth-1]./nth;

xr = r*cos(th);
yr = r*sin(th);

while(1)
  x0 = ipath(end);
  y0 = jpath(end);
  d0 = subs_interp(A, x0, y0);
  
  % Neighbors:
  x1 = x0+xr;
  y1 = y0+yr;
  d1 = subs_interp(A, x1, y1);
  
  [dmin, imin] = min(d1);
  if (dmin > d0),
    break;
  end

  ipath = [ipath; x1(imin)];
  jpath = [jpath; y1(imin)];
end

if nargout == 1 || nargout == 0
    varargout{1} = [ipath, jpath];
elseif nargout == 2
    varargout{1} = ipath;
    varargout{2} = jpath;
end
end


function [amin,i,j] = min2(a)
[amin, imin] = min(a(:));
[i,j] = ind2sub(size(a), imin);
end