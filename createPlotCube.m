function [plotObj,data] = createPlotCube(xdim,ydim,zdim)
% This creates the matlab plot object for a box (not just a cube, TODO
% change name).
%
% Matthew Sheen
Vert = [ ...
  0 0 0 ;
  1 0 0 ;
  1 1 0 ;
  0 1 0 ;
  0 0 1 ;
  1 0 1 ;
  1 1 1 ;
  0 1 1]-.5;
Faces = [ ...
   1 2 6 5 ;
   2 3 7 6 ;
   3 4 8 7 ;
   4 1 5 8 ;
   1 2 3 4 ;
   5 6 7 8 ];

ptch.Vertices = Vert;
ptch.Faces = Faces;
ptch.FaceVertexCData = hsv(6);
ptch.FaceColor = 'flat';
plotObj = patch(ptch);

data.xd = xdim*get(plotObj,'XData');
data.yd = ydim*get(plotObj,'YData');
data.zd = zdim*get(plotObj,'ZData');
data.cd = get(plotObj,'CData');

set(plotObj,'XData',data.xd,'YData',data.yd,'ZData',data.zd,'CData',data.cd);
% set(gca,'CLim',cl)
end