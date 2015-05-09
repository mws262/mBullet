function [sphereObj,data] = createPlotSphere(radius)
%Creates the matlab plot object for a sphere (or a faceted sphere
%approximation anyway).
%
% Matthew Sheen

[x y z] = sphere; 
sphereObj = patch(surf2patch(x,y,z,z));
shading faceted;

data.xd = .5*radius*get(sphereObj,'XData');
data.yd = .5*radius*get(sphereObj,'YData');
data.zd = .5*radius*get(sphereObj,'ZData');
data.cd = .5*get(sphereObj,'CData');

set(sphereObj,'XData',data.xd,'YData',data.yd,'ZData',data.zd,'CData',data.cd);
% set(gca,'CLim',cl)
end