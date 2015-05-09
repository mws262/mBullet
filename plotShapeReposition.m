function plotShapeReposition(movingObj,data,QUAT,POS)

dataSize = size(data.xd);

Z = [data.xd(:),data.yd(:),data.zd(:)];
Zprime = quatrotate(QUAT,Z);


xprime = reshape(Zprime(:,1),dataSize) + POS(1);
yprime = reshape(Zprime(:,2),dataSize) + POS(2);
zprime = reshape(Zprime(:,3),dataSize) + POS(3);
set(movingObj,'XData',xprime,'YData',yprime,'ZData',zprime,'CData',data.cd);


end