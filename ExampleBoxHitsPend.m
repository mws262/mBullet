% mBullet example:
% 
% Matthew Sheen

%Start fresh
clear all
clc
close all

%Import all the classes we need:
import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;!
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.dynamics.constraintsolver.*;
import javax.vecmath.*;

%Set the animation playback speed.
playback = 0.5;

%Define World Gravity
gravity = -10;

%Define characteristics of the shapes -- can be set for each shape
%individually if desired.
objProp = struct('restitution',0.1,'friction',0.5,'linDamp',0,'angDamp',0);

%Define ground characteristics.git
groundOrig = [0,0,-3]; %Center position
groundDim = [5,5,.5]; %Dimensions
groundAng = [0 pi/4 0]; %Euler angle rotation

%Define Box 1 characteristics.
objMass1 = 1;
objOrig1 = [-1,0,0]; %Center position
objDim1 = [2,.4,.4]; %Dimensions
objAng1 = [0 pi/4 0]; %Euler angle rotation

%Define Box 2 characteristics.
objMass2 = 1;
objOrig2 = [1, 0, 10]; %Center position
objDim2 = [1,1,1]; %Dimensions
objAng2 = [0,pi/3,pi/4]; %Euler angle rotation

%Define sphere 1 characteristics.
objMass3 = 1;
objOrig3 = [1, 0, 14]; %Center position
objDim3 = 1; %Dimensions
objAng3 = [0,0,0]; %Euler angle rotation

%Define sphere 2 characteristics.
objMass4 = 1;
objOrig4 = [1, 0, 20]; %Center position
objDim4 = 1; %Dimensions
objAng4 = [0,0,0]; %Euler angle rotation

%Setup the world, define solvers, gravity. We get an object representing
%our world and a special Java array that keeps a running tab of our bodies
%for collision purposes.
[dynamicWorld,collisionShapes] = setupWorld(gravity);

%Add the ground to our world and create the graphics for it.
[dynamicWorld, collisionShapes, groundPlotObj ] = createGround(dynamicWorld, collisionShapes,groundOrig, groundDim, groundAng, objProp);
       
%Add the first dynamic object to the world. Return our updated world, the updated list of collision shapes, the body object, and a Matlab plot object 
[dynamicWorld, collisionShapes, body1, plotObj1, plotData1 ] = createShape('box',dynamicWorld, collisionShapes,objOrig1, objDim1, objAng1, objMass1, objProp); 
        
hinge = HingeConstraint(body1, Vector3f(1,0,0),Vector3f(0,1,0)); 
dynamicWorld.addConstraint(hinge);
                          
%Add the second dynamic object to the world. Return our updated world, the updated list of collision shapes, the body object, and a Matlab plot object 
[dynamicWorld, collisionShapes, body2, plotObj2, plotData2 ] = createShape('box',dynamicWorld, collisionShapes,objOrig2, objDim2, objAng2, objMass2, objProp) ;
 
%Add the third dynamic object to the world. Return our updated world, the updated list of collision shapes, the body object, and a Matlab plot object 
[dynamicWorld, collisionShapes, body3, plotObj3, plotData3 ] = createShape('sphere',dynamicWorld, collisionShapes,objOrig3, objDim3, objAng3, objMass3, objProp);

%Add the Fourth dynamic object to the world. Return our updated world, the updated list of collision shapes, the body object, and a Matlab plot object 
[dynamicWorld, collisionShapes, body4, plotObj4, plotData4 ] = createShape('sphere',dynamicWorld, collisionShapes,objOrig4, objDim4, objAng4, objMass4, objProp);

%Run it for the time specified with a given timestep.
[tarray,PosArray,RotArray] = bulletSim(dynamicWorld, 10, .01);




%Plotting:
cameramenu; grid; axis([-4 3 -5 3 -5 3]); axis square
set(gca, 'CameraPosition', .7*[   .8   55  8], 'CameraViewAngle',  13.1736)

tic %Start the timer.
while toc<tarray(end)/playback 

    %If i close the figure, this prevents the error message (anal
    %programming)
    if ishandle(1) == 0
        break;
    end
    
    tstar = playback*toc; %Get the time
    
    %Takes plotShapeReposition(plotObj, plotData, Rotation, Position) --
    %I'm just interpolating below so I can have smooth playback even if I'm
    %going 20% real time or 200% real time.
    plotShapeReposition(plotObj1,plotData1,interp1(tarray,RotArray{2},tstar),interp1(tarray,PosArray{2},tstar));
    plotShapeReposition(plotObj2,plotData2,interp1(tarray,RotArray{3},tstar),interp1(tarray,PosArray{3},tstar));
    plotShapeReposition(plotObj3,plotData3,interp1(tarray,RotArray{4},tstar),interp1(tarray,PosArray{4},tstar));
    plotShapeReposition(plotObj4,plotData4,interp1(tarray,RotArray{5},tstar),interp1(tarray,PosArray{5},tstar));
    drawnow;
        
end   
        
        
        
        
        
        
            