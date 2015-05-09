%Start fresh
clear all
clc
close all

%Import all the classes we need:
import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
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

%Define World Gravity
gravity = -2;

%Define characteristics of the shapes -- can be set for each shape
%individually if desired.
objProp = struct('restitution',0.6,'friction',0.5,'linDamp',0.5,'angDamp',0);

%Define ground characteristics.
groundOrig = [0,0,-5]; %Center position
groundDim = [5,5,.5]; %Dimensions
groundAng = [0 0 0]; %Euler angle rotation

%Define Box 1 characteristics.
objMass1 = 1;
objOrig1 = [0,0,1]; %Center position
objDim1 = [2,.05,.05]; %Dimensions
objAng1 = [0 pi/4 0]; %Euler angle rotation

%Setup the world, define solvers, gravity. We get an object representing
%our world and a special Java array that keeps a running tab of our bodies
%for collision purposes.
[dynamicWorld,collisionShapes] = setupWorld(gravity);

%Add the ground to our world and create the graphics for it.
[dynamicWorld, collisionShapes, groundPlotObj ] = createGround(dynamicWorld, collisionShapes,groundOrig, groundDim, groundAng, objProp);
       
%Add the first dynamic object to the world. Return our updated world, the updated list of collision shapes, the body object, and a Matlab plot object 
[dynamicWorld, collisionShapes, body1, plotObj1, plotData1 ] = createShape('box',dynamicWorld, collisionShapes,objOrig1, objDim1, objAng1, objMass1, objProp); 

[tarray,PosArray,RotArray] = bulletSim(dynamicWorld, 10, .01);

cameramenu; grid; axis([-4 3 -5 3 -5 3]); axis square
set(gca, 'CameraPosition', .7*[   .8   55  8], 'CameraViewAngle',  13.1736)

        for i = 1:5:size(PosArray{1},1)

            plotShapeReposition(plotObj1,plotData1,RotArray{2}(i,:),PosArray{2}(i,:));
            pause(0.001)
        end