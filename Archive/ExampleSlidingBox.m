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
gravity = -9.81;

%Define characteristics of the shapes -- can be set for each shape
%individually if desired.
objProp = struct('restitution',0,'friction',1,'linDamp',0,'angDamp',0);

%Define ground characteristics.
groundOrig = [0,0,-0.25]; %Center position
groundDim = [20,5,.5]; %Dimensions
groundAng = [0 0 0]; %Euler angle rotation

%Define Box 1 characteristics.
objMass1 = 1;
objOrig1 = [0,0,.501]; %Center position
objDim1 = [1,1,1]; %Dimensions
objAng1 = [0 0 0]; %Euler angle rotation


%Setup the world, define solvers, gravity. We get an object representing
%our world and a special Java array that keeps a running tab of our bodies
%for collision purposes.
[dynamicWorld,collisionShapes] = setupWorld(gravity);

%Add the ground to our world and create the graphics for it.
[dynamicWorld, collisionShapes, groundPlotObj ] = createGround(dynamicWorld, collisionShapes,groundOrig, groundDim, groundAng, objProp);
       
%Add the first dynamic object to the world. Return our updated world, the updated list of collision shapes, the body object, and a Matlab plot object 
[dynamicWorld, collisionShapes, body1, plotObj1, plotData1 ] = createShape('box',dynamicWorld, collisionShapes,objOrig1, objDim1, objAng1, objMass1, objProp); 

body1.setLinearVelocity(Vector3f(10.0,0,0));
% hinge = HingeConstraint(body2, Vector3f(endaxis(1),endaxis(2),endaxis(3)),Vector3f(0,1,0)); 
% dynamicWorld.addConstraint(hinge);

[tarray,PosArray,RotArray,LinVelArray,AngVelArray] = bulletSim(dynamicWorld, 5, .01);

% for i = 1:size(LinVelArray{3},1)
%    LinVelSq1(i) = dot(LinVelArray{2}(i,:),LinVelArray{2}(i,:));
%    %AngVelSq1(i) = dot(AngVelArray{2}(i,:),AngVelArray{2}(i,:));
%    
%    LinVelSq2(i) = dot(LinVelArray{3}(i,:),LinVelArray{3}(i,:));
%    AngVelSq2(i) = dot(AngVelArray{3}(i,:),AngVelArray{3}(i,:)); 
%    
%    LocalAngVel1(i,:) = quatrotate(RotArray{2}(i,:),AngVelArray{2}(i,:));
%    LocalAngVel2(i,:) = quatrotate(RotArray{3}(i,:),AngVelArray{3}(i,:));
% end


% LinKEnergy = ( LinVelSq2*1/2*objMass2)'; %+ LinVelSq1*1/2*objMass1 )';
% PotEnergy = PosArray{3}(:,3)*-gravity*objMass2;% + PosArray{2}(:,3)*-gravity*objMass1;
% 
% Inertia1 = plotData1.localInertia;
% Inertia2 = plotData2.localInertia;
% 
% AngKEnergy =  1/2*LocalAngVel2.*LocalAngVel2*Inertia2;% + 1/2*LocalAngVel1.*LocalAngVel1*Inertia1;

% % % 
%    figure
%    plot(tarray,1/2*LocalAngVel2.*LocalAngVel2*Inertia2,tarray, 1/2*AngVelSq2*1/12*objMass2*(1.5^2+.1^2))

cameramenu; grid; axis([-10 3 -3 3 -0 2]); axis equal
set(gca, 'CameraPosition', .4*[   .8   55  8], 'CameraViewAngle',  10)

        for i = 1:5:size(PosArray{1},1)
%fprintf('%f\n',PosArray{2}(i,1));
            plotShapeReposition(plotObj1,plotData1,RotArray{2}(i,:),PosArray{2}(i,:));
            pause(0.001)
        end  
        
       figure
       hold on
       %plot(PosArray{2}(:,1))
       plot(PosArray{2}(:,3))
       hold off
% 
%  figure       
% plot(tarray, LinKEnergy + AngKEnergy) %+ PotEnergy)        
        
            