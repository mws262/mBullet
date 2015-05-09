function [dynamicWorld,collisionShapes] = setupWorld(gravity)
%Performs a number of setup functions to setup jBullet and create a "world"
%This includes: defining solvers, world boundaries, gravity, etc.
% This is stuff you could easily do from your main script, I'm just trying
% to make things a little easier.
%NOTE: If you want an object to fall "down" the gravity should be negative.
%I'm defining -z as DOWN.
%
% Matthew Sheen

%Import all the classes we need:
import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import javax.vecmath.*;
import com.bulletphysics.util.ObjectArrayList;


collisionConfiguration =  DefaultCollisionConfiguration();
dispatcher =  CollisionDispatcher(collisionConfiguration);

%World Boundaries
worldAabbMin =  Vector3f(-1000,-1000,-1000);
worldAabbMax =  Vector3f(1000,1000,1000);
overlappingPairCache =  AxisSweep3(worldAabbMin, worldAabbMax);
%Several solvers possible -- investigate later
solver =  SequentialImpulseConstraintSolver();

%Create the world "object" and define gravity.
dynamicWorld =  DiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
dynamicWorld.setGravity(Vector3f(0,0,gravity));
        
%How much penetration is allowed in collisions. May need to adjust for
%stability
dispatchInf = dynamicWorld.getDispatchInfo;
% dispatchInf.allowedCcdPenetration = 0.05;

%Create a running tab of all the objects we throw into our world.
collisionShapes = ObjectArrayList();