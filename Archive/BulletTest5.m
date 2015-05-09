clear all
clc
close all
javaaddpath jbullet.jar;

import com.bulletphysics.collision.broadphase.AxisSweep3;
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
import com.bulletphysics.util.ObjectArrayList;
import com.bulletphysics.dynamics.constraintsolver.*;
import javax.vecmath.*;

BoxSize = [2 .2 .2];
BoxSize2 = [2 .2 .2];

		collisionConfiguration =  DefaultCollisionConfiguration();
		dispatcher =  CollisionDispatcher(collisionConfiguration);
		 worldAabbMin =  Vector3f(-10000,-10000,-10000);
		 worldAabbMax =  Vector3f(10000,10000,10000);
		 overlappingPairCache =  AxisSweep3(worldAabbMin, worldAabbMax);
		 solver =  SequentialImpulseConstraintSolver();
		
		dynamicWorld =  DiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
		dynamicWorld.setGravity(Vector3f(0,0,-2));
		%dynamicWorld.getDispatchInfo().allowedCcdPenetration = 0;
        
        collisionShapes =  ObjectArrayList();
        
      %%% STATIC SHAPE -- The ground %%%% 
        groundShape = BoxShape( Vector3f(100, 50, 100));
		groundTransform =  Transform();
		groundTransform.setIdentity();
		groundTransform.origin.set(Vector3f(0, -56, 0));
        collisionShapes.add(groundShape);
        
		mass = 0;
		localInertia = Vector3f(0, 0, 0);
	    myMotionState = DefaultMotionState(groundTransform);
		rbInfo = RigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia);
		body = RigidBody(rbInfo);
		dynamicWorld.addRigidBody(body);
        
      %%% DYNAMIC SHAPE %%%% 
      mass = 1;
          shape =  BoxShape(Vector3f(.5*BoxSize(1),.5*BoxSize(2),.5*BoxSize(3)));
 	      localInertia =  Vector3f(0,0,0);
	      shape.calculateLocalInertia(mass, localInertia);
		  collisionShapes.add(shape);
          
          
	    startTransform = Transform();
	    startTransform.setIdentity();
	    startTransform.origin.set(-1, 0, 0);
        
        
        initialRot = angle2quat(0,0,0);
        rotation = Quat4f(initialRot);
        startTransform.setRotation(rotation);
    
        
	    ms =  DefaultMotionState(startTransform);
	    
	    rbInfo =  RigidBodyConstructionInfo(mass, ms, shape, localInertia);
	    body1 = RigidBody(rbInfo);
	    body1.setRestitution(0.1);
	    body1.setFriction(0.10);
	    body1.setDamping(0,0);
        
        hinge = HingeConstraint(body1, Vector3f(1,0,0),Vector3f(0,1,0)) 
        dynamicWorld.addConstraint(hinge);
	    dynamicWorld.addRigidBody(body1);

              %%% DYNAMIC SHAPE 2 %%%% 
      mass = 1;
          shape =  BoxShape(Vector3f(.5*BoxSize2(1),.5*BoxSize2(2),.5*BoxSize2(3)));
 	      localInertia =  Vector3f(0,0,0);
	      shape.calculateLocalInertia(mass, localInertia);
		  collisionShapes.add(shape);
          
          
	    startTransform = Transform();
	    startTransform.setIdentity();
	    startTransform.origin.set(-3, 0.2, 0);
        
        initialRot = angle2quat(0,pi/3,0);
        rotation = Quat4f(initialRot);
        startTransform.setRotation(rotation);
        
	    ms =  DefaultMotionState(startTransform);
	    
	    rbInfo =  RigidBodyConstructionInfo(mass, ms, shape, localInertia);
	    body2 = RigidBody(rbInfo);
	    body2.setRestitution(0.1);
	    body2.setFriction(0.10);
	    body2.setDamping(0,0);

        hinge = HingeConstraint(body1,body2,Vector3f(-1,0,0),Vector3f(1,.2,0),Vector3f(0,1,0),Vector3f(0,1,0)) 
        dynamicWorld.addConstraint(hinge);
	    dynamicWorld.addRigidBody(body2);
        
    %Sim    
        currentRot = Quat4f();
        CubeRotArray = [];
        currentPos = Vector3f();
        CubePosArray = [];
		for i=0:1200
			dynamicWorld.stepSimulation(1 / 60, 10);

			obj = dynamicWorld.getCollisionObjectArray().getQuick(1);
			body = RigidBody.upcast(obj);
			trans = Transform();
    
			body.getMotionState().getWorldTransform(trans);
            
            		CubeRotArray1(i+1,:) = cell2mat(struct2cell(struct(body.getOrientation(currentRot))))';
                    CubePosArray1(i+1,:) = cell2mat(struct2cell(struct(body.getCenterOfMassPosition(currentPos))))';
                    
            obj = dynamicWorld.getCollisionObjectArray().getQuick(2);
			body = RigidBody.upcast(obj);
			trans = Transform();
    
			body.getMotionState().getWorldTransform(trans);
            
            		CubeRotArray2(i+1,:) = cell2mat(struct2cell(struct(body.getOrientation(currentRot))))';
                    CubePosArray2(i+1,:) = cell2mat(struct2cell(struct(body.getCenterOfMassPosition(currentPos))))';
        end
        
[p1,xd1,yd1,zd1,cd1] = CreateCube(BoxSize(1),BoxSize(2),BoxSize(3));
[p2,xd2,yd2,zd2,cd2] = CreateCube(BoxSize2(1),BoxSize2(2),BoxSize2(3));

                     cameramenu; grid; axis([-4 3 -5 3 -3 3]); axis square
                     set(gca, 'CameraPosition', .7*[   -.8   43.5514  8])
                     set(gca, 'CameraViewAngle',  13.1736)
                     xlabel('x')
                     ylabel('y')
                     zlabel('z')

        for i = 1:4:size(CubePosArray1,1)

            CubeReposition(p1,xd1,yd1,zd1,cd1,CubeRotArray1(i,:),CubePosArray1(i,:));
            CubeReposition(p2,xd2,yd2,zd2,cd2,CubeRotArray2(i,:),CubePosArray2(i,:));
            pause(0.001)
        end
        
        
        
        
        
        
        
        
            