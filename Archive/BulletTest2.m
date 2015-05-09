clear all
clc
close all
javaaddpath jbullet.jar

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
import javax.vecmath.Vector3f;
import javax.vecmath.Quat4f;


		collisionConfiguration =  DefaultCollisionConfiguration();
		dispatcher =  CollisionDispatcher(collisionConfiguration);
		 worldAabbMin =  Vector3f(-10000,-10000,-10000);
		 worldAabbMax =  Vector3f(10000,10000,10000);
		 overlappingPairCache =  AxisSweep3(worldAabbMin, worldAabbMax);
		 solver =  SequentialImpulseConstraintSolver();
		
		dynamicWorld =  DiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
		dynamicWorld.setGravity(Vector3f(0,-2,0));
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
          shape =  BoxShape(Vector3f(2,2,2));
 	      localInertia =  Vector3f(0,0,0);
	      shape.calculateLocalInertia(mass, localInertia);
		  collisionShapes.add(shape);
          
          
	    startTransform = Transform();
	    startTransform.setIdentity();
	    startTransform.origin.set(0, 0, 0);
        
        initialRot = angle2quat(pi/6,0,pi/6);
        rotation = Quat4f(initialRot);
        startTransform.setRotation(rotation);
        
	    ms =  DefaultMotionState(startTransform);
	    
	    rbInfo =  RigidBodyConstructionInfo(mass, ms, shape, localInertia);
	    body = RigidBody(rbInfo);
	    body.setRestitution(0.1);
	    body.setFriction(0.10);
	    body.setDamping(0,0);

	    dynamicWorld.addRigidBody(body);
        
        
        
    %Sim    
        currentRot = Quat4f();
        CubeRotArray = [];
        currentPos = Vector3f();
        CubePosArray = [];
		for i=0:200
			dynamicWorld.stepSimulation(1 / 30, 10);

			obj = dynamicWorld.getCollisionObjectArray().getQuick(1);
			body = RigidBody.upcast(obj);
			trans = Transform();
    
			body.getMotionState().getWorldTransform(trans);
            
            		CubeRotArray(i+1,:) = cell2mat(struct2cell(struct(body.getOrientation(currentRot))))';
                    CubePosArray(i+1,:) = cell2mat(struct2cell(struct(body.getCenterOfMassPosition(currentPos))))';
                    finalYPos(i+1) = trans.origin.y;

        end
       %methodsview RigidBody
       % plot(finalYPos)
        
        
           hold on
           
           
d = [-1 1];
[x,y,z] = meshgrid(d,d,d);  % A cube
x = [x(:);0];
y = [y(:);0];
z = [z(:);0];
% [x,y,z] are corners of a cube plus the center.
X = [x(:) y(:) z(:)];

%X=quatrotate(initialRot,X)
                     cameramenu; grid; axis([-3 3 -5 3 -3 3])
                     set(gca, 'CameraPosition', .7*[   -0.8448   43.5514  -37.6093])
                     set(gca, 'CameraViewAngle',  13.1736)

        for i = 1:size(CubePosArray,1)
            RotX=quatrotate(CubeRotArray(i,:),X);
            TransX = RotX + repmat(CubePosArray(i,:),[size(X,1),1]);
            Tes = delaunayn(TransX);
            tetramesh(Tes,TransX,repmat([0.7, 1.0, 0.0],[12,1]))
            pause(0.01)
            cla
            
            
            
        end
hold off
        
        
        
        
        
        
        
        
            