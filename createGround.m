function [dynamicWorld, collisionShapes, groundObj ] = createGround(dynamicWorld, collisionShapes,origin, dimensions,angles,objProp)
%Use to define a static box (e.g. the ground) and create a plot object for
%later use.
%
% Matthew Sheen

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import javax.vecmath.*;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.RigidBody;

      %%% STATIC SHAPE -- The ground %%%% 
      %Make the box of specified dimensions
        groundShape = BoxShape( Vector3f(.5*dimensions(1), .5*dimensions(2), .5*dimensions(3)));
		groundTransform =  Transform();
		groundTransform.setIdentity();
      %Place its center at the specified origin
		groundTransform.origin.set(Vector3f(origin(1), origin(2), origin(3)));
      %Rotate it as defined
        groundRot = angle2quat(angles(1),angles(2),angles(3)); %This is the matlab quaternion rotation
        rotation = Quat4f(groundRot); %Java has its own quaternion type in the vecmath package.
        groundTransform.setRotation(rotation);
        collisionShapes.add(groundShape);
        
        %Immovable objects have zero mass in Bullet.
		mass = 0;
		localInertia = Vector3f(0, 0, 0);
	    myMotionState = DefaultMotionState(groundTransform);
		rbInfo = RigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia);
		body = RigidBody(rbInfo);
        
        %Set properties:
        body.setRestitution(objProp.restitution);
        body.setFriction(objProp.friction);
        body.setDamping(objProp.linDamp,objProp.angDamp);
        
        
		dynamicWorld.addRigidBody(body);
        
        %Create plots
        [groundObj,data] = createPlotCube(dimensions(1), dimensions(2), dimensions(3));
         plotShapeReposition(groundObj,data,groundRot,origin);