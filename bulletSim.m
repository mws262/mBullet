function [tarray,PosArray,RotArray,LinVelArray,AngVelArray] = bulletSim(dynamicWorld, duration, stepSize)
%This actually runs the simulation given a definition of the world, a
%simulation duration, and a physics time step size.
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
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;

        %How many objects
        numObjects = dynamicWorld.getCollisionObjectArray().size();
        numSteps = duration/stepSize+1; % We start at 0, so one extra step.
        
        %Preallocate some cell arrays. Each element is a 
        dummyPos = zeros(numSteps,3);
        dummyRot = zeros(numSteps,4);
        currentRot = Quat4f();
        currentPos = Vector3f();
        RotArray = repmat({dummyRot},1,numObjects);
        PosArray = repmat({dummyPos},1,numObjects);
        
        currentAngVel = Vector3f();
        currentLinVel = Vector3f();
        
        AngVelArray = repmat({dummyPos},1,numObjects);
        LinVelArray = repmat({dummyPos},1,numObjects);
        
        tarray = 0:stepSize:duration; %Make our time array
           
		for i=0:numSteps-1
			dynamicWorld.stepSimulation(stepSize, 5); %Second parameter is max allowed "substeps" -- may need to change for speed
            %At each sim step, go through each object to look up
            %position & rotation
            for j = 0:numObjects-1
                obj = dynamicWorld.getCollisionObjectArray().getQuick(j);
                body = RigidBody.upcast(obj);
                trans = Transform();
                body.getMotionState().getWorldTransform(trans);
            
            		RotArray{j+1}(i+1,:) = cell2mat(struct2cell(struct(body.getOrientation(currentRot))))';
                    PosArray{j+1}(i+1,:) = cell2mat(struct2cell(struct(body.getCenterOfMassPosition(currentPos))))';
                    
                    AngVelArray{j+1}(i+1,:) = cell2mat(struct2cell(struct(body.getAngularVelocity(currentAngVel))))';
                    LinVelArray{j+1}(i+1,:) = cell2mat(struct2cell(struct(body.getLinearVelocity(currentLinVel))))';
            end       
        end
end