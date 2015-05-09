Setup instructions:

Move the mBullet folder into your Matlab directory. You will also need to have Matlab load the Java .jar file that contains the Bullet source code. For instructions on doing this, open the “startup.m” file found in this folder.

Run ExampleBoxHitsPend for a quick example. More examples will (hopefully) be added soon.

TODO:
Cylinder collision shapes.


Use:

Javadoc:
All the physics engine functions are part of jBullet, a java port of the Bullet engine. Documentation of the available classes can be found at the jBullet javadoc site:

http://jbullet.advel.cz/javadoc/

Syntax:
In general, you can use example jBullet code natively in Matlab with a few alterations. Pretend you’re coding Matlab while using the Java functions. Matlab is much more laid-back about variable types. So the Java might be something like:

		Transform myTransform = new Transform();

Converting to Matlab-speak:

		myTransform = Transform();

Also, in Java, 4.f means “A floating point number (not an integer) of value four.” Matlab will only take 4 and doesn’t need a type. A few other differences: != in Java vs. ~= in Matlab, for loop structure, and commenting syntax.

Importing classes:
The startup file loads the Java .jar file into the dynamic file path, but each Matlab file you create still needs to import the specific jBullet classes you’re going to use. For example:

import com.bulletphysics.dynamics.constraintsolver.*;
import javax.vecmath.*;

See some of the demo files for other examples.

Useful Java tricks in Matlab:
Figuring out available functions:
methodsview command (example: methodsview com.bulletphysics.linearmath) displays all available java methods within that particular class. In the Matlab world: “what functions does this java class give me?”

Figuring out important variables:
fieldnames(object) shows the fields within a particular object. Example:
        dispatchInf = dynamicWorld.getDispatchInfo;
        fieldnames(dispatchInf)
In the Matlab world: “I’ve created a new object. Now what parameters can I change in it?”

Important Bullet notes:

Bullet does everything in HALF-lengths. For instance, if you define a cube [1,1,1], you get a 2x2x2 cube (i.e. -1 to +1 in every direction). For my Matlab functions, I chose to do everything in terms of actual dimensions due to personal confusion.



