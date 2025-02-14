

## ofxDabPhysics

**Author**:  Daniel Bisig - Coventry University, UK - [ad5041@coventry.ac.uk](ad5041@coventry.ac.uk) - Zurich University of the Arts, CH - [daniel.bisig@zhdk.ch](daniel.bisig@zhdk.ch)

**Dependencies**: [ofxDabBase](https://bitbucket.org/dbisig/ofxdabbase_011/src/master/), [ofxDabMath](https://bitbucket.org/dbisig/ofxdabmath_011/src/master/), [ofxDabOsc](https://bitbucket.org/dbisig/ofxdabosc_011/src/master/), [Bullet3](https://github.com/bulletphysics/bullet3) (included), [tinyxml2](https://github.com/leethomason/tinyxml2) (included)

---

## Summary

ofxDabPhysics is a wrapper for the Bullet physics engine that facilitates the simulation of bodies with articulated morphologies. ofxDabPhysics implements behaviours for generating autonomous movements. It also offers the possibility to import articulated morphologies from URDF file. Furthermore, ofxDabPhysics can communicate via OSC messages with other applications. OSC messages can be used by other applications to receive the values of simulation parameters. In addition, OSC messages can also be employed as commands to remotely configure and control the simulation. Finally, ofxDabPhysics provides simple rendering functionality to visualise the simulation. The code is compatible with OpenFrameworks 0.11 and has been tested on Windows.

### Entities

**Body**: represents an articulated morphology that consist of body parts, body joints, body motors, and behaviours.

### Morphology

**BodyPart**: represents a rigid body which includes both its dynamics properties and collision boundary.

**BodyJoint**: represents a passive constraint between two body parts. This class is mainly used as base for an universal joint.

**UniversalJoint**: a joint that possesses individual lower and upper for its three perpendicular rotation axes.

**BodyMotor**: represents an active constraint between two body parts. Active constraints can be actuated. This class is mainly used as base for an universal motor.

**UniversalMotor**: an active joint that can operate in three different modes. The motor can operate in three different modes, as free spinning motor, as servo motor with position control, or as damped spring that possesses a linear and/or angular rest length.

### Behaviours

**Behavior**: a behaviour is part of a body and can manipulate physical properties of the body parts, joints, or motors that it has been assigned to. Alternatively or in addition, a behaviour can also generate forces that impact on body parts. 

### Communication

**OscControl**: manages all communication with the physics simulation. Routes incoming OSC message to parsers for controlling either the simulation or its visualisation.

**PhysicsOscControl**: Parses incoming OSC messages to control the physics simulation.

**VisualsOscControl**: Parses incoming OSC messages to control the visualisation of the physics simulation.

### Serialisation

**UrdfImporter**: loads robot definition files in URDF format and converts them into articulated bodies for simulation.

### Visualisation

**BodyVisualization**: provides simple functionality for rendering body parts either as simple geometrical primitives or as triangulated meshes. Also provides a camera and handles material specifications for rendering.

