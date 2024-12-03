import { lerp } from '../helpers'
//import AmmoJS from "../ammo/ammo.wasm.es.js"
import { AmmoModule } from "../ammo/ammo_custom.js";

// Firefox limitation: https://github.com/vitejs/vite/issues/4586

function sfc32(seed) {
	let { a, b, c, d } = seed
    return function() {
        a |= 0; b |= 0; c |= 0; d |= 0;
        let t = (a + b) | 0;
        a = b ^ (b >>> 9);
        b = (c + (c << 3)) | 0;
        c = (c << 21 | c >>> 11);
        d = (d + 1) | 0;
        t = (t + d) | 0;
        c = (c + t) | 0;
        return (t >>> 0) / 4294967296;
    };
}

let seededRandom = sfc32({a: 2, b: 3, c: 4, d: 5})

// there's probably a better place for these variables
let bodies = []
let sleepingBodies = []
let colliders = {}
let physicsWorld
let Ammo
let worldWorkerPort
let tmpBtTrans
let createVector3
let width = 150
let height = 150
let aspect = 1
let stopLoop = false
let isInitialized = false;
let simSpeed = 16

const defaultOptions = {
	size: 9.5,
	startingHeight: 8,
	spinForce: 6,
	throwForce: 5,
	gravity: 1,
	mass: 1,
	friction: .8,
	restitution: .1,
	linearDamping: .5,
	angularDamping: .4,
	settleTimeout: 5000,
	// TODO: toss: "center", "edge", "allEdges"
}

let config = {...defaultOptions}

let emptyVector
let diceBufferView

let queueLength = 1; // Set this to your desired queue size
let diceQueue = [];

self.onmessage = (e) => {
  switch (e.data.action) {
    case "rollDie":
      rollDie(e.data.sides)
      break;
    case "init":
      init(e.data).then(()=>{
        self.postMessage({
          action:"init-complete"
        })
      })
      break
    case "clearDice":
			clearDice()
      break
		case "removeDie":
			removeDie(e.data.id)
			break;
		case "resize":
			width = e.data.width
			height = e.data.height
			aspect = width / height
			//addBoxToWorld(config.size, config.startingHeight + 10)
			clearDice();
			break
		case "updateConfig":
			updateConfig(e.data.options)
			break
    case "connect":
		worldWorkerPort = e.ports[0]
		worldWorkerPort.onmessage = (e) => {
        switch (e.data.action) {
			case "initBuffer":
				diceBufferView = new Float32Array(e.data.diceBuffer)
				diceBufferView[0] = -1
				break;
			case "loadModels":
				loadModels(e.data.options)
				break;
          	case "addDie":
				queueLength = e.data.options.queueLength
				queueDie(e.data.options)
            break;

          case "rollDie":
						// TODO: this won't work, need a die object
            rollDie(e.data.id)
            break;
					case "removeDie":
						removeDie(e.data.id)
						break;
          case "stopSimulation":

            stopLoop = true
						
            break;
          case "resumeSimulation":
				seededRandom = sfc32(e.data.seed);
				simSpeed = e.data.simSpeed
				// if(e.data.newStartPoint){
				// 	setStartPosition()
				// }
            break;
					case "stepSimulation":
						diceBufferView = new Float32Array(e.data.diceBuffer)
						loop()
						break;
          default:
            console.error("action not found in physics worker from worldOffscreen worker:", e.data.action)
        }
      }
      break
    default:
      console.error("action not found in physics worker:", e.data.action)
  }
}


const computeGravity = (gravity = defaultOptions.gravity, mass = defaultOptions.mass) => {
	// make gravity a little bit stronger for heavy objects, so they seem heavier
	return gravity === 0 ? 0 : gravity + mass / 3
}

const computeMass = (mass = defaultOptions.mass) => {
	// high values in mass are pretty ineffective, but whole intigers make better config values, so we shave down the value
	// also prevents mass from ever being zero
	return 1 + mass / 3
}

const computeSpin = (spin = defaultOptions.spinForce, spinScale = 40) => {
	// scale down the actual spin value from a nice intiger in config to a fractional value
	return spin/spinScale
}

const computeThrowForce = (throwForce = defaultOptions.throwForce, mass = defaultOptions.mass, scale = defaultOptions.scale) => {
	return throwForce / 2 / mass * (1 + scale / 6)
}

const computeStartingHeight = (height = defaultOptions.startingHeight) => {
	// ensure minimum startingHeight of 1
	return height < 1 ? 1 : height
}



// runs when the worker loads to set up the Ammo physics world and load our colliders
// loaded colliders will be cached and added to the world in a later post message
const init = async (data) => {
	width = data.width
	height = data.height
	aspect = width / height

	config = {...config,...data.options}
	config.gravity = computeGravity(config.gravity, config.mass)
	config.mass = computeMass(config.mass)
	config.spinForce = computeSpin(config.spinForce)
	config.throwForce = computeThrowForce(config.throwForce,config.mass,config.scale)
	config.startingHeight = computeStartingHeight(config.startingHeight)

	const ammoConfig = {
		locateFile: (file) => `${config.origin + config.assetPath}ammo/${file}`
	};
	
	// Initialize Ammo.js
	Ammo = await AmmoModule(ammoConfig);

	
	createVector3 = (x, y, z) => new Ammo.btVector3(x, y, z);
	tmpBtTrans = new Ammo.btTransform()
	emptyVector = createVector3(0,0,0)

	//setStartPosition()
	
	physicsWorld = setupPhysicsWorld()
	clearDice();
}

const updateConfig = (options) => {
	config = {...config, ...options}
	if(options.mass){
		config.mass = computeMass(config.mass)
	}
	if(options.mass || options.gravity) {
		config.gravity = computeGravity(config.gravity, config.mass)
	}
	if(options.spinForce) {
		config.spinForce = computeSpin(config.spinForce)
	}
	if(options.throwForce || options.mass || options.scale){
		config.throwForce = computeThrowForce(config.throwForce, config.mass, config.scale)
	}
	if(options.startingHeight) {
		computeStartingHeight(config.startingHeight)
	}

	clearDice();
	physicsWorld.setGravity(createVector3(0, -9.81 * config.gravity, 0))
	Object.values(colliders).map((collider) => {
		collider.convexHull.setLocalScaling(createVector3(collider.scaling[0] * config.scale, collider.scaling[1] * config.scale, collider.scaling[2] * config.scale))
	})
	clearDice();
}

// options object with colliders and meshName are required
const loadModels = async ({colliders: modelData, meshName}) => {
	// Sort modelData by name or id to ensure consistent ordering
	modelData.sort((a, b) => a.name.localeCompare(b.name));
	let has_d100 = false
	let has_d10 = false

	// turn our model data into convex hull items for the physics world
	modelData.forEach((model,i) => {
		colliders[meshName + '_' + model.name] = model
		colliders[meshName + '_' + model.name].convexHull = createConvexHull(model)
		if (!has_d10) {
			has_d10 = model.id === "d10_collider"
		}
		if (!has_d100) {
			has_d100 = model.id === "d100_collider"
		}
	})
	if (!has_d100 && has_d10) {
		colliders[`${meshName}_d100_collider`] = colliders[`${meshName}_d10_collider`]
	}
}

// const setVector3 = (x,y,z) => {
// 	sharedVector3.setValue(x,y,z)
// 	return sharedVector3
// }

const setStartPosition = () => {
	let size = config.size
	// let envelopeSize = size * .6 / 2
	let edgeOffset = .5
	let xMin = size * aspect / 2 - edgeOffset
	let xMax = size * aspect / -2 + edgeOffset
	let yMin = size / 2 - edgeOffset
	let yMax = size / -2 + edgeOffset
	// let xEnvelope = lerp(envelopeSize * aspect - edgeOffset * aspect, -envelopeSize * aspect + edgeOffset * aspect, seededRandom())
	let xEnvelope = lerp(xMin, xMax, seededRandom())
	let yEnvelope = lerp(yMin, yMax, seededRandom())
	let tossFromTop = Math.round(seededRandom())
	let tossFromLeft = Math.round(seededRandom())
	let tossX = Math.round(seededRandom())
	// console.log(`throw coming from`, tossX ? tossFromTop ? "top" : "bottom" : tossFromLeft ? "left" : "right")

	// forces = {
	// 	xMinForce: tossX ? -config.throwForce * aspect : tossFromLeft ? config.throwForce * aspect * .3 : -config.throwForce * aspect * .3,
	// 	xMaxForce: tossX ? config.throwForce * aspect : tossFromLeft ? config.throwForce * aspect * 1 : -config.throwForce * aspect * 1,
	// 	zMinForce: tossX ? tossFromTop ? config.throwForce * .3 : -config.throwForce * .3 : -config.throwForce,
	// 	zMaxForce: tossX ? tossFromTop ? config.throwForce * 1 : -config.throwForce * 1 : config.throwForce,
	// }

	config.startPosition = [
		// tossing on x axis then z should be locked to top or bottom
		// not tossing on x axis then x should be locked to the left or right
		tossX ? xEnvelope : tossFromLeft ? xMax : xMin,
		config.startingHeight,
		tossX ? tossFromTop ? yMax : yMin : yEnvelope
	]
}

const createConvexHull = (mesh) => {
	const convexMesh = new Ammo.btConvexHullShape()

	let count = mesh.positions.length

	for (let i = 0; i < count; i+=3) {
		let v = createVector3(mesh.positions[i], mesh.positions[i+1], mesh.positions[i+2])
		convexMesh.addPoint(v, true)
	}
	
	convexMesh.setLocalScaling(createVector3(mesh.scaling[0] * config.scale, mesh.scaling[1] * config.scale, mesh.scaling[2] * config.scale))

	return convexMesh
}

const createRigidBody = (collisionShape, params) => {
	// apply params
	const {
		mass = .1,
		collisionFlags = 0,
		// pos = { x: 0, y: 0, z: 0 },
		// quat = { x: 0, y: 0, z: 0, w: 1 }
		pos = [0,0,0],
		// quat = [0,0,0,-1],
		quat = [
			lerp(-1.5, 1.5, seededRandom()),
			lerp(-1.5, 1.5, seededRandom()),
			lerp(-1.5, 1.5, seededRandom()),
			-1
		],
		scale = [1,1,1],
		friction = config.friction,
		restitution = config.restitution
	} = params

	const quatObj = new Ammo.btQuaternion(quat[0], quat[1], quat[2], quat[3])
    quatObj.normalize() // Ensure the quaternion is of unit length
    // Apply position and rotation
    const transform = new Ammo.btTransform()
    transform.setIdentity()
    transform.setOrigin(createVector3(pos[0], pos[1], pos[2]))
    transform.setRotation(quatObj)

	// collisionShape.setLocalScaling(new Ammo.btVector3(1.1, -1.1, 1.1))
	// transform.ScalingToRef()
	// set the scale of the collider
	// collisionShape.setLocalScaling(new Ammo.btVector3(scale[0],scale[1],scale[2]))

	// create the rigid body
	const motionState = new Ammo.btDefaultMotionState(transform)
	const localInertia = createVector3(0, 0, 0)
	if (mass > 0) collisionShape.calculateLocalInertia(mass, localInertia)
	const rbInfo = new Ammo.btRigidBodyConstructionInfo(
		mass,
		motionState,
		collisionShape,
		localInertia
	)
	const rigidBody = new Ammo.btRigidBody(rbInfo)
	
	// rigid body properties
	if (mass > 0) rigidBody.setActivationState(4) // Disable deactivation
	rigidBody.setCollisionFlags(collisionFlags)
	rigidBody.setFriction(friction)
	rigidBody.setRestitution(restitution)
	rigidBody.setDamping(config.linearDamping, config.angularDamping)

	// ad rigid body to physics world
	// physicsWorld.addRigidBody(rigidBody)

	return rigidBody

}
// cache for box parts so it can be removed after a new one has been made
let boxParts = []
const addBoxToWorld = (size, height) => {
	const tempParts = []
	// ground
	const localInertia = createVector3(0, 0, 0);

	const groundTransform = new Ammo.btTransform()
	groundTransform.setIdentity()
	groundTransform.setOrigin(createVector3(0, -.5, 0))
	const groundShape = new Ammo.btBoxShape(createVector3(size * aspect, 1, size))
	const groundMotionState = new Ammo.btDefaultMotionState(groundTransform)
	const groundInfo = new Ammo.btRigidBodyConstructionInfo(0, groundMotionState, groundShape, localInertia)
	const groundBody = new Ammo.btRigidBody(groundInfo)
	groundBody.id='box_bottom'
	groundBody.setFriction(config.friction)
	groundBody.setRestitution(config.restitution)
	physicsWorld.addRigidBody(groundBody)
	tempParts.push(groundBody)

	const ceilingTransform = new Ammo.btTransform()
	ceilingTransform.setIdentity()
	ceilingTransform.setOrigin(createVector3(0, height - .5, 0))
	const ceilingShape = new Ammo.btBoxShape(createVector3(size * aspect, 1, size))
	const ceilingMotionState = new Ammo.btDefaultMotionState(ceilingTransform)
	const ceilingInfo = new Ammo.btRigidBodyConstructionInfo(0, ceilingMotionState, ceilingShape, localInertia)
	const ceilingBody = new Ammo.btRigidBody(ceilingInfo)
	ceilingBody.id='box_top'
	ceilingBody.setFriction(config.friction)
	ceilingBody.setRestitution(config.restitution)
	physicsWorld.addRigidBody(ceilingBody)
	tempParts.push(ceilingBody)

	const wallTopTransform = new Ammo.btTransform()
	wallTopTransform.setIdentity()
	wallTopTransform.setOrigin(createVector3(0, 0, (size/-2) - .5))
	const wallTopShape = new Ammo.btBoxShape(createVector3(size * aspect, height, 1))
	const topMotionState = new Ammo.btDefaultMotionState(wallTopTransform)
	const topInfo = new Ammo.btRigidBodyConstructionInfo(0, topMotionState, wallTopShape, localInertia)
	const topBody = new Ammo.btRigidBody(topInfo)
	topBody.id='box_wall_north'
	topBody.setFriction(config.friction)
	topBody.setRestitution(config.restitution)
	physicsWorld.addRigidBody(topBody)
	tempParts.push(topBody)

	const wallBottomTransform = new Ammo.btTransform()
	wallBottomTransform.setIdentity()
	wallBottomTransform.setOrigin(createVector3(0, 0, (size/2) + .5))
	const wallBottomShape = new Ammo.btBoxShape(createVector3(size * aspect, height, 1))
	const bottomMotionState = new Ammo.btDefaultMotionState(wallBottomTransform)
	const bottomInfo = new Ammo.btRigidBodyConstructionInfo(0, bottomMotionState, wallBottomShape, localInertia)
	const bottomBody = new Ammo.btRigidBody(bottomInfo)
	bottomBody.id='box_wall_south'
	bottomBody.setFriction(config.friction)
	bottomBody.setRestitution(config.restitution)
	physicsWorld.addRigidBody(bottomBody)
	tempParts.push(bottomBody)

	const wallRightTransform = new Ammo.btTransform()
	wallRightTransform.setIdentity()
	wallRightTransform.setOrigin(createVector3((size * aspect / -2) - .5, 0, 0))
	const wallRightShape = new Ammo.btBoxShape(createVector3(1, height, size))
	const rightMotionState = new Ammo.btDefaultMotionState(wallRightTransform)
	const rightInfo = new Ammo.btRigidBodyConstructionInfo(0, rightMotionState, wallRightShape, localInertia)
	const rightBody = new Ammo.btRigidBody(rightInfo)
	rightBody.id='box_wall_east'
	rightBody.setFriction(config.friction)
	rightBody.setRestitution(config.restitution)
	physicsWorld.addRigidBody(rightBody)
	tempParts.push(rightBody)

	const wallLeftTransform = new Ammo.btTransform()
	wallLeftTransform.setIdentity()
	wallLeftTransform.setOrigin(createVector3((size * aspect / 2) + .5, 0, 0))
	const wallLeftShape = new Ammo.btBoxShape(createVector3(1, height, size))
	const leftMotionState = new Ammo.btDefaultMotionState(wallLeftTransform)
	const leftInfo = new Ammo.btRigidBodyConstructionInfo(0, leftMotionState, wallLeftShape, localInertia)
	const leftBody = new Ammo.btRigidBody(leftInfo)
	leftBody.id='box_wall_west'
	leftBody.setFriction(config.friction)
	leftBody.setRestitution(config.restitution)
	physicsWorld.addRigidBody(leftBody)
	tempParts.push(leftBody)

	if(boxParts.length){
		removeBoxFromWorld()
	}
	boxParts = [...tempParts]
}

const removeBoxFromWorld = () => {
	boxParts.forEach(part => physicsWorld.removeRigidBody(part))
}

const queueDie = (options) => {
    diceQueue.push(options);

    // Check if the queue is full
    if (diceQueue.length >= queueLength) {
        // Sort the queue by die id
        diceQueue.sort((a, b) => a.id - b.id);
        // Set isInitialized to true to indicate the queue is ready to be processed
		isInitialized = true;
		physicsWorld.resetLocalTime();
		stopLoop = false
		loop()
    }
};

const addDie = (options) => {
	if(true){ //e.data.options.newStartPoint
		setStartPosition();
	}
	const { sides, id, meshName, scale} = options
	const dieType = Number.isInteger(sides) ? `d${sides}` : sides
	let cType = `${dieType}_collider`
	const comboKey = `${meshName}_${cType}`
	const colliderMass = colliders[comboKey]?.physicsMass || .1
	const mass = colliderMass * config.mass * config.scale // feature? mass should go up with scale, but it throws off the throwForce and spinForce scaling
	// TODO: incorporate colliders physicsFriction and physicsRestitution settings
	// clone the collider
	const newDie = createRigidBody(colliders[comboKey].convexHull, {
		mass,
		scaling: colliders[comboKey].scaling,
		pos: config.startPosition,
		// quat: colliders[cType].rotationQuaternion,
	})
	newDie.id = id
	newDie.timeout = config.settleTimeout
	newDie.mass = mass
	physicsWorld.addRigidBody(newDie)
	bodies.push(newDie)

	return newDie
	// console.log(`added collider for `, type)
	// rollDie(newDie)
}

const rollDie = (die) => {
	// lerp picks a random number between two values
	die.setLinearVelocity(createVector3(
		lerp(-config.startPosition[0] * .5, -config.startPosition[0] * config.throwForce, seededRandom()),
		lerp(-config.startPosition[1], -config.startPosition[1] * 2, seededRandom()), // limit the y force to 2
		lerp(-config.startPosition[2] * .5, -config.startPosition[2] * config.throwForce, seededRandom()),
	))

	const flippy = seededRandom() > .5 ? 1 : -1 // random positive or negative number
	const spinny = lerp(config.spinForce * .5, config.spinForce, seededRandom())
	const force = new Ammo.btVector3(
		spinny * flippy,
		spinny * -flippy, // flip the flippy to avoid gimble lock
		spinny * flippy
	)

	// attempting to create an envelope for the force influence based on scale and mass
	// linear scale was no good - this creates a nice power curve
	const scale = Math.abs(config.scale - 1) + config.scale * config.scale * (die.mass/config.mass) * .75

	// console.log('scale', scale)
	
	die.applyImpulse(force, createVector3(scale, scale, scale))

}

const removeDie = (id) => {
	sleepingBodies = sleepingBodies.filter((die) => {
		let match = die.id === id
		if(match){
			// remove the mesh from the scene
			physicsWorld.removeRigidBody(die)
		}
		return !match
	})

	// step the animation forward
	// requestAnimationFrame(loop)
}

const clearDice = () => {
	//console.log("clearDice--------------------------")
	physicsWorld.clearCollisionCache();
	isInitialized = false;
	diceQueue = []

	if (diceBufferView) {
		if(diceBufferView.byteLength){
			diceBufferView.fill(0)
		}
	}
	
	stopLoop = true
	// clear all bodies
	bodies.forEach(body => physicsWorld.removeRigidBody(body))
	sleepingBodies.forEach(body => physicsWorld.removeRigidBody(body))
	// clear cache arrays
	bodies = []
	sleepingBodies = []

	removeBoxFromWorld();
	boxParts = [];
	addBoxToWorld(config.size, config.startingHeight + 10)
	physicsWorld.resetLocalTime();
	physicsWorld.clearCollisionCache();
}


const setupPhysicsWorld = () => {
	const collisionConfiguration = new Ammo.btDefaultCollisionConfiguration()
	let worldSize = 1000;
	const broadphase = new Ammo.btAxisSweep3(
		createVector3(-worldSize, -worldSize, -worldSize), // Minimum world bounds
		createVector3(worldSize, worldSize, worldSize)     // Maximum world bounds
	);

	const solver = new Ammo.btSequentialImpulseConstraintSolver()

	const dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration)
	const World = new Ammo.btDiscreteDynamicsWorld(
		dispatcher,
		broadphase,
		solver,
		collisionConfiguration
	)

	const solverInfo = World.getSolverInfo();
    // Clear SOLVER_RANDMIZE_ORDER and SOLVER_USE_WARMSTARTING flags
    solverInfo.m_solverMode &= ~Ammo.SOLVER_RANDMIZE_ORDER;
    solverInfo.m_solverMode &= ~Ammo.SOLVER_USE_WARMSTARTING;

	World.setGravity(createVector3(0, -9.81 * config.gravity, 0))

	return World
}

const update = (delta) => {

	// console.log("Current Positions of All Bodies:");
    // bodies.forEach((rb, index) => {
    //     const ms = rb.getMotionState();
    //     if (ms) {
    //         ms.getWorldTransform(tmpBtTrans);
    //         const p = tmpBtTrans.getOrigin();
    //         console.log(`Body ${index} (ID: ${rb.id}) Position: x=${p.x()}, y=${p.y()}, z=${p.z()}`);
    //     } else {
    //         console.log(`Body ${index} (ID: ${rb.id}) has no motion state.`);
    //     }
    // });
	
	// step world
	const deltaTime = delta / 1000
	
	// console.time("stepSimulation")
	//physicsWorld.clearCollisionCache();
	physicsWorld.stepSimulation(deltaTime, 2, deltaTime) // higher number = slow motion
	//physicsWorld.clearCollisionCache();

	// console.timeEnd("stepSimulation")

	diceBufferView[0] = bodies.length

	// Detect collisions
    const numManifolds = physicsWorld.getDispatcher().getNumManifolds();
    for (let i = 0; i < numManifolds; i++) {
        const contactManifold = physicsWorld.getDispatcher().getManifoldByIndexInternal(i);
        const body0 = Ammo.castObject(contactManifold.getBody0(), Ammo.btRigidBody);
        const body1 = Ammo.castObject(contactManifold.getBody1(), Ammo.btRigidBody);

        const rb0Id = body0.id;
        const rb1Id = body1.id;

        let totalForce = 0;

        // Calculate collision force
        const numContacts = contactManifold.getNumContacts();
        for (let j = 0; j < numContacts; j++) {
            const contactPoint = contactManifold.getContactPoint(j);

            // Check if the contact point indicates collision (penetration depth)
            if (contactPoint.getDistance() < 0) {
                // Relative velocity of the two bodies at the contact point
                const normal = contactPoint.get_m_normalWorldOnB();

                const velocity0 = body0.getLinearVelocity();
                const velocity1 = body1.getLinearVelocity();

                // Calculate relative velocity
                const relativeVelocity = new Ammo.btVector3();
                relativeVelocity.setValue(
                    velocity0.x() - velocity1.x(),
                    velocity0.y() - velocity1.y(),
                    velocity0.z() - velocity1.z()
                );

                // Calculate the force (F = m * a) based on velocity and collision normal
                const collisionForce = normal.dot(relativeVelocity);
                totalForce += Math.abs(collisionForce);  // Add to total collision force
							}
						}
						
        if (totalForce > 0) {
            // Send the collision data to the main thread
            self.postMessage({
                action: "collision",
                body0Id: rb0Id,
                body1Id: rb1Id,
                force: totalForce
            });
        }
    }

	// looping backwards since bodies are removed as they are put to sleep
	for (let i = bodies.length - 1; i >= 0; i--) {
		const rb = bodies[i]
		const speed = rb.getLinearVelocity().length()
		const tilt = rb.getAngularVelocity().length()
		//console.log("speed", speed, "tilt", tilt, "rb.timeout", rb.timeout)
		if(speed < .01 && tilt < .005 || rb.timeout < 0) {
			//console.log("zeroing out", rb.id)
			// flag the second param for this body so it can be processed in World, first param will be the roll.id
			diceBufferView[(i*8) + 1] = rb.id
			diceBufferView[(i*8) + 2] = -1
			rb.asleep = true
			rb.setMassProps(0)
			rb.forceActivationState(3)
			// zero out anything left
			rb.setLinearVelocity(emptyVector)
			rb.setAngularVelocity(emptyVector)
			sleepingBodies.push(bodies.splice(i,1)[0])
			continue
		}
		// tick down the movement timeout on this die
		rb.timeout -= delta
		const ms = rb.getMotionState()
		if (ms) {
			ms.getWorldTransform(tmpBtTrans)
			let p = tmpBtTrans.getOrigin()
			let q = tmpBtTrans.getRotation()
			let j = i*8 + 1

			diceBufferView[j] = rb.id
			diceBufferView[j+1] = p.x()
			diceBufferView[j+2] = p.y()
			diceBufferView[j+3] = p.z()
			diceBufferView[j+4] = q.x()
			diceBufferView[j+5] = q.y()
			diceBufferView[j+6] = q.z()
			diceBufferView[j+7] = q.w()
		}
	}
}

const loop = () => {
    const delta = simSpeed;

    if (!stopLoop && diceBufferView.byteLength) {
        // If the queue is initialized and has dice, add them to the world
        if (isInitialized && diceQueue.length > 0) {
            // Dequeue the next die
			const nextOptions = diceQueue.shift();
			if (nextOptions) {
				const nextDie = addDie(nextOptions);
				rollDie(nextDie);
			}
        }

        update(delta);

        worldWorkerPort.postMessage({
            action: 'updates',
            diceBuffer: diceBufferView.buffer
        }, [diceBufferView.buffer]);
    }
};
