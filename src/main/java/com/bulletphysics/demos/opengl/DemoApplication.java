/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package com.bulletphysics.demos.opengl;

import java.awt.Color;
import java.nio.FloatBuffer;

import javax.media.opengl.GL2;
import javax.media.opengl.glu.GLU;
import javax.vecmath.Color3f;
import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import br.com.etyllica.core.event.GUIEvent;
import br.com.etyllica.core.event.KeyEvent;
import br.com.etyllica.core.event.KeyState;
import br.com.etyllica.core.event.PointerEvent;
import br.com.etyllica.core.event.PointerState;
import br.com.etyllica.core.graphics.Graphic;
import br.com.luvia.core.context.ApplicationGL;
import br.com.luvia.core.video.Graphics3D;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.BulletStats;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.Point2PointConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.linearmath.CProfileIterator;
import com.bulletphysics.linearmath.CProfileManager;
import com.bulletphysics.linearmath.Clock;
import com.bulletphysics.linearmath.DebugDrawModes;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;

/**
 *
 * @author jezek2
 */
public abstract class DemoApplication extends ApplicationGL {

	//protected final BulletStack stack = BulletStack.get();

	private static final float STEPSIZE = 5;

	public static int numObjects = 0;
	public static final int maxNumObjects = 16384;
	public static Transform[] startTransforms = new Transform[maxNumObjects];
	public static CollisionShape[] gShapePtr = new CollisionShape[maxNumObjects]; //1 rigidbody has 1 shape (no re-use of shapes)

	public static RigidBody pickedBody = null; // for deactivation state

	private static float mousePickClamping = 3f;

	private boolean needUpdateCamera = false;

	static {
		for (int i=0; i<startTransforms.length; i++) {
			startTransforms[i] = new Transform();
		}
	}
	// TODO: class CProfileIterator* m_profileIterator;

	// JAVA NOTE: added
	//protected IGL gl;

	protected Clock clock = new Clock();

	// this is the most important class
	protected DynamicsWorld dynamicsWorld = null;

	// constraint for mouse picking
	protected TypedConstraint pickConstraint = null;

	protected CollisionShape shootBoxShape = null;

	protected float cameraDistance = 15f;
	protected int debugMode = 0;

	protected float ele = 20f;
	protected float azi = 0f;
	protected final Vector3f cameraPosition = new Vector3f(0f, 0f, 0f);
	protected final Vector3f cameraTargetPosition = new Vector3f(0f, 0f, 0f); // look at

	protected float scaleBottom = 0.5f;
	protected float scaleFactor = 2f;
	protected final Vector3f cameraUp = new Vector3f(0f, 1f, 0f);
	protected int forwardAxis = 2;

	protected int glutScreenWidth = 0;
	protected int glutScreenHeight = 0;

	protected float ShootBoxInitialSpeed = 40f;

	protected boolean stepping = true;
	protected boolean singleStep = false;
	protected boolean idle = false;
	protected int lastKey;

	private CProfileIterator profileIterator;

	public DemoApplication(int w, int h) {
		super(w, h);
		//this.gl = gl;

		BulletStats.setProfileEnabled(true);
		profileIterator = CProfileManager.getIterator();
	}

	public void load() {
		loading = 100;
	}

	@Override
	public void draw(Graphic g) {
		// TODO Auto-generated method stub

	}

	public abstract void initPhysics(Graphics3D g) throws Exception;

	public void destroy() {
		// TODO: CProfileManager::Release_Iterator(m_profileIterator);
		//if (m_shootBoxShape)
		//	delete m_shootBoxShape;
	}

	public void init(Graphics3D g) {

		GL2 gl = g.getGL2();

		FloatBuffer light_ambient = FloatBuffer.wrap(new float[] { 0.2f, 0.2f, 0.2f, 1.0f });
		FloatBuffer light_diffuse = FloatBuffer.wrap(new float[] { 1.0f, 1.0f, 1.0f, 1.0f });
		FloatBuffer light_specular = FloatBuffer.wrap(new float[] { 1.0f, 1.0f, 1.0f, 1.0f });
		/* light_position is NOT default value */
		FloatBuffer light_position0 = FloatBuffer.wrap(new float[] { 1.0f, 10.0f, 1.0f, 0.0f });
		FloatBuffer light_position1 = FloatBuffer.wrap(new float[] { -1.0f, -10.0f, -1.0f, 0.0f });

		gl.glLightfv(GL2.GL_LIGHT0, GL2.GL_AMBIENT, light_ambient);
		gl.glLightfv(GL2.GL_LIGHT0, GL2.GL_DIFFUSE, light_diffuse);
		gl.glLightfv(GL2.GL_LIGHT0, GL2.GL_SPECULAR, light_specular);
		gl.glLightfv(GL2.GL_LIGHT0, GL2.GL_POSITION, light_position0);

		gl.glLightfv(GL2.GL_LIGHT1, GL2.GL_AMBIENT, light_ambient);
		gl.glLightfv(GL2.GL_LIGHT1, GL2.GL_DIFFUSE, light_diffuse);
		gl.glLightfv(GL2.GL_LIGHT1, GL2.GL_SPECULAR, light_specular);
		gl.glLightfv(GL2.GL_LIGHT1, GL2.GL_POSITION, light_position1);

		gl.glEnable(GL2.GL_LIGHTING);
		gl.glEnable(GL2.GL_LIGHT0);
		gl.glEnable(GL2.GL_LIGHT1);

		gl.glShadeModel(GL2.GL_SMOOTH);
		gl.glEnable(GL2.GL_DEPTH_TEST);
		gl.glDepthFunc(GL2.GL_LESS);

		gl.glClearColor(0.7f, 0.7f, 0.7f, 0f);

		try {
			initPhysics(g);
			//Add debugger
			getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(g));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		//glEnable(GL_CULL_FACE);
		//glCullFace(GL_BACK);
	}

	public void setCameraDistance(float dist) {
		cameraDistance = dist;
	}

	public float getCameraDistance() {
		return cameraDistance;
	}

	public void toggleIdle() {
		if (idle) {
			idle = false;
		} else {
			idle = true;
		}
	}

	public void updateCamera(Graphics3D g) {
		/*if(!needUpdateCamera) {
			return; 
		} else {
			needUpdateCamera = false;
		}*/

		GL2 gl = g.getGL2();
		GLU glu = g.getGLU();

		gl.glMatrixMode(GL2.GL_PROJECTION);
		gl.glLoadIdentity();
		float rele = ele * 0.01745329251994329547f; // rads per deg
		float razi = azi * 0.01745329251994329547f; // rads per deg

		Quat4f rot = new Quat4f();
		QuaternionUtil.setRotation(rot, cameraUp, razi);

		Vector3f eyePos = new Vector3f();
		eyePos.set(0f, 0f, 0f);
		VectorUtil.setCoord(eyePos, forwardAxis, -cameraDistance);

		Vector3f forward = new Vector3f();
		forward.set(eyePos.x, eyePos.y, eyePos.z);
		if (forward.lengthSquared() < BulletGlobals.FLT_EPSILON) {
			forward.set(1f, 0f, 0f);
		}
		Vector3f right = new Vector3f();
		right.cross(cameraUp, forward);
		Quat4f roll = new Quat4f();
		QuaternionUtil.setRotation(roll, right, -rele);

		Matrix3f tmpMat1 = new Matrix3f();
		Matrix3f tmpMat2 = new Matrix3f();
		tmpMat1.set(rot);
		tmpMat2.set(roll);
		tmpMat1.mul(tmpMat2);
		tmpMat1.transform(eyePos);

		cameraPosition.set(eyePos);

		if (glutScreenWidth > glutScreenHeight) {
			float aspect = glutScreenWidth / (float) glutScreenHeight;
			gl.glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
		}
		else {
			float aspect = glutScreenHeight / (float) glutScreenWidth;
			gl.glFrustum(-1.0, 1.0, -aspect, aspect, 1.0, 10000.0);
		}

		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity();
		glu.gluLookAt(cameraPosition.x, cameraPosition.y, cameraPosition.z,
				cameraTargetPosition.x, cameraTargetPosition.y, cameraTargetPosition.z,
				cameraUp.x, cameraUp.y, cameraUp.z);
	}

	public void stepLeft() {
		azi -= STEPSIZE;
		if (azi < 0) {
			azi += 360;
		}
		needUpdateCamera = true;
	}

	public void stepRight() {
		azi += STEPSIZE;
		if (azi >= 360) {
			azi -= 360;
		}
		needUpdateCamera = true;
	}

	public void stepFront() {
		ele += STEPSIZE;
		if (ele >= 360) {
			ele -= 360;
		}
		needUpdateCamera = true;
	}

	public void stepBack() {
		ele -= STEPSIZE;
		if (ele < 0) {
			ele += 360;
		}
		needUpdateCamera = true;
	}

	public void zoomIn() {
		cameraDistance -= 0.4f;
		needUpdateCamera = true;
		if (cameraDistance < 0.1f) {
			cameraDistance = 0.1f;
		}
	}

	public void zoomOut() {
		cameraDistance += 0.4f;
		needUpdateCamera = true;
	}

	public void reshape(Graphics3D g, int x, int y, int width, int height) {
		glutScreenWidth = w;
		glutScreenHeight = h;

		g.getGL2().glViewport(0, 0, w, h);
		needUpdateCamera = true;
	}

	public void keyboardCallback(char key, int x, int y, int modifiers) {
		lastKey = 0;

		if (key >= 0x31 && key < 0x37) {
			int child = key - 0x31;
			profileIterator.enterChild(child);
		}
		if (key == 0x30) {
			profileIterator.enterParent();
		}

		switch (key) {
		case 'l':
			stepLeft();
			break;
		case 'r':
			stepRight();
			break;
		case 'f':
			stepFront();
			break;
		case 'b':
			stepBack();
			break;
		case 'z':
			zoomIn();
			break;
		case 'x':
			zoomOut();
			break;
		case 'i':
			toggleIdle();
			break;
		case 'h':
			if ((debugMode & DebugDrawModes.NO_HELP_TEXT) != 0) {
				debugMode = debugMode & (~DebugDrawModes.NO_HELP_TEXT);
			} else {
				debugMode |= DebugDrawModes.NO_HELP_TEXT;
			}
			break;

		case 'w':
			if ((debugMode & DebugDrawModes.DRAW_WIREFRAME) != 0) {
				debugMode = debugMode & (~DebugDrawModes.DRAW_WIREFRAME);
			} else {
				debugMode |= DebugDrawModes.DRAW_WIREFRAME;
			}
			break;

		case 'p':
			if ((debugMode & DebugDrawModes.PROFILE_TIMINGS) != 0) {
				debugMode = debugMode & (~DebugDrawModes.PROFILE_TIMINGS);
			} else {
				debugMode |= DebugDrawModes.PROFILE_TIMINGS;
			}
			break;

		case 'm':
			if ((debugMode & DebugDrawModes.ENABLE_SAT_COMPARISON) != 0) {
				debugMode = debugMode & (~DebugDrawModes.ENABLE_SAT_COMPARISON);
			} else {
				debugMode |= DebugDrawModes.ENABLE_SAT_COMPARISON;
			}
			break;

		case 'n':
			if ((debugMode & DebugDrawModes.DISABLE_BULLET_LCP) != 0) {
				debugMode = debugMode & (~DebugDrawModes.DISABLE_BULLET_LCP);
			} else {
				debugMode |= DebugDrawModes.DISABLE_BULLET_LCP;
			}
			break;

		case 't':
			if ((debugMode & DebugDrawModes.DRAW_TEXT) != 0) {
				debugMode = debugMode & (~DebugDrawModes.DRAW_TEXT);
			} else {
				debugMode |= DebugDrawModes.DRAW_TEXT;
			}
			break;
		case 'y':
			if ((debugMode & DebugDrawModes.DRAW_FEATURES_TEXT) != 0) {
				debugMode = debugMode & (~DebugDrawModes.DRAW_FEATURES_TEXT);
			} else {
				debugMode |= DebugDrawModes.DRAW_FEATURES_TEXT;
			}
			break;
		case 'a':
			if ((debugMode & DebugDrawModes.DRAW_AABB) != 0) {
				debugMode = debugMode & (~DebugDrawModes.DRAW_AABB);
			} else {
				debugMode |= DebugDrawModes.DRAW_AABB;
			}
			break;
		case 'c':
			if ((debugMode & DebugDrawModes.DRAW_CONTACT_POINTS) != 0) {
				debugMode = debugMode & (~DebugDrawModes.DRAW_CONTACT_POINTS);
			} else {
				debugMode |= DebugDrawModes.DRAW_CONTACT_POINTS;
			}
			break;

		case 'd':
			if ((debugMode & DebugDrawModes.NO_DEACTIVATION) != 0) {
				debugMode = debugMode & (~DebugDrawModes.NO_DEACTIVATION);
			} else {
				debugMode |= DebugDrawModes.NO_DEACTIVATION;
			}
			if ((debugMode & DebugDrawModes.NO_DEACTIVATION) != 0) {
				BulletGlobals.setDeactivationDisabled(true);
			} else {
				BulletGlobals.setDeactivationDisabled(false);
			}
			break;

		case 'o': {
			stepping = !stepping;
			break;
		}
		case 's':
			//clientMoveAndDisplay();
			break;
			//    case ' ' : newRandom(); break;
		case ' ':
			clientResetScene();
			break;
		case '1': {
			if ((debugMode & DebugDrawModes.ENABLE_CCD) != 0) {
				debugMode = debugMode & (~DebugDrawModes.ENABLE_CCD);
			} else {
				debugMode |= DebugDrawModes.ENABLE_CCD;
			}
			break;
		}

		case '.': {
			shootBox(getCameraTargetPosition());
			break;
		}

		case '+': {
			ShootBoxInitialSpeed += 10f;
			break;
		}
		case '-': {
			ShootBoxInitialSpeed -= 10f;
			break;
		}

		default:
			// std::cout << "unused key : " << key << std::endl;
			break;
		}

		if (getDynamicsWorld() != null && getDynamicsWorld().getDebugDrawer() != null) {
			getDynamicsWorld().getDebugDrawer().setDebugMode(debugMode);
		}

		//LWJGL.postRedisplay();
	}

	public int getDebugMode() {
		return debugMode;
	}

	public void setDebugMode(int mode) {
		debugMode = mode;
		if (getDynamicsWorld() != null && getDynamicsWorld().getDebugDrawer() != null) {
			getDynamicsWorld().getDebugDrawer().setDebugMode(mode);
		}
	}

	//Based on LWJGL methods
	public void specialKeyboard(int key, int x, int y, int modifiers) {

	}
	
	public void specialKeyboardUp(int key, int x, int y, int modifiers) {

	}

	@Override
	public GUIEvent updateKeyboard(KeyEvent event) {

		if(event.getState() == KeyState.PRESSED) {

			switch(event.getKey()) {
			case KeyEvent.TSK_F1:
				break;
			case KeyEvent.TSK_F2:
				break;
			case KeyEvent.TSK_END:
				int numObj = getDynamicsWorld().getNumCollisionObjects();
				if (numObj != 0) {
					CollisionObject obj = getDynamicsWorld().getCollisionObjectArray().getQuick(numObj - 1);

					getDynamicsWorld().removeCollisionObject(obj);
					RigidBody body = RigidBody.upcast(obj);
					if (body != null && body.getMotionState() != null) {
						//delete body->getMotionState();
					}
					//delete obj;
				}
				break;
			case KeyEvent.TSK_LEFT_ARROW:
				stepLeft();
				break;
			case KeyEvent.TSK_RIGHT_ARROW:
				stepRight();
				break;
			case KeyEvent.TSK_UP_ARROW:
				stepFront();
				break;
			case KeyEvent.TSK_DOWN_ARROW:
				stepBack();
				break;
			case KeyEvent.TSK_PAGE_UP:
				zoomIn();
				break;
			case KeyEvent.TSK_PAGE_DOWN:
				zoomOut();
				break;
			case KeyEvent.TSK_HOME:
				toggleIdle();
				break;

			case KeyEvent.TSK_H:
				if ((debugMode & DebugDrawModes.NO_HELP_TEXT) != 0) {
					debugMode = debugMode & (~DebugDrawModes.NO_HELP_TEXT);

				} else {
					debugMode |= DebugDrawModes.NO_HELP_TEXT;
				}
				break;
			}
			
			specialKeyboard(event.getKey(),0,0,0);

		} else if(event.getState() == KeyState.RELEASED) {
			specialKeyboardUp(event.getKey(),0,0,0);
		}

		return GUIEvent.NONE;
	}

	private void moveAndDisplay(Graphics3D g) {

	}

	public void displayCallback() {
	}

	public void shootBox(Vector3f destination) {
		if (dynamicsWorld != null) {
			float mass = 10f;
			Transform startTransform = new Transform();
			startTransform.setIdentity();
			Vector3f camPos = new Vector3f(getCameraPosition());
			startTransform.origin.set(camPos);

			if (shootBoxShape == null) {
				//#define TEST_UNIFORM_SCALING_SHAPE 1
				//#ifdef TEST_UNIFORM_SCALING_SHAPE
				//btConvexShape* childShape = new btBoxShape(btVector3(1.f,1.f,1.f));
				//m_shootBoxShape = new btUniformScalingShape(childShape,0.5f);
				//#else
				shootBoxShape = new BoxShape(new Vector3f(1f, 1f, 1f));
				//#endif//
			}

			RigidBody body = this.localCreateRigidBody(mass, startTransform, shootBoxShape);

			Vector3f linVel = new Vector3f(destination.x - camPos.x, destination.y - camPos.y, destination.z - camPos.z);
			linVel.normalize();
			linVel.scale(ShootBoxInitialSpeed);

			Transform worldTrans = body.getWorldTransform(new Transform());
			worldTrans.origin.set(camPos);
			worldTrans.setRotation(new Quat4f(0f, 0f, 0f, 1f));
			body.setWorldTransform(worldTrans);

			body.setLinearVelocity(linVel);
			body.setAngularVelocity(new Vector3f(0f, 0f, 0f));

			body.setCcdMotionThreshold(1f);
			body.setCcdSweptSphereRadius(0.2f);
		}
	}

	public Vector3f getRayTo(int x, int y) {
		float top = 1f;
		float bottom = -1f;
		float nearPlane = 1f;
		float tanFov = (top - bottom) * 0.5f / nearPlane;
		float fov = 2f * (float) Math.atan(tanFov);

		Vector3f rayFrom = new Vector3f(getCameraPosition());
		Vector3f rayForward = new Vector3f();
		rayForward.sub(getCameraTargetPosition(), getCameraPosition());
		rayForward.normalize();
		float farPlane = 10000f;
		rayForward.scale(farPlane);

		Vector3f rightOffset = new Vector3f();
		Vector3f vertical = new Vector3f(cameraUp);

		Vector3f hor = new Vector3f();
		// TODO: check: hor = rayForward.cross(vertical);
		hor.cross(rayForward, vertical);
		hor.normalize();
		// TODO: check: vertical = hor.cross(rayForward);
		vertical.cross(hor, rayForward);
		vertical.normalize();

		float tanfov = (float) Math.tan(0.5f * fov);

		float aspect = glutScreenHeight / (float)glutScreenWidth;

		hor.scale(2f * farPlane * tanfov);
		vertical.scale(2f * farPlane * tanfov);

		if (aspect < 1f) {
			hor.scale(1f / aspect);
		}
		else {
			vertical.scale(aspect);
		}

		Vector3f rayToCenter = new Vector3f();
		rayToCenter.add(rayFrom, rayForward);
		Vector3f dHor = new Vector3f(hor);
		dHor.scale(1f / (float) glutScreenWidth);
		Vector3f dVert = new Vector3f(vertical);
		dVert.scale(1.f / (float) glutScreenHeight);

		Vector3f tmp1 = new Vector3f();
		Vector3f tmp2 = new Vector3f();
		tmp1.scale(0.5f, hor);
		tmp2.scale(0.5f, vertical);

		Vector3f rayTo = new Vector3f();
		rayTo.sub(rayToCenter, tmp1);
		rayTo.add(tmp2);

		tmp1.scale(x, dHor);
		tmp2.scale(y, dVert);

		rayTo.add(tmp1);
		rayTo.sub(tmp2);
		return rayTo;
	}

	@Override
	public GUIEvent updateMouse(PointerEvent event) {

		int x = event.getX();
		int y = event.getY();

		Vector3f rayTo = new Vector3f(getRayTo(x, y));

		switch (event.getKey()) {
		case MOUSE_BUTTON_RIGHT: {
			if (event.getState() == PointerState.PRESSED) {
				shootBox(rayTo);
			}
			break;
		}
		case MOUSE_BUTTON_MIDDLE: {
			if (event.getState() == PointerState.PRESSED) {
				// apply an impulse
				if (dynamicsWorld != null) {
					CollisionWorld.ClosestRayResultCallback rayCallback = new CollisionWorld.ClosestRayResultCallback(cameraPosition, rayTo);
					dynamicsWorld.rayTest(cameraPosition, rayTo, rayCallback);
					if (rayCallback.hasHit()) {
						RigidBody body = RigidBody.upcast(rayCallback.collisionObject);
						if (body != null) {
							body.setActivationState(CollisionObject.ACTIVE_TAG);
							Vector3f impulse = new Vector3f(rayTo);
							impulse.normalize();
							float impulseStrength = 10f;
							impulse.scale(impulseStrength);
							Vector3f relPos = new Vector3f();
							relPos.sub(rayCallback.hitPointWorld, body.getCenterOfMassPosition(new Vector3f()));
							body.applyImpulse(impulse, relPos);
						}
					}
				}
			} else {
			}
			break;
		}
		case MOUSE_BUTTON_LEFT: {
			if (event.getState() == PointerState.PRESSED) {
				// add a point to point constraint for picking
				if (dynamicsWorld != null) {
					CollisionWorld.ClosestRayResultCallback rayCallback = new CollisionWorld.ClosestRayResultCallback(cameraPosition, rayTo);
					dynamicsWorld.rayTest(cameraPosition, rayTo, rayCallback);
					if (rayCallback.hasHit()) {
						RigidBody body = RigidBody.upcast(rayCallback.collisionObject);
						if (body != null) {
							// other exclusions?
							if (!(body.isStaticObject() || body.isKinematicObject())) {
								pickedBody = body;
								pickedBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);

								Vector3f pickPos = new Vector3f(rayCallback.hitPointWorld);

								Transform tmpTrans = body.getCenterOfMassTransform(new Transform());
								tmpTrans.inverse();
								Vector3f localPivot = new Vector3f(pickPos);
								tmpTrans.transform(localPivot);

								Point2PointConstraint p2p = new Point2PointConstraint(body, localPivot);
								p2p.setting.impulseClamp = mousePickClamping;

								dynamicsWorld.addConstraint(p2p);
								pickConstraint = p2p;
								// save mouse position for dragging
								BulletStats.gOldPickingPos.set(rayTo);
								Vector3f eyePos = new Vector3f(cameraPosition);
								Vector3f tmp = new Vector3f();
								tmp.sub(pickPos, eyePos);
								BulletStats.gOldPickingDist = tmp.length();
								// very weak constraint for picking
								p2p.setting.tau = 0.1f;
							}
						}
					}
				}

			} else {

				if (pickConstraint != null && dynamicsWorld != null) {
					dynamicsWorld.removeConstraint(pickConstraint);
					// delete m_pickConstraint;
					//printf("removed constraint %i",gPickingConstraintId);
					pickConstraint = null;
					pickedBody.forceActivationState(CollisionObject.ACTIVE_TAG);
					pickedBody.setDeactivationTime(0f);
					pickedBody = null;
				}
			}
			break;
		}
		default: {
		}
		}

		return GUIEvent.NONE;
	}

	public void mouseMotionFunc(int x, int y) {
		if (pickConstraint != null) {
			// move the constraint pivot
			Point2PointConstraint p2p = (Point2PointConstraint) pickConstraint;
			if (p2p != null) {
				// keep it at the same picking distance

				Vector3f newRayTo = new Vector3f(getRayTo(x, y));
				Vector3f eyePos = new Vector3f(cameraPosition);
				Vector3f dir = new Vector3f();
				dir.sub(newRayTo, eyePos);
				dir.normalize();
				dir.scale(BulletStats.gOldPickingDist);

				Vector3f newPos = new Vector3f();
				newPos.add(eyePos, dir);
				p2p.setPivotB(newPos);
			}
		}
	}

	public RigidBody localCreateRigidBody(float mass, Transform startTransform, CollisionShape shape) {
		// rigidbody is dynamic if and only if mass is non zero, otherwise static
		boolean isDynamic = (mass != 0f);

		Vector3f localInertia = new Vector3f(0f, 0f, 0f);
		if (isDynamic) {
			shape.calculateLocalInertia(mass, localInertia);
		}

		// using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		//#define USE_MOTIONSTATE 1
		//#ifdef USE_MOTIONSTATE
		DefaultMotionState myMotionState = new DefaultMotionState(startTransform);

		RigidBodyConstructionInfo cInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);

		RigidBody body = new RigidBody(cInfo);
		//#else
		//btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
		//body->setWorldTransform(startTransform);
		//#endif//

		dynamicsWorld.addRigidBody(body);

		return body;
	}

	// See http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
	public void setOrthographicProjection(Graphics3D g) {
		GL2 gl = g.getGL2();
		GLU glu = g.getGLU();

		// switch to projection mode
		gl.glMatrixMode(GL2.GL_PROJECTION);

		// save previous matrix which contains the 
		//settings for the perspective projection
		gl.glPushMatrix();
		// reset matrix
		gl.glLoadIdentity();
		// set a 2D orthographic projection
		glu.gluOrtho2D(0f, glutScreenWidth, 0f, glutScreenHeight);
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity();

		// invert the y axis, down is positive
		gl.glScalef(1f, -1f, 1f);
		// mover the origin from the bottom left corner
		// to the upper left corner
		gl.glTranslatef(0f, -glutScreenHeight, 0f);
	}

	public void resetPerspectiveProjection(Graphics3D g) {
		GL2 gl = g.getGL2();

		gl.glMatrixMode(GL2.GL_PROJECTION);
		gl.glPopMatrix();
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		needUpdateCamera = true;
	}

	private void displayProfileString(Graphics3D g, float xOffset, float yStart, CharSequence message) {
		drawString(g, message, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
	}	

	private static double time_since_reset = 0f;

	protected float showProfileInfo(Graphics3D g, float xOffset, float yStart, float yIncr) {
		if (!idle) {
			time_since_reset = CProfileManager.getTimeSinceReset();
		}

		{
			// recompute profiling data, and store profile strings

			double totalTime = 0;

			int frames_since_reset = CProfileManager.getFrameCountSinceReset();

			profileIterator.first();

			double parent_time = profileIterator.isRoot() ? time_since_reset : profileIterator.getCurrentParentTotalTime();

			{
				buf.setLength(0);
				buf.append("--- Profiling: ");
				buf.append(profileIterator.getCurrentParentName());
				buf.append(" (total running time: ");
				FastFormat.append(buf, (float) parent_time, 3);
				buf.append(" ms) ---");
				displayProfileString(g, xOffset, yStart, buf);
				yStart += yIncr;
				String s = "press number (1,2...) to display child timings, or 0 to go up to parent";
				displayProfileString(g, xOffset, yStart, s);
				yStart += yIncr;
			}

			double accumulated_time = 0.f;

			for (int i = 0; !profileIterator.isDone(); profileIterator.next()) {
				double current_total_time = profileIterator.getCurrentTotalTime();
				accumulated_time += current_total_time;
				double fraction = parent_time > BulletGlobals.FLT_EPSILON ? (current_total_time / parent_time) * 100 : 0f;

				buf.setLength(0);
				FastFormat.append(buf, ++i);
				buf.append(" -- ");
				buf.append(profileIterator.getCurrentName());
				buf.append(" (");
				FastFormat.append(buf, (float) fraction, 2);
				buf.append(" %) :: ");
				FastFormat.append(buf, (float) (current_total_time / (double) frames_since_reset), 3);
				buf.append(" ms / frame (");
				FastFormat.append(buf, profileIterator.getCurrentTotalCalls());
				buf.append(" calls)");

				displayProfileString(g, xOffset, yStart, buf);
				yStart += yIncr;
				totalTime += current_total_time;
			}

			buf.setLength(0);
			buf.append("Unaccounted (");
			FastFormat.append(buf, (float) (parent_time > BulletGlobals.FLT_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f), 3);
			buf.append(" %) :: ");
			FastFormat.append(buf, (float) (parent_time - accumulated_time), 3);
			buf.append(" ms");

			displayProfileString(g, xOffset, yStart, buf);
			yStart += yIncr;

			String s = "-------------------------------------------------";
			displayProfileString(g, xOffset, yStart, s);
			yStart += yIncr;

		}

		return yStart;
	}

	private final Transform m = new Transform();
	private Vector3f wireColor = new Vector3f();
	protected Color3f TEXT_COLOR = new Color3f(0f, 0f, 0f);
	private StringBuilder buf = new StringBuilder();

	public void display(Graphics3D g) {
		updateCamera(g);

		g.getGL2().glClear(GL2.GL_COLOR_BUFFER_BIT | GL2.GL_DEPTH_BUFFER_BIT);

		if (!idle) {
			clientMoveAndDisplay();
		}

		GL2 gl = g.getGL2();

		if (dynamicsWorld != null) {
			int numObjects = dynamicsWorld.getNumCollisionObjects();
			wireColor.set(1f, 0f, 0f);
			for (int i = 0; i < numObjects; i++) {
				CollisionObject colObj = dynamicsWorld.getCollisionObjectArray().getQuick(i);
				RigidBody body = RigidBody.upcast(colObj);

				if (body != null && body.getMotionState() != null) {
					DefaultMotionState myMotionState = (DefaultMotionState) body.getMotionState();
					m.set(myMotionState.graphicsWorldTrans);
				}
				else {
					colObj.getWorldTransform(m);
				}

				wireColor.set(1f, 1f, 0.5f); // wants deactivation
				if ((i & 1) != 0) {
					wireColor.set(0f, 0f, 1f);
				}

				// color differently for active, sleeping, wantsdeactivation states
				if (colObj.getActivationState() == 1) // active
				{
					if ((i & 1) != 0) {
						//wireColor.add(new Vector3f(1f, 0f, 0f));
						wireColor.x += 1f;
					}
					else {
						//wireColor.add(new Vector3f(0.5f, 0f, 0f));
						wireColor.x += 0.5f;
					}
				}
				if (colObj.getActivationState() == 2) // ISLAND_SLEEPING
				{
					if ((i & 1) != 0) {
						//wireColor.add(new Vector3f(0f, 1f, 0f));
						wireColor.y += 1f;
					}
					else {
						//wireColor.add(new Vector3f(0f, 0.5f, 0f));
						wireColor.y += 0.5f;
					}
				}

				GLShapeDrawer.drawOpenGL(g, m, colObj.getCollisionShape(), wireColor, getDebugMode());
			}

			float xOffset = 10f;
			float yStart = 20f;
			float yIncr = 20f;

			gl.glDisable(GL2.GL_LIGHTING);
			gl.glColor3f(0f, 0f, 0f);

			if ((debugMode & DebugDrawModes.NO_HELP_TEXT) == 0) {
				setOrthographicProjection(g);

				yStart = showProfileInfo(g, xOffset, yStart, yIncr);

				//					#ifdef USE_QUICKPROF
				//					if ( getDebugMode() & btIDebugDraw::DBG_ProfileTimings)
				//					{
				//						static int counter = 0;
				//						counter++;
				//						std::map<std::string, hidden::ProfileBlock*>::iterator iter;
				//						for (iter = btProfiler::mProfileBlocks.begin(); iter != btProfiler::mProfileBlocks.end(); ++iter)
				//						{
				//							char blockTime[128];
				//							sprintf(blockTime, "%s: %lf",&((*iter).first[0]),btProfiler::getBlockTime((*iter).first, btProfiler::BLOCK_CYCLE_SECONDS));//BLOCK_TOTAL_PERCENT));
				//							glRasterPos3f(xOffset,yStart,0);
				//							BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),blockTime);
				//							yStart += yIncr;
				//
				//						}
				//					}
				//					#endif //USE_QUICKPROF


				String s = "mouse to interact";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				// JAVA NOTE: added
				s = "LMB=drag, RMB=shoot box, MIDDLE=apply impulse";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "space to reset";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "cursor keys and z,x to navigate";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "i to toggle simulation, s single step";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "q to quit";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = ". to shoot box or trimesh (MovingConcaveDemo)";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				// not yet hooked up again after refactoring...

				s = "d to toggle deactivation";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "g to toggle mesh animation (ConcaveDemo)";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				// JAVA NOTE: added
				s = "e to spawn new body (GenericJointDemo)";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				s = "h to toggle help text";
				drawString(g, s, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				//buf = "p to toggle profiling (+results to file)";
				//drawString(buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				//bool useBulletLCP = !(getDebugMode() & btIDebugDraw::DBG_DisableBulletLCP);
				//bool useCCD = (getDebugMode() & btIDebugDraw::DBG_EnableCCD);
				//glRasterPos3f(xOffset,yStart,0);
				//sprintf(buf,"1 CCD mode (adhoc) = %i",useCCD);
				//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				//yStart += yIncr;

				//glRasterPos3f(xOffset, yStart, 0);
				//buf = String.format(%10.2f", ShootBoxInitialSpeed);
				buf.setLength(0);
				buf.append("+- shooting speed = ");
				FastFormat.append(buf, ShootBoxInitialSpeed);
				drawString(g, buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				//#ifdef SHOW_NUM_DEEP_PENETRATIONS
				buf.setLength(0);
				buf.append("gNumDeepPenetrationChecks = ");
				FastFormat.append(buf, BulletStats.gNumDeepPenetrationChecks);
				drawString(g, buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				buf.setLength(0);
				buf.append("gNumGjkChecks = ");
				FastFormat.append(buf, BulletStats.gNumGjkChecks);
				drawString(g, buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				buf.setLength(0);
				buf.append("gNumSplitImpulseRecoveries = ");
				FastFormat.append(buf, BulletStats.gNumSplitImpulseRecoveries);
				drawString(g, buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				//buf = String.format("gNumAlignedAllocs = %d", BulletGlobals.gNumAlignedAllocs);
				// TODO: BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				//yStart += yIncr;

				//buf = String.format("gNumAlignedFree= %d", BulletGlobals.gNumAlignedFree);
				// TODO: BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				//yStart += yIncr;

				//buf = String.format("# alloc-free = %d", BulletGlobals.gNumAlignedAllocs - BulletGlobals.gNumAlignedFree);
				// TODO: BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				//yStart += yIncr;

				//enable BT_DEBUG_MEMORY_ALLOCATIONS define in Bullet/src/LinearMath/btAlignedAllocator.h for memory leak detection
				//#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
				//glRasterPos3f(xOffset,yStart,0);
				//sprintf(buf,"gTotalBytesAlignedAllocs = %d",gTotalBytesAlignedAllocs);
				//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				//yStart += yIncr;
				//#endif //BT_DEBUG_MEMORY_ALLOCATIONS

				if (getDynamicsWorld() != null) {
					buf.setLength(0);
					buf.append("# objects = ");
					FastFormat.append(buf, getDynamicsWorld().getNumCollisionObjects());
					drawString(g, buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
					yStart += yIncr;

					buf.setLength(0);
					buf.append("# pairs = ");
					FastFormat.append(buf, getDynamicsWorld().getBroadphase().getOverlappingPairCache().getNumOverlappingPairs());
					drawString(g, buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
					yStart += yIncr;

				}
				//#endif //SHOW_NUM_DEEP_PENETRATIONS

				// JAVA NOTE: added
				int free = (int)Runtime.getRuntime().freeMemory();
				int total = (int)Runtime.getRuntime().totalMemory();
				buf.setLength(0);
				buf.append("heap = ");
				FastFormat.append(buf, (float)(total - free) / (1024*1024));
				buf.append(" / ");
				FastFormat.append(buf, (float)(total) / (1024*1024));
				buf.append(" MB");
				drawString(g, buf, Math.round(xOffset), Math.round(yStart), TEXT_COLOR);
				yStart += yIncr;

				resetPerspectiveProjection(g);
			}

			gl.glEnable(GL2.GL_LIGHTING);
		}

		needUpdateCamera = true;
	}

	public void clientResetScene() {
		//#ifdef SHOW_NUM_DEEP_PENETRATIONS
		BulletStats.gNumDeepPenetrationChecks = 0;
		BulletStats.gNumGjkChecks = 0;
		//#endif //SHOW_NUM_DEEP_PENETRATIONS

		int numObjects = 0;
		if (dynamicsWorld != null) {
			dynamicsWorld.stepSimulation(1f / 60f, 0);
			numObjects = dynamicsWorld.getNumCollisionObjects();
		}

		for (int i = 0; i < numObjects; i++) {
			CollisionObject colObj = dynamicsWorld.getCollisionObjectArray().getQuick(i);
			RigidBody body = RigidBody.upcast(colObj);
			if (body != null) {
				if (body.getMotionState() != null) {
					DefaultMotionState myMotionState = (DefaultMotionState) body.getMotionState();
					myMotionState.graphicsWorldTrans.set(myMotionState.startWorldTrans);
					colObj.setWorldTransform(myMotionState.graphicsWorldTrans);
					colObj.setInterpolationWorldTransform(myMotionState.startWorldTrans);
					colObj.activate();
				}
				// removed cached contact points
				dynamicsWorld.getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(colObj.getBroadphaseHandle(), getDynamicsWorld().getDispatcher());

				body = RigidBody.upcast(colObj);
				if (body != null && !body.isStaticObject()) {
					RigidBody.upcast(colObj).setLinearVelocity(new Vector3f(0f, 0f, 0f));
					RigidBody.upcast(colObj).setAngularVelocity(new Vector3f(0f, 0f, 0f));
				}
			}

			/*
			//quickly search some issue at a certain simulation frame, pressing space to reset
			int fixed=18;
			for (int i=0;i<fixed;i++)
			{
			getDynamicsWorld()->stepSimulation(1./60.f,1);
			}
			 */
		}
	}

	public DynamicsWorld getDynamicsWorld() {
		return dynamicsWorld;
	}

	public void setCameraUp(Vector3f camUp) {
		cameraUp.set(camUp);
	}

	public void setCameraForwardAxis(int axis) {
		forwardAxis = axis;
	}

	public Vector3f getCameraPosition() {
		return cameraPosition;
	}

	public Vector3f getCameraTargetPosition() {
		return cameraTargetPosition;
	}

	public float getDeltaTimeMicroseconds() {
		//#ifdef USE_BT_CLOCK
		float dt = clock.getTimeMicroseconds();
		clock.reset();
		return dt;
		//#else
		//return btScalar(16666.);
		//#endif
	}

	public abstract void clientMoveAndDisplay();

	public boolean isIdle() {
		return idle;
	}

	public void setIdle(boolean idle) {
		this.idle = idle;
	}

	public void drawString(Graphics3D g, CharSequence s, int x, int y, Color3f color) {
		g.setColor(new Color(color.x*255, color.y*255, color.y*255));
		g.drawString(s.toString(), x, y);
	}
}
