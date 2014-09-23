/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

/**
 *
 *
 * Generic 6 DOF constraint that allows to set spring motors to any translational and rotational DOF
 * DOF index used in enableSpring() and setStiffness() means:
 *    0 : translation X
 *    1 : translation Y
 *    2 : translation Z
 *    3 : rotation X (3rd Euler rotational around new position of X axis, range [-PI+epsilon, PI-epsilon] )
 *    4 : rotation Y (2nd Euler rotational around new position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
 *    5 : rotation Z (1st Euler rotational around Z axis, range [-PI+epsilon, PI-epsilon] )
 *
 * @author Ported to JBullet from Bullet by gideonk as part of the QIntBio project
 */
public class Generic6DofSpringConstraint extends Generic6DofConstraint {

    private boolean springEnabled[] = new boolean[6];
    private float equilibriumPoint[] = new float[6];
    private float springStiffness[] = new float[6];
    private float springDamping[] = new float[6]; // between 0 and 1 (1 == no damping)

    public Generic6DofSpringConstraint(RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB, boolean useLinearReferenceFrameA) {
        super(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA);
        this.constraintType = TypedConstraintType.D6_SPRING_CONSTRAINT_TYPE;

        for (int i = 0; i < 6; i++) {
            springEnabled[i] = false;
            equilibriumPoint[i] = 0.f;
            springStiffness[i] = 0.f;
            springDamping[i] = 1.f;
        }
    }

    public void enableSpring(int index, boolean onOff) {
        assert (index >= 0) && (index < 6);
        springEnabled[index] = onOff;
        if (index < 3) {
            linearLimits.enableMotor[index] = onOff;
        } else {
            angularLimits[index - 3].enableMotor = onOff;
        }
    }

    public void setStiffness(int index, float stiffness) {
        assert (index >= 0) && (index < 6);
        springStiffness[index] = stiffness;
    }

    public void setDamping(int index, float damping) {
        assert ((index >= 0) && (index < 6));
        springDamping[index] = damping;
    }

    public void setEquilibriumPoint() {
        calculateTransforms();
        int i;

        for (i = 0; i < 3; i++) {
            equilibriumPoint[i] = VectorUtil.getCoord(calculatedLinearDiff, i);
        }
        for (i = 0; i < 3; i++) {
            equilibriumPoint[i + 3] = VectorUtil.getCoord(calculatedAxisAngleDiff, i);
        }
    }

    public void setEquilibriumPoint(int index) {
        assert ((index >= 0) && (index < 6));
        calculateTransforms();
        if (index < 3) {
            equilibriumPoint[index] = VectorUtil.getCoord(calculatedLinearDiff, index);
        } else {
            equilibriumPoint[index] = VectorUtil.getCoord(calculatedAxisAngleDiff, index-3);
        }
    }

    public void setEquilibriumPoint(int index, float val) {
        assert ((index >= 0) && (index < 6));
        equilibriumPoint[index] = val;
    }

    public void internalUpdateSprings(ContactSolverInfo info) {
	// it is assumed that calculateTransforms() have been called before this call
	int i;

        Vector3f velA = rbA.getLinearVelocity( new Vector3f() );
        Vector3f velB = rbB.getLinearVelocity( new Vector3f() );
        
	Vector3f relVel = new Vector3f();
        relVel.sub(velB, velA);

        float fps = 1.f / info.timeStep;

	for(i = 0; i < 3; i++) {
            if (springEnabled[i]) {
                // get current position of constraint
                float currPos = VectorUtil.getCoord(calculatedLinearDiff, i);
                // calculate difference
                float delta = currPos - equilibriumPoint[i];
                // spring force is (delta * stiffness) according to Hooke's Law
                float force = delta * springStiffness[i];

                float velFactor = fps * springDamping[i] / (float)(info.numIterations);
                VectorUtil.setCoord(linearLimits.targetVelocity, i, velFactor * force);
		VectorUtil.setCoord(linearLimits.maxMotorForce, i, Math.abs(force) / fps);
            }
        }
        for (i = 0; i < 3; i++) {
            if (springEnabled[i + 3]) {
                // get current position of constraint
                float currPos = VectorUtil.getCoord(calculatedAxisAngleDiff, i);
                // calculate difference
                float delta = currPos - equilibriumPoint[i + 3];
                // spring force is (-delta * stiffness) according to Hooke's Law
                float force = -delta * springStiffness[i + 3];
                float velFactor = fps * springDamping[i + 3] / (float)(info.numIterations);
                angularLimits[i].targetVelocity = velFactor * force;
                angularLimits[i].maxMotorForce = Math.abs(force) / fps;
            }
        }
    }


public void setAxis(Vector3f axis1, Vector3f axis2)
{
	Vector3f zAxis = new Vector3f(axis1);
        zAxis.normalize();

        Vector3f yAxis = new Vector3f(axis2);
        zAxis.normalize();

        Vector3f xAxis = new Vector3f();
        xAxis.cross(yAxis, zAxis);      // we want right coordinate system

	Transform frameInW = new Transform();
	frameInW.setIdentity();

        frameInW.basis.setColumn(0, xAxis);
        frameInW.basis.setColumn(1, yAxis);
        frameInW.basis.setColumn(2, zAxis);


        // now get constraint frame in local coordinate systems
        Transform temp = new Transform();

        rbA.getCenterOfMassTransform(temp);
        temp.inverse();
        frameInA.mul(temp, frameInW);

        rbB.getCenterOfMassTransform(temp);
        temp.inverse();
        frameInB.mul(temp, frameInW);

//        // now get constraint frame in local coordinate systems
//        frameInA = rbA.getCenterOfMassTransform().inverse() * frameInW;
//        frameInB = rbB.getCenterOfMassTransform().inverse() * frameInW;

        calculateTransforms();
    }


    @Override
    public void getInfo2(ContactSolverInfo infoGlobal) {
        internalUpdateSprings(infoGlobal);
    }

}
