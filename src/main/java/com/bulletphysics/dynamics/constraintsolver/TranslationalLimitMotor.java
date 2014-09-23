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

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le�n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/

package com.bulletphysics.dynamics.constraintsolver;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.VectorUtil;


import javax.vecmath.Vector3f;

/**
 *
 * @author jezek2
 */
public class TranslationalLimitMotor {
	
	//protected final BulletStack stack = BulletStack.get();
	
	public final Vector3f lowerLimit = new Vector3f(); //!< the constraint lower limits
	public final Vector3f upperLimit = new Vector3f(); //!< the constraint upper limits
	public final Vector3f accumulatedImpulse = new Vector3f();
	
	public float limitSoftness;     //!< Softness for linear limit
	public float damping;           //!< Damping for linear limit
	public float restitution;       //!< Bounce parameter for linear limit

        // added for 6dofSpring
        public final boolean enableMotor[]      = new boolean[3];
        public final Vector3f targetVelocity    = new Vector3f();   //!< target motor velocity
	public final Vector3f maxMotorForce     = new Vector3f();   //!< max force on motor
        public final Vector3f maxLimitForce     = new Vector3f();   //!< max force on limit
        public final Vector3f currentLimitError = new Vector3f();   //!  How much is violated this limit
        public final Vector3f currentLinearDiff = new Vector3f();   //!  Current relative offset of constraint frames
        public final int currentLimit[]         = new int[3];       //!< 0=free, 1=at lower limit, 2=at upper limit
        

	public TranslationalLimitMotor() {
		lowerLimit.set(0f, 0f, 0f);
		upperLimit.set(0f, 0f, 0f);
		accumulatedImpulse.set(0f, 0f, 0f);

		limitSoftness = 0.7f;
		damping = 1.0f;
		restitution = 0.5f;

                targetVelocity.set(0f, 0f, 0f);
                maxMotorForce.set(0.1f, 0.1f, 0.1f);
                maxLimitForce.set(300.0f, 300.0f, 300.0f);

                for (int i = 0 ; i<3 ; i++) {
                    enableMotor[i] = false;
                }
	}


	public TranslationalLimitMotor(TranslationalLimitMotor other) {
		lowerLimit.set(other.lowerLimit);
		upperLimit.set(other.upperLimit);
		accumulatedImpulse.set(other.accumulatedImpulse);

		limitSoftness = other.limitSoftness;
		damping = other.damping;
		restitution = other.restitution;
	}

	/**
	 * Test limit.<p>
	 * - free means upper &lt; lower,<br>
	 * - locked means upper == lower<br>
	 * - limited means upper &gt; lower<br>
	 * - limitIndex: first 3 are linear, next 3 are angular
	 */
	public boolean isLimited(int limitIndex) {
		return (VectorUtil.getCoord(upperLimit, limitIndex) >= VectorUtil.getCoord(lowerLimit, limitIndex));
	}

	/**
	 * Need apply correction?
	 */
        public boolean needApplyForces(int idx)
        {
            if(currentLimit[idx] == 0 && enableMotor[idx] == false) {
                return false;
            } 
            return true;
        }

        public int testLimitValue(int limitIndex, float test_value)
        {
            float loLimit = VectorUtil.getCoord(lowerLimit, limitIndex);
            float hiLimit = VectorUtil.getCoord(upperLimit, limitIndex);
            if(loLimit > hiLimit)
            {
                currentLimit[limitIndex] = 0;//Free from violation
                VectorUtil.setCoord(currentLimitError, limitIndex, 0.f);
                return 0;
            }

            if (test_value < loLimit)
            {
                currentLimit[limitIndex] = 2;//low limit violation
                VectorUtil.setCoord(currentLimitError, limitIndex, test_value - loLimit);
                return 2;
            }
            else if (test_value > hiLimit)
            {
                currentLimit[limitIndex] = 1;//High limit violation
                VectorUtil.setCoord(currentLimitError, limitIndex, test_value - hiLimit);
                return 1;
            }

            currentLimit[limitIndex] = 0;//Free from violation
            VectorUtil.setCoord(currentLimitError, limitIndex, 0.f);
            return 0;
        }




	public float solveLinearAxis(float timeStep, float jacDiagABInv, RigidBody body1, Vector3f pointInA, RigidBody body2, Vector3f pointInB, int limit_index, Vector3f axis_normal_on_a, Vector3f anchorPos) {
		Vector3f tmp = new Vector3f();
		Vector3f tmpVec = new Vector3f();
		
		// find relative velocity
		Vector3f rel_pos1 = new Vector3f();
		//rel_pos1.sub(pointInA, body1.getCenterOfMassPosition(tmpVec));
		rel_pos1.sub(anchorPos, body1.getCenterOfMassPosition(tmpVec));

		Vector3f rel_pos2 = new Vector3f();
		//rel_pos2.sub(pointInB, body2.getCenterOfMassPosition(tmpVec));
		rel_pos2.sub(anchorPos, body2.getCenterOfMassPosition(tmpVec));

		Vector3f vel1 = body1.getVelocityInLocalPoint(rel_pos1, new Vector3f());
		Vector3f vel2 = body2.getVelocityInLocalPoint(rel_pos2, new Vector3f());
		Vector3f vel = new Vector3f();
		vel.sub(vel1, vel2);

		float rel_vel = axis_normal_on_a.dot(vel);

		// apply displacement correction
                float target_velocity   = VectorUtil.getCoord(this.targetVelocity, limit_index);
                float maxMotorForce     = VectorUtil.getCoord(this.maxMotorForce, limit_index);

                float limErr = VectorUtil.getCoord(currentLimitError, limit_index);
                if (currentLimit[limit_index] != 0)
                {
                    target_velocity = restitution * limErr / (timeStep);
                    maxMotorForce = VectorUtil.getCoord(maxLimitForce, limit_index);
                }
		maxMotorForce *= timeStep;


                // correction velocity
		float motor_relvel = limitSoftness * (target_velocity - damping * rel_vel);
		if (motor_relvel < BulletGlobals.FLT_EPSILON && motor_relvel > -BulletGlobals.FLT_EPSILON) {
			return 0.0f; // no need for applying force
		}
                
                // correction impulse
		float unclippedMotorImpulse = motor_relvel * jacDiagABInv;

		// clip correction impulse
		float clippedMotorImpulse;

		// todo: should clip against accumulated impulse
		if (unclippedMotorImpulse > 0.0f) {
			clippedMotorImpulse = unclippedMotorImpulse > maxMotorForce ? maxMotorForce : unclippedMotorImpulse;
		}
		else {
			clippedMotorImpulse = unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce : unclippedMotorImpulse;
		}

                float normalImpulse = clippedMotorImpulse;

                // sort with accumulated impulses
		float lo = -1e30f;
		float hi = 1e30f;

                
		float oldNormalImpulse = VectorUtil.getCoord(accumulatedImpulse, limit_index);
		float sum = oldNormalImpulse + normalImpulse;
		VectorUtil.setCoord(accumulatedImpulse, limit_index, sum > hi ? 0f : sum < lo ? 0f : sum);
		normalImpulse = VectorUtil.getCoord(accumulatedImpulse, limit_index) - oldNormalImpulse;

		Vector3f impulse_vector = new Vector3f();
		impulse_vector.scale(normalImpulse, axis_normal_on_a);
		body1.applyImpulse(impulse_vector, rel_pos1);

		tmp.negate(impulse_vector);
		body2.applyImpulse(tmp, rel_pos2);
		return normalImpulse;
	}
	
}
