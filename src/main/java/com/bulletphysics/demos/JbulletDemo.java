package com.bulletphysics.demos;

import br.com.luvia.Luvia;
import br.com.luvia.core.context.ApplicationGL;

import com.bulletphysics.demos.genericjoint.GenericJointDemo;
import com.bulletphysics.demos.vehicle.VehicleDemo;

import examples.jbullet.JbulletExample;

public class JbulletDemo extends Luvia {

	public JbulletDemo() {
		super(800,600);
	}

	// Main program
	public static void main(String[] args) {
		JbulletDemo demo = new JbulletDemo();
		demo.init();
	}
	
	@Override
	public ApplicationGL startApplication() {
		initialSetup("../../../");
		
		//return new BasicDemo(w, h);
		//return new CharacterDemo(w, h);
		//return new ConcaveDemo(w, h);
		//return new DynamicControlDemo(w, h);
		return new GenericJointDemo(w, h);
		//return new MovingConcaveDemo(w, h);
		//return new VehicleDemo(w, h);
		//return new ForkLiftDemo(w, h);
		
	}
}
