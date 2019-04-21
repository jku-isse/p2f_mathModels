package at.pro2future.shopfloors.vmodels.tests;

import java.util.Vector;

import at.pro2future.shopfloors.vmodels.LinearRobot;

public class LinearRobotTest {
	public static void main(String[] args) {
		LinearRobot robot = new LinearRobot(0.45,0.45,0.5);
		long lastTime = System.nanoTime();
		long curTime = lastTime;
		
		long lastPrintTime = lastTime;
		long lastDemoTime = lastTime;
		long lastControlTime = lastTime;
		long lastModelTime = lastTime;
		
		long dt_demo_l = 5000000000l; //5*10e9 nanoseconds ~ 5 seconds
		
		
		long dt_print_l = 500000000l; //5*10e8 nanoseconds ~ 0.5 second
		
		long dtl = 100000l; //10e5 nanoseconds ~ 0.1 milliseconds
		double dtc = 0.0001; //10e-4 seconds ~ 0.1 milliseconds

		long dt_model_l = 10000l; // 10e4 nanoseconds ~ 0.01 milliseconds;
		double dt = 0.00001; //10e-5 seconds ~ 0.01 milliseconds
		
		Vector<Double> inputs = new Vector<>(); // Forces acting on the axis q1, q2, q3
		inputs.addElement(0.0);
		inputs.addElement(0.0);
		inputs.addElement(30.0);
		
		Vector<Double> coeffQP = new Vector<>(); // control amplification, positions 
		coeffQP.add(200.0);
		coeffQP.add(200.0);
		coeffQP.add(300.0);

		Vector<Double> coeffQI = new Vector<>(); // control amplification, positions 
		coeffQI.add(220.0);
		coeffQI.add(220.0);
		coeffQI.add(320.0);

		Vector<Double> coeffQpP = new Vector<>(); // control amplification, positions 
		coeffQpP.add(24.9);
		coeffQpP.add(24.9);
		coeffQpP.add(24.9);

		Vector<Double> coeffQpI = new Vector<>(); // control amplification, positions 
		coeffQpI.add(1.5);
		coeffQpI.add(1.5);
		coeffQpI.add(1.5);

		Vector<Double> qt = new Vector<>(); // target for robot state 
		qt.add(0.5);
		qt.add(0.5);
		qt.add(0.5);

		Vector<Double> qpt = new Vector<>(); // target for robot speeds - should not move if at target state.
		qpt.add(0.0);
		qpt.add(0.0);
		qpt.add(0.0);
		
		PIController control = new PIController(3, dtc, coeffQP, coeffQI, coeffQpP, coeffQpI);
		
		
		
		while(true) {
			
			// periodic task for model calculation
			long modelDiff = curTime - lastModelTime;
			if(modelDiff < 0l) {
				modelDiff = (Long.MAX_VALUE - lastModelTime) + (curTime - Long.MIN_VALUE);
				if(modelDiff > dt_print_l) {
					System.out.println(curTime);
					lastModelTime = Long.MIN_VALUE + dt_print_l - (Long.MAX_VALUE-lastModelTime);
				}
			} else {
				if(modelDiff > dt_model_l) {
					lastModelTime = lastModelTime + dt_model_l;
				}
			}
			if(modelDiff > dt_model_l) {
				robot.step(dt, inputs);
			}
			
			// periodic task for controller calculation
			long diff = curTime - lastControlTime;
			if(diff < 0l) {
				diff = (Long.MAX_VALUE - lastControlTime) + (curTime - Long.MIN_VALUE);
				if(diff > dt_print_l) {
					System.out.println(curTime);
					lastControlTime = Long.MIN_VALUE + dt_print_l - (Long.MAX_VALUE-lastControlTime);
				}
			} else {
				if(diff > dtl) {
					lastControlTime = lastControlTime + dtl;
				}
			}
			if(diff > dtl) {
				
				Vector<Double> input_calc = control.getInputs(qt, qpt, robot.getState(), robot.getDerivate());
				
				double F2 = input_calc.get(0);
				double F1 = input_calc.get(1);
				double F3 = -input_calc.get(2);
				
				F1 = Math.min(50.0, Math.max(-50.0, F1));
				F2 = Math.min(50.0, Math.max(-50.0, F2));
				F3 = Math.min(50.0, Math.max(-50.0, F3));
				
				inputs.set(0,F1);
				inputs.set(1,F2);
				inputs.set(2,F3);
			}
			
			// periodic task for printing state
			long printDiff = curTime - lastPrintTime;
			if(printDiff < 0l) {
				printDiff = (Long.MAX_VALUE - lastPrintTime) + (curTime - Long.MIN_VALUE);
				if(printDiff > dt_print_l) {
					lastPrintTime = Long.MIN_VALUE + dt_print_l - (Long.MAX_VALUE-lastPrintTime);
				}
			} else {
				if(printDiff > dt_print_l) {
					lastPrintTime += dt_print_l;
				}
			}
			if(printDiff > dt_print_l) {
				Vector<Double> state = robot.getState();
				Vector<Double> der = robot.getDerivate();
				System.out.println(String.format("Position at time %d: %6.3fm(%6.3fm/s) / %6.3fm(%6.3fm/s) / %6.3fm(%6.3fm/s)", curTime, state.get(0), der.get(0),state.get(1), der.get(1), state.get(2), der.get(2)));
			}

			// periodic task for setting target state
			long demoDiff = curTime - lastDemoTime;
			if(demoDiff < 0l) {
				demoDiff = (Long.MAX_VALUE - lastDemoTime) + (curTime - Long.MIN_VALUE);
				if(demoDiff > dtl) {
					lastDemoTime = Long.MIN_VALUE + dtl - (Long.MAX_VALUE-lastDemoTime);
				}
			} else {
				if(demoDiff > dt_demo_l) {
					lastDemoTime += dt_demo_l;
				}
			}
			if(demoDiff > dt_demo_l) {
				qt.set(0, qt.get(0) + 0.1);
				qt.set(1, qt.get(1) + 0.05);
				qt.set(2, qt.get(2) + 0.1);
			}
			curTime = System.nanoTime();
		}
		
	}
}
