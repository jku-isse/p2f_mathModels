package at.pro2future.shopfloors.vmodels;

import java.util.Vector;

public class LinearRobot implements VirtualMachineModel {

	final int dof = 3;
	
	private DegreeOfFreedom q1, q2, q3;
	private double q1p, q2p, q3p;
	private double m1, m2, m3;
	
	final double g = 9.81;
	
	Vector<Double> derInternal; 
	Vector<DegreeOfFreedom> stateInternal;
	
	public LinearRobot(double q10, double q20, double q30) {
		this(3.0,2.0,1.0, q10,q20,q30, 0.0,0.0,0.0);
	}
	
	public LinearRobot(
			double mBase,
			double mMiddle,
			double mTool,
			double q10,
			double q20, 
			double q30,
			double q1p0,
			double q2p0,
			double q3p0) {
		
		m1 = mBase;
		m2 = mMiddle;
		m3 = mTool;
		
		q1 = new DegreeOfFreedom(q10);
		q2 = new DegreeOfFreedom(q20);
		q3 = new DegreeOfFreedom(q30);
		
		q1p = q1p0;
		q2p = q2p0;
		q3p = q3p0;
		
		stateInternal = new Vector<>(3);
		stateInternal.add(q1);
		stateInternal.add(q2);
		stateInternal.add(q3);
		
		derInternal = new Vector<>(3);
		derInternal.add(q1p0);
		derInternal.add(q2p0);
		derInternal.add(q3p0);
	}
	
	public LinearRobot() {
		this(3.0,2.0,1.0, 0.0,0.0,0.0, 0.0,0.0,0.0);
	}
	
	public void setToolMass(double mTool) {
		m3 = mTool;
	}
	
	@Override
	public int getDof() {
		return dof;
	}

	@Override
	public void step(double dt, Vector<Double> inputs) {
		Vector<Double> state = step(dt, stateInternal, derInternal, inputs, derInternal);
		for(int i = 0; i < 3; i++) {
			stateInternal.get(i).value = state.get(i);
		}
	}

	@Override
	public Vector<Double> step(double dt, Vector<DegreeOfFreedom> currentState, Vector<Double> currentDerivate, Vector<Double> inputs, Vector<Double> derExpected) {
		
		Vector<Double> stateExpected = new Vector<>(); 
		
		double xp0 = currentDerivate.get(0);
		double yp0 = currentDerivate.get(1);
		double zp0 = currentDerivate.get(2);
		
		
		double xpp = inputs.get(1) / (m1 + m2 + m3);
		double ypp = inputs.get(0) / (m1 + m2 + m3);
		double zpp = g - inputs.get(2) / (m2 + m3);
		double xp = q1p + xpp*dt;
		double yp = q2p + ypp*dt;
		double zp = q3p + zpp*dt;
		double x = q1.value + (xp + q1p)*dt/2.0;
		double y = q2.value + (yp + q2p)*dt/2.0;
		double z = q3.value + (zp + q3p)*dt/2.0;
		
		q1.value = x;
		q2.value = y;
		q3.value = z;
		q1p = xp;
		q2p = yp;
		q3p = zp;
		
		stateExpected.add(x);
		stateExpected.add(y);
		stateExpected.add(z);
		derExpected.set(0, xp);
		derExpected.set(1,yp);
		derExpected.set(2,zp);
		
		return stateExpected;
	}

	@Override
	public Vector<DegreeOfFreedom> getState() {
		return stateInternal;
	}
	
	@Override
	public Vector<Double> getDerivate() {
		return derInternal;
	}

}
