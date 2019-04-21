package at.pro2future.shopfloors.vmodels;

import java.util.Vector;

public class LinearRobot implements VirtualMachineModel {

	final int dof = 3;
	
	private double q1, q2, q3;
	private double q1p, q2p, q3p;
	private double m1, m2, m3;
	
	final double g = 9.81;
	
	Vector<Double> state, der; 
	
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
		
		q1 = q10;
		q2 = q20;
		q3 = q30;
		
		q1p = q1p0;
		q2p = q2p0;
		q3p = q3p0;
		
		state = new Vector<>(3);
		state.add(q10);
		state.add(q20);
		state.add(q30);
		
		der = new Vector<>(3);
		der.add(q1p0);
		der.add(q2p0);
		der.add(q3p0);
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
	public Vector<Double> step(double dt, Vector<Double> inputs) {
		Vector<Double> res = step(dt, state, der, inputs, der);
		for(int i = 0; i < dof; i++) {
			state.set(i, res.get(i));
		}
		return state;
	}

	@Override
	public Vector<Double> step(double dt, Vector<Double> state, Vector<Double> derivate, Vector<Double> inputs, Vector<Double> derivateExpected ) {
		Vector<Double> res = new Vector<>(dof);
		
		double xpp = inputs.get(1) / (m1 + m2 + m3);
		double ypp = inputs.get(0) / (m1 + m2 + m3);
		double zpp = g - inputs.get(2) / (m2 + m3);
		double xp = q1p + xpp*dt;
		double yp = q2p + ypp*dt;
		double zp = q3p + zpp*dt;
		double x = q1 + (xp + q1p)*dt/2.0;
		double y = q2 + (yp + q2p)*dt/2.0;
		double z = q3 + (zp + q3p)*dt/2.0;
		
		q1 = x;
		q2 = y;
		q3 = z;
		q1p = xp;
		q2p = yp;
		q3p = zp;
		
		res.add(0, x);
		res.add(1,y);
		res.add(2,z);
		derivate.set(0, xp);
		derivate.set(1,yp);
		derivate.set(2,zp);
		
		return res;
	}

	@Override
	public Vector<Double> getState() {
		return state;
	}
	
	@Override
	public Vector<Double> getDerivate() {
		return der;
	}

}
