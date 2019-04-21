package at.pro2future.shopfloors.vmodels;

import java.util.Vector;

public interface VirtualMachineModel {
	// get the number of degrees of freedom
	public int getDof();
	// calculate the next timestep of the model with given input (will advance the state of the model)
	public Vector<Double> step(double dt, Vector<Double> inputs);
	// extrapolate a specific state of the model (will not change the state of the model)
	public Vector<Double> step(double dt, Vector<Double> state, Vector<Double> derivate, Vector<Double> inputs, Vector<Double> derivateExpected);
	// get the current state of the model
	public Vector<Double> getState();
	// get the approximate change of the state
	public Vector<Double> getDerivate(); 
}
