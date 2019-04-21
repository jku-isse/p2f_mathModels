package at.pro2future.shopfloors.vmodels.tests;

import java.util.Vector;

public class PIController {
	Vector<Double> eq, eqi, eqp, eqpi; 
	Vector<Double> cQP, cQI, cQpP, cQpI;
	Vector<Double> result; 
	double dtc;
	int numStates;
	public PIController(int numStates, double controlTime, Vector<Double> coeffQP, Vector<Double> coeffQI, Vector<Double> coeffQpP, Vector<Double> coeffQpI) {
		eq = new Vector<>(3);
		eqi = new Vector<>(3);
		eqp = new Vector<>(3);
		eqpi = new Vector<>(3);
		result = new Vector<>(3);
		for(int i = 0; i < numStates; i++) {
			eq.add(0.0);
			eqi.add(0.0);
			eqp.add(0.0);
			eqpi.add(0.0);
			result.add(0.0);
		}
		cQP = new Vector<>(coeffQP);
		cQI = new Vector<>(coeffQI);
		cQpP = new Vector<>(coeffQpP);
		cQpI = new Vector<>(coeffQpI);
		
		dtc = controlTime;
		this.numStates = numStates;
	}
	public Vector<Double> getInputs(Vector<Double> qt, Vector<Double> qpt, Vector<Double> q, Vector<Double> qp) {
		for(int i = 0; i < numStates; i++) {
			eq.set(i, qt.get(i) - q.get(i));
			eqp.set(i, qpt.get(i) - qp.get(i));
			eqi.set(i, eqi.get(i) + eq.get(i) * dtc);
			eqpi.set(i, eqpi.get(i) + eqp.get(i) * dtc);
			result.set(i, cQP.get(i) * eq.get(i) + 
					cQI.get(i) * eqi.get(i) +
					cQpP.get(i) * eqp.get(i) + 
					cQpI.get(i) * eqpi.get(i));
		}
		return result;
	}
}
