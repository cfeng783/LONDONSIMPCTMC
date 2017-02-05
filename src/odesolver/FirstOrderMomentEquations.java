package odesolver;

import java.util.ArrayList;
import java.util.HashMap;

import moment.MomentGenerator;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MaxCountExceededException;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;

import trans.FluidTrans;
import trans.Trans;

public class FirstOrderMomentEquations implements FirstOrderDifferentialEquations {
	ArrayList<String> agentList;

	HashMap<String, Integer> indexMap; //indexMap for first moment
	
	ArrayList<FluidTrans> transArray;
	
	HashMap<Integer, ArrayList<FluidTrans>> indexTransMap;
	
	
	int finalOdeIndex;
	
	int[] alterPoints;
	
	public FirstOrderMomentEquations(ArrayList<String> agentList, ArrayList<Trans> transArray,
			HashMap<String, Integer> indexMap, int[] alterPoints) {
		this.agentList = agentList;
		this.finalOdeIndex = agentList.size();
		this.indexMap = indexMap;
		
		this.transArray = new ArrayList<FluidTrans>();
		for(Trans trans: transArray) {
			FluidTrans ftrans = new FluidTrans(trans);
			this.transArray.add(ftrans);
		}
		this.finalOdeIndex = agentList.size();
		this.alterPoints = alterPoints;
		indexTrans();
	}
	
	private void indexTrans() {
		indexTransMap = new HashMap<Integer, ArrayList<FluidTrans>>();
		for(int i=0; i<agentList.size(); i++) {
			String agent = agentList.get(i);
			int firstOderIndex = indexMap.get(agent);
		
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkSingleMoment(agent)) {
					correspondingTrans.add(trans);
				}
			}
			indexTransMap.put(firstOderIndex, correspondingTrans);
			
		}
	}
	
	
	@Override
	public void computeDerivatives(double t, double[] y, double[] yDot)
			throws MaxCountExceededException, DimensionMismatchException {
		for(int i=0; i<agentList.size(); i++) {
			String agent = agentList.get(i);
			int firstOrderIndex = indexMap.get(agent);
			yDot[firstOrderIndex] = 0;
			for(FluidTrans trans: indexTransMap.get(firstOrderIndex)) {
				yDot[firstOrderIndex] += trans.makeFirstMoment(t, y, alterPoints, agent, indexMap, null, null);
			}
		}
	}

	@Override
	public int getDimension() {
		return this.finalOdeIndex;
	}


}
