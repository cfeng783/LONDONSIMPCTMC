package odesolver;

import java.util.ArrayList;
import java.util.HashMap;

import moment.MomentGenerator;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MaxCountExceededException;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;

import trans.FluidTrans;
import trans.Trans;

public class SecondOrderMomentEquations implements FirstOrderDifferentialEquations {
	ArrayList<String> agentList;

	HashMap<String, Integer> indexMap; //indexMap for first moment
	
	HashMap<String, Integer> secondIndexMap; //indexMap for second moment
	
	HashMap<String, Integer> covIndexMap; //indexMap for covariance
	
	HashMap<Integer, String> covStateMap;
	
	ArrayList<FluidTrans> transArray;
	
	HashMap<Integer, ArrayList<FluidTrans>> indexTransMap;
	
	
	int finalOdeIndex;
	
	int[] alterPoints;
	
	public SecondOrderMomentEquations(ArrayList<String> agentList, ArrayList<Trans> transArray, int finalOdeIndex,
			HashMap<String, Integer> indexMap, HashMap<String, Integer> secondIndexMap, 
			HashMap<String, Integer> covIndexMap, HashMap<Integer, String> covStateMap, int[] alterPoints) {
		this.agentList = agentList;
		this.finalOdeIndex = finalOdeIndex;
		this.indexMap = indexMap;
		this.secondIndexMap = secondIndexMap;
		this.covIndexMap = covIndexMap;
		this.covStateMap = covStateMap;
		
		this.transArray = new ArrayList<FluidTrans>();
		for(Trans trans: transArray) {
			FluidTrans ftrans = new FluidTrans(trans);
			this.transArray.add(ftrans);
		}
		
		this.alterPoints = alterPoints;
		indexTrans();
	}
	
	private void indexTrans() {
		indexTransMap = new HashMap<Integer, ArrayList<FluidTrans>>();
		for(int i=0; i<agentList.size(); i++) {
			String agent = agentList.get(i);
			int firstOderIndex = indexMap.get(agent);
			int secondOrderIndex = secondIndexMap.get(agent);
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkSingleMoment(agent)) {
					correspondingTrans.add(trans);
				}
			}
			indexTransMap.put(firstOderIndex, correspondingTrans);
			indexTransMap.put(secondOrderIndex, correspondingTrans);
		}
		
		for(int i=2*agentList.size(); i<finalOdeIndex; i++) {
			String st[] = MomentGenerator.resolveCovKey(covStateMap.get(i));
			int index = i;
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkCovMoment(st[0], st[1])) {
					correspondingTrans.add(trans);
				}
			}
			indexTransMap.put(index, correspondingTrans);
//			System.out.println("corresponding trans: " + correspondingTrans.size());
		}
	}
	
	
//	@Override
//	public void computeDerivatives(double t, double[] y, double[] yDot)
//			throws MaxCountExceededException, DimensionMismatchException {
//		java.util.Arrays.fill(yDot, 0.0);
//		for(FluidTrans trans: transArray) {
//			ArrayList<Integer> odeIndices = trans.getODEIndices();
//			for(Integer index: odeIndices) {
//				if(index<agentList.size()) {
//					yDot[index] += trans.makeFirstMoment(t, y, alterPoints, agentList.get(index), indexMap, secondIndexMap, covIndexMap);
//				}else if(index>=agentList.size() && index<2*agentList.size()) {
//					yDot[index] += trans.makeSecondMoment(t, y, alterPoints, agentList.get(index-agentList.size()), indexMap, secondIndexMap, covIndexMap);
//				}else{
//					String st[] = MomentScripter.resolveCovKey(covStateMap.get(index));
//					yDot[index]  += trans.makeCovMoment(t, y, alterPoints, st[0], st[1], indexMap, secondIndexMap, covIndexMap);
//					
//				}
//			}
//		}
//		
//	}
	
	@Override
	public void computeDerivatives(double t, double[] y, double[] yDot)
			throws MaxCountExceededException, DimensionMismatchException {
		for(int i=0; i<agentList.size(); i++) {
			String agent = agentList.get(i);
			int firstOrderIndex = indexMap.get(agent);
			int secondOrderIndex = secondIndexMap.get(agent);
			yDot[firstOrderIndex] = 0;
			yDot[secondOrderIndex] = 0;
			for(FluidTrans trans: indexTransMap.get(firstOrderIndex)) {
				yDot[firstOrderIndex] += trans.makeFirstMoment(t, y, alterPoints, agent, indexMap, secondIndexMap, covIndexMap);
				yDot[secondOrderIndex] += trans.makeSecondMoment(t, y, alterPoints, agent, indexMap, secondIndexMap, covIndexMap);
			}
		}
		
//		for(int i=0; i<agentList.size(); i++) {
//			String agent = agentList.get(i);
//			int index = secondIndexMap.get(agent);
//			yDot[index] = 0;
//			for(FluidTrans trans: indexTransMap.get(index)) {
//				yDot[index] += trans.makeSecondMoment(t, y, alterPoints, agent, indexMap, secondIndexMap, covIndexMap);
//			}
//		}
		
		for(int i=2*agentList.size(); i<finalOdeIndex; i++) {
			yDot[i] = 0;
			String st[] = MomentGenerator.resolveCovKey(covStateMap.get(i));
			for(FluidTrans trans: indexTransMap.get(i)) {
				yDot[i]  += trans.makeCovMoment(t, y, alterPoints, st[0], st[1], indexMap, secondIndexMap, covIndexMap);
			}
		}
		
	}

	@Override
	public int getDimension() {
		return this.finalOdeIndex;
	}


}