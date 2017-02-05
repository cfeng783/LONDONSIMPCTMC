package odesolver;

import java.util.ArrayList;
import java.util.HashMap;

import moment.MomentScripter;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MaxCountExceededException;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;

import trans.FluidTrans;
import trans.Trans;

public class TestSolver implements FirstOrderDifferentialEquations {
	ArrayList<String> agentList;

	HashMap<String, Integer> indexMap; //indexMap for first moment
	
	HashMap<String, Integer> secondIndexMap; //indexMap for second moment
	
	HashMap<String, Integer> covIndexMap; //indexMap for covariance
	
	HashMap<Integer, String> covStateMap;
	
	ArrayList<FluidTrans> transArray;
	
	HashMap<Integer, ArrayList<FluidTrans>> indexTransMap;
	
	
	int finalOdeIndex;
	
	int[] alterPoints;
	
	public TestSolver(ArrayList<String> agentList, ArrayList<Trans> transArray, int finalOdeIndex,
			HashMap<String, Integer> indexMap, HashMap<String, Integer> secondIndexMap, 
			HashMap<String, Integer> covIndexMap, HashMap<Integer, String> covStateMap, int[] alterPoints) {
		this.agentList = agentList;
		this.finalOdeIndex = finalOdeIndex - 1;
		this.indexMap = new HashMap<String, Integer>();
		for(String key: indexMap.keySet()) {
			this.indexMap.put(key, indexMap.get(key)-1);
		}
		
		this.secondIndexMap = new HashMap<String, Integer>();
		for(String key: secondIndexMap.keySet()) {
			this.secondIndexMap.put(key, secondIndexMap.get(key)-1);
		}
		
		this.covIndexMap = new HashMap<String, Integer>();
		this.covStateMap = new HashMap<Integer, String>();
		for(String key: covIndexMap.keySet()) {
			this.covIndexMap.put(key, covIndexMap.get(key)-1);
			this.covStateMap.put(covIndexMap.get(key)-1, key);
		}
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
			int index = indexMap.get(agent);
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkSingleMoment(agent)) {
					correspondingTrans.add(trans);
//					trans.getODEIndices().add(index); //add corresponding ODE index
				}
			}
			indexTransMap.put(index, correspondingTrans);
		}
		
		for(int i=0; i<agentList.size(); i++) {
			String agent = agentList.get(i);
			int index = secondIndexMap.get(agent);
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkSingleMoment(agent)) {
					correspondingTrans.add(trans);
//					trans.getODEIndices().add(index);
				}
			}
			indexTransMap.put(index, correspondingTrans);
		}
		
		for(int i=2*agentList.size(); i<finalOdeIndex; i++) {
			String st[] = MomentScripter.resolveCovKey(covStateMap.get(i));
			int index = i;
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkCovMoment(st[0], st[1])) {
					correspondingTrans.add(trans);
//					trans.getODEIndices().add(index);
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
			int index = indexMap.get(agent);
			yDot[index] = 0;
			for(FluidTrans trans: indexTransMap.get(index)) {
				yDot[index] += trans.makeFirstMoment(t, y, alterPoints, agent, indexMap, secondIndexMap, covIndexMap);
			}
		}
		
		for(int i=0; i<agentList.size(); i++) {
			String agent = agentList.get(i);
			int index = secondIndexMap.get(agent);
			yDot[index] = 0;
			for(FluidTrans trans: indexTransMap.get(index)) {
				yDot[index] += trans.makeSecondMoment(t, y, alterPoints, agent, indexMap, secondIndexMap, covIndexMap);
			}
		}
		
		for(int i=2*agentList.size(); i<finalOdeIndex; i++) {
			yDot[i] = 0;
			String st[] = MomentScripter.resolveCovKey(covStateMap.get(i));
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