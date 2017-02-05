package odesolver;

import java.util.ArrayList;
import java.util.HashMap;

import moment.MomentGenerator;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MaxCountExceededException;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;

import trans.FluidTrans;
import trans.Trans;

public class ThirdOrderMomentEquations implements FirstOrderDifferentialEquations {
	ArrayList<String> agentList;

	HashMap<String, Integer> indexMap; //indexMap for first moment
	
	HashMap<String, Integer> secondIndexMap; //indexMap for second moment
	
	HashMap<String, Integer> covIndexMap; //indexMap for covariance
	
	HashMap<Integer, String> covStateMap;
	
	ArrayList<FluidTrans> transArray;
	
	HashMap<Integer, ArrayList<FluidTrans>> indexTransMap;
	
	HashMap<String, Integer> indexMap3;
	HashMap<String, Integer> indexMap21;
	HashMap<String, Integer> indexMap111;
	
	HashMap<Integer, String> stateIndexMap21;
	HashMap<Integer, String> stateIndexMap111;
	
	int finalOdeIndex;
	int covOdeIndex;
	int cov21StartIndex;
	int cov111StartIndex;
	
	int[] alterPoints;
	
	public ThirdOrderMomentEquations(ArrayList<String> agentList, ArrayList<Trans> transArray, int finalOdeIndex,
			int covOdeIndex, int cov21StartIndex, int cov111StartIndex,
			HashMap<String, Integer> indexMap, HashMap<String, Integer> secondIndexMap, 
			HashMap<String, Integer> covIndexMap, HashMap<Integer, String> covStateMap, 
			HashMap<String, Integer> indexMap3, HashMap<String, Integer> indexMap21, HashMap<String, Integer> indexMap111,
			HashMap<Integer, String> stateIndexMap21, HashMap<Integer, String> stateIndexMap111, int[] alterPoints) {
		this.agentList = agentList;
		this.finalOdeIndex = finalOdeIndex;
		this.indexMap = indexMap;
		this.covOdeIndex = covOdeIndex;
		this.cov21StartIndex = cov21StartIndex;
		this.cov111StartIndex = cov111StartIndex;
		this.secondIndexMap = secondIndexMap;
		this.covIndexMap = covIndexMap;
		this.covStateMap = covStateMap;
		this.indexMap3 = indexMap3;
		this.indexMap21 = indexMap21;
		this.indexMap111 = indexMap111;
		this.stateIndexMap21 = stateIndexMap21;
		this.stateIndexMap111 = stateIndexMap111;
		
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
			int firstOrderIndex = indexMap.get(agent);
			int secondOrderIndex = secondIndexMap.get(agent);
			int thirdOrderIndex = indexMap3.get(agent);
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkSingleMoment(agent)) {
					correspondingTrans.add(trans);
				}
			}
			indexTransMap.put(firstOrderIndex, correspondingTrans);
			indexTransMap.put(secondOrderIndex, correspondingTrans);
			indexTransMap.put(thirdOrderIndex, correspondingTrans);
		}
		
		for(int i=2*agentList.size(); i<this.covOdeIndex; i++) {
			String st[] = MomentGenerator.resolveCovKey(covStateMap.get(i));
			int index = i;
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkCovMoment(st[0], st[1])) {
					correspondingTrans.add(trans);
				}
			}
			indexTransMap.put(index, correspondingTrans);
		}
		
		for(int i=this.cov21StartIndex; i<this.cov111StartIndex; i++) {
			String st[] = MomentGenerator.resolveCovKey(stateIndexMap21.get(i));
			int index = i;
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkCovMoment(st[0], st[1])) {
					correspondingTrans.add(trans);
				}
			}
			indexTransMap.put(index, correspondingTrans);
		}
		
		for(int i=this.cov111StartIndex; i<this.finalOdeIndex; i++) {
			String st[] = MomentGenerator.resolveCovKey(stateIndexMap111.get(i));
			int index = i;
			ArrayList<FluidTrans> correspondingTrans = new ArrayList<FluidTrans>();
			for(FluidTrans trans: transArray) {
				if(trans.checkCovMoment(st[0], st[1], st[2])) {
					correspondingTrans.add(trans);
				}
			}
			indexTransMap.put(index, correspondingTrans);
		}
	}
	
	@Override
	public void computeDerivatives(double t, double[] y, double[] yDot)
			throws MaxCountExceededException, DimensionMismatchException {
		for(int i=0; i<agentList.size(); i++) {
			String agent = agentList.get(i);
			int firstOrderIndex = indexMap.get(agent);
			int secondOrderIndex = secondIndexMap.get(agent);
			int thirdOrderIndex = indexMap3.get(agent);
			yDot[firstOrderIndex] = 0;
			yDot[secondOrderIndex] = 0;
			yDot[thirdOrderIndex] = 0;
			for(FluidTrans trans: indexTransMap.get(firstOrderIndex)) {
				yDot[firstOrderIndex] += trans.makeFirstMoment(t, y, alterPoints, agent, indexMap, secondIndexMap, covIndexMap);
				yDot[secondOrderIndex] += trans.makeSecondMoment(t, y, alterPoints, agent, indexMap, secondIndexMap, covIndexMap);
				yDot[thirdOrderIndex] += trans.make3rdMoment(t, y, alterPoints, agent, indexMap, secondIndexMap,
						covIndexMap, indexMap111, indexMap21, indexMap3);
			}
		}
		
		for(int i=2*agentList.size(); i<covOdeIndex; i++) {
			yDot[i] = 0;
			String st[] = MomentGenerator.resolveCovKey(covStateMap.get(i));
			for(FluidTrans trans: indexTransMap.get(i)) {
				yDot[i]  += trans.makeCovMoment(t, y, alterPoints, st[0], st[1], indexMap, secondIndexMap, covIndexMap);
			}
		}
		
		for(int i=this.cov21StartIndex; i<this.cov111StartIndex; i++) {
			yDot[i] = 0;
			String st[] = MomentGenerator.resolveCovKey(stateIndexMap21.get(i));
			for(FluidTrans trans: indexTransMap.get(i)) {
				yDot[i]  += trans.make2plus1moment(t, y, alterPoints, st[0], st[1], indexMap, secondIndexMap,
						covIndexMap, indexMap111, indexMap21, indexMap3);
			}
		}
		
		for(int i=this.cov111StartIndex; i<this.finalOdeIndex; i++) {
			yDot[i] = 0;
			String st[] = MomentGenerator.resolveCovKey(stateIndexMap111.get(i));
			for(FluidTrans trans: indexTransMap.get(i)) {
				yDot[i]  += trans.make1plus1plus1moment(t, y, alterPoints, st[0], st[1], st[2], indexMap, secondIndexMap,
						covIndexMap, indexMap111, indexMap21, indexMap3);
			}
		}
		
	}

	@Override
	public int getDimension() {
		return this.finalOdeIndex;
	}


}