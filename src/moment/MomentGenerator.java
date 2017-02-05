package moment;

import java.util.ArrayList;
import java.util.HashMap;

import odesolver.FirstOrderMomentEquations;
import odesolver.SecondOrderMomentEquations;
import odesolver.TestSolver;
import odesolver.ThirdOrderMomentEquations;

import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;

import simulator.RealSimuator;
import trans.Trans;

/**
 * A built-in moment ode generator
 * */
public class MomentGenerator {

	ArrayList<String> agentList;

	HashMap<String, Integer> indexMap = new HashMap<String, Integer>(); //indexMap for first moment
	
	HashMap<String, Integer> secondIndexMap = new HashMap<String, Integer>(); //indexMap for second moment
	
	HashMap<String, Integer> covIndexMap = new HashMap<String, Integer>(); //indexMap for covariance
	
	HashMap<Integer, String> covStateMap = new HashMap<Integer,String>();
	
	HashMap<String, Integer> indexMap3;
	HashMap<String, Integer> indexMap21;
	HashMap<String, Integer> indexMap111;
	
	HashMap<Integer, String> stateIndexMap21;
	HashMap<Integer, String> stateIndexMap111;
	
	ArrayList<Trans> transArray;
	
	int finalOdeIndex;
	
	
	public static int couplingThreshold = 1;
	
	int order;
	
	int covOdeIndex;
	int cov21StartIndex;
	int cov111StartIndex;
	
	public MomentGenerator(HashMap<String, Integer> initAgentMap, ArrayList<Trans> transArray) {
		this.transArray = transArray;
		agentList = new ArrayList<String>();
		for(String agent: initAgentMap.keySet()) {
			agentList.add(agent);
		}
	}
	
	public void setupIndex(int order) {
		this.order = order;
		for(int i=0;i<agentList.size();i++) {
			indexMap.put(agentList.get(i), i);
			secondIndexMap.put(agentList.get(i), i+agentList.size());
		}
		
		int current = 2*agentList.size();
//		System.out.println("current: " + current);
		for(int i=0; i<agentList.size(); i++) {
			for(int j=0; j<agentList.size(); j++) {
				String agent1 = agentList.get(i);
				String agent2 = agentList.get(j);
				if(!agent1.equals(agent2)) {
					//check correlation here
					int couplingcoefficient = RealSimuator.getGraphBuilder().getCouplingCoefficient(agent1, agent2);
//					if(couplingcoefficient >= couplingThreshold)
//						System.out.println(agent1+ " with " + agent2 + " : " + couplingcoefficient);
//				
					if(!checkCovItem(agent1, agent2) && couplingcoefficient <= couplingThreshold) {
						covIndexMap.put(makeCovKey(agent1, agent2), current);
						covStateMap.put(current, makeCovKey(agent1, agent2));
						current ++;
					}
				}
			}
		}
		
		covOdeIndex = current;
		
		if(order == 3) {
			indexMap3 = new HashMap<String, Integer>();
			indexMap21 = new HashMap<String, Integer>();
			indexMap111 = new HashMap<String, Integer>();
			stateIndexMap21 = new HashMap<Integer, String>();
			stateIndexMap111 = new HashMap<Integer, String>();
			for(int i=0; i<agentList.size(); i++) {
				indexMap3.put(agentList.get(i), current);
				current++;
			}
			cov21StartIndex = current;
			for(int i=0; i<agentList.size(); i++) {
				for(int j=0; j<agentList.size(); j++) {
					String agent1 = agentList.get(i);
					String agent2 = agentList.get(j);
					if(i != j) {
						//check correlation here
						double couplingcoefficient = RealSimuator.getGraphBuilder().getCouplingCoefficient(agent1, agent2);
						if(couplingcoefficient <= couplingThreshold) {
							indexMap21.put(makeCovKey(agent1, agent2), current);
							stateIndexMap21.put(current, makeCovKey(agent1, agent2));
							current ++;
						}
					}
				}
			}
			cov111StartIndex = current;
			for(int i=0; i<agentList.size(); i++) {
				for(int j=0; j<agentList.size(); j++) {
					for(int k=0; k<agentList.size(); k++) {
						String agent1 = agentList.get(i);
						String agent2 = agentList.get(j);
						String agent3 = agentList.get(k);
						if(i != j && i != k && j != k) {
							//check correlation here
							double couplingcoefficient1 = RealSimuator.getGraphBuilder().getCouplingCoefficient(agent1, agent2);
							double couplingcoefficient2 = RealSimuator.getGraphBuilder().getCouplingCoefficient(agent3, agent2);
							double couplingcoefficient3 = RealSimuator.getGraphBuilder().getCouplingCoefficient(agent1, agent3);
							if((couplingcoefficient1 <= couplingThreshold && couplingcoefficient2 <= couplingThreshold) ||
									(couplingcoefficient2 <= couplingThreshold && couplingcoefficient3 <= couplingThreshold) ||
									(couplingcoefficient1 <= couplingThreshold && couplingcoefficient3 <= couplingThreshold) ) {
								if(!checkCovItem(agent1, agent2, agent3)) {
									indexMap111.put(makeCovKey(agent1, agent2, agent3), current);
									stateIndexMap111.put(current, makeCovKey(agent1, agent2, agent3));
									current ++;
								}
							}
						}
					}
				}
			}
			
		}
		
		
		finalOdeIndex = current;
		System.out.println("ODE number: "+ finalOdeIndex);
	}
	
	
	public static String makeCovKey(String st1, String st2) {
		return st1 + "#" + st2;
	}
	
	public static String makeCovKey(String st1, String st2, String st3) {
		return st1 + "#" + st2 + "#" + st3;
	}
	
	public static String[] resolveCovKey(String covKey) {
		return covKey.split("#");
	}
	
	//looke for a cov item in covIndexMap, return true for existence
	public boolean checkCovItem(String st1, String st2) {
		if(covIndexMap.get(makeCovKey(st1, st2)) != null || covIndexMap.get(makeCovKey(st2, st1)) != null) {
			return true;
		}else {
			return false;
		}
	}
	
	public boolean checkCovItem(String st1, String st2, String st3) {
		if(indexMap111.get(makeCovKey(st1, st2, st3)) != null 
				|| indexMap111.get(makeCovKey(st1, st3, st2)) != null
				|| indexMap111.get(makeCovKey(st2, st1, st3)) != null
				|| indexMap111.get(makeCovKey(st2, st3, st1)) != null
				|| indexMap111.get(makeCovKey(st3, st1, st2)) != null
				|| indexMap111.get(makeCovKey(st3, st2, st1)) != null ) {
			return true;
		}else {
			return false;
		}
	}
	
	public static double getCovMoment(double[] y, String agent1, String agent2, HashMap<String, Integer> covMap) {
		if(covMap.get(makeCovKey(agent1, agent2)) != null) {
			return y[covMap.get(makeCovKey(agent1, agent2))];
		}
//		if(covMap.get(makeCovKey(agent2, agent1)) != null) {
			return y[covMap.get(makeCovKey(agent2, agent1))];
//		}
//		return y[indexMap.get(agent1)] * y[indexMap.get(agent2)];
	}
	
	public static double get111moment(double[] y, String agent1, String agent2, String agent3,
			HashMap<String, Integer> indexMap111) {
		if(indexMap111.get(makeCovKey(agent1, agent2, agent3)) != null) {
			return y[indexMap111.get(makeCovKey(agent1, agent2, agent3))];
		}
		if(indexMap111.get(makeCovKey(agent1, agent3, agent2)) != null) {
			return y[indexMap111.get(makeCovKey(agent1, agent3, agent2))];
		}
		if(indexMap111.get(makeCovKey(agent2, agent1, agent3)) != null) {
			return y[indexMap111.get(makeCovKey(agent2, agent1, agent3))];
		}
		if(indexMap111.get(makeCovKey(agent2, agent3, agent1)) != null) {
			return y[indexMap111.get(makeCovKey(agent2, agent3, agent1))];
		}
		if(indexMap111.get(makeCovKey(agent3, agent1, agent2)) != null) {
			return y[indexMap111.get(makeCovKey(agent3, agent1, agent2))];
		}
//		else(indexMap111.get(makeCovKey(agent3, agent2, agent1)) != null) {
			return y[indexMap111.get(makeCovKey(agent3, agent2, agent1))];
//		}
		
	}
	
	public FirstOrderDifferentialEquations getDiffEquations(int[] alterPoints) {
		if(this.order == 1) {
			FirstOrderDifferentialEquations ode = new FirstOrderMomentEquations(agentList, transArray, indexMap, alterPoints);
			return ode;
		} else if(this.order == 2) {
			FirstOrderDifferentialEquations ode = new SecondOrderMomentEquations(agentList, transArray, finalOdeIndex, indexMap, 
					secondIndexMap, covIndexMap, covStateMap,alterPoints);
			return ode;
		} else {
			FirstOrderDifferentialEquations ode = new ThirdOrderMomentEquations(agentList, transArray, finalOdeIndex, 
					covOdeIndex, cov21StartIndex, cov111StartIndex, indexMap, 
					secondIndexMap, covIndexMap, covStateMap,
					indexMap3, indexMap21, indexMap111, 
					stateIndexMap21, stateIndexMap111, alterPoints);
			return ode;
		}
		
		
	}
	
	public double[] getInitValues() {
		if(this.order == 1) {
			return getFirstOrderInitValues();
		}else if(this.order == 2) {
			return getSecondOrderInitValues();
		}else {
			return getThirdOrderInitValues();
		}
	}
	
	private double[] getFirstOrderInitValues() {
		double[] initValues = new double[agentList.size()];
		
		for(String agent: agentList) {
			int count = RealSimuator.getModel().getInitAgentMap().get(agent);
			int ind = indexMap.get(agent);
			initValues[ind] = count;
		}
		
		return initValues;
	}
	
	
	private double[] getSecondOrderInitValues() {
		double[] initValues = new double[finalOdeIndex];
		
		for(String agent: agentList) {
			int count = RealSimuator.getModel().getInitAgentMap().get(agent);
			int ind = indexMap.get(agent);
			initValues[ind] = count;
			
			int secIndex = secondIndexMap.get(agent);
			initValues[secIndex] = count*count;
		}
		
		for(int i=2*agentList.size(); i<finalOdeIndex; i++) {
			String st[] = resolveCovKey(covStateMap.get(i));
			int pos1 = indexMap.get(st[0]);
			int pos2 = indexMap.get(st[1]);
			double trueNum = initValues[pos1] * initValues[pos2];
			initValues[i] = trueNum;
		}
		return initValues;
	}
	
	private double[] getThirdOrderInitValues() {
		double[] initValues = new double[finalOdeIndex];
		
		for(String agent: agentList) {
			int count = RealSimuator.getModel().getInitAgentMap().get(agent);
			int ind = indexMap.get(agent);
			initValues[ind] = count;
			
			int secIndex = secondIndexMap.get(agent);
			initValues[secIndex] = count*count;
			
			int thirdIndex = indexMap3.get(agent);
			initValues[thirdIndex] = count*count*count;
		}
		
		for(int i=2*agentList.size(); i<this.covOdeIndex; i++) {
			String st[] = resolveCovKey(covStateMap.get(i));
			int pos1 = indexMap.get(st[0]);
			int pos2 = indexMap.get(st[1]);
			double trueNum = initValues[pos1] * initValues[pos2];
			initValues[i] = trueNum;
		}
		
		for(int i=this.cov21StartIndex; i<this.cov111StartIndex; i++) {
			String st[] = resolveCovKey(stateIndexMap21.get(i));
			int pos1 = indexMap.get(st[0]);
			int pos2 = indexMap.get(st[1]);
			double trueNum = Math.pow(initValues[pos1], 2) * initValues[pos2];
			initValues[i] = trueNum;
		}
		
		for(int i=this.cov111StartIndex; i<this.finalOdeIndex; i++) {
			String st[] = resolveCovKey(stateIndexMap111.get(i));
			int pos1 = indexMap.get(st[0]);
			int pos2 = indexMap.get(st[1]);
			int pos3 = indexMap.get(st[2]);
			double trueNum = initValues[pos1]* initValues[pos2] * initValues[pos3];
			initValues[i] = trueNum;
		}
		return initValues;
	}
	
	public int getIndex(String agent) {
		return indexMap.get(agent);
	}
	
	public int getSecondIndex(String agent) {
		return secondIndexMap.get(agent);
	}
	
	public int getThirdIndex(String agent) {
		return indexMap3.get(agent);
	}
}
