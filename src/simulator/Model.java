package simulator;


import java.util.ArrayList;
import java.util.HashMap;

import trans.Trans;
import utality.Utality;

public class Model extends Event{
	ArrayList<Trans> transArray;
	HashMap<String, Integer> agentMap;
	HashMap<String, Integer> initAgentMap;
	
	int chosenIndex;
	
	int[] alterPoints;
	boolean isTimeImhomogeneous = false;
	
	public void init(ArrayList<Trans> transArray, HashMap<String, Integer> initAgentMap) {
		this.transArray = transArray;
		this.initAgentMap = initAgentMap;
	}
	
	public void setAlterPoints(int[] alterPoints) {
		this.alterPoints = alterPoints;
		isTimeImhomogeneous = true;
	}
	
	public int[] getAlterPoints() {
		return alterPoints;
	}
	
	public boolean isTimeImhomogeneous() {
		return isTimeImhomogeneous;
	}
	
	
	public void reset(int run) {
		if(run == 0) {
			agentMap = new HashMap<String, Integer>();
		}
		
		for(String key: initAgentMap.keySet()) {
			int value = initAgentMap.get(key);
			agentMap.put(key, value);
		}
	}
	
	public HashMap<String, Integer> getInitAgentMap() {
		return this.initAgentMap;
	}
	
	public HashMap<String, Integer> getAgentMap() {
		return this.agentMap;
	}
	
	public ArrayList<Trans> getTransArray() {
		return this.transArray;
	}
	
	@Override
	void execute(AbstractSimulator simulator) {
		Trans chosenTrans = transArray.get(chosenIndex);
		chosenTrans.fire(agentMap);
		Simulator sim = (Simulator)simulator;
		RealSimuator.getCounter().record(sim.now(), agentMap);
		scheduleNextEvent(sim);
	}
	
	private void initPropensityFuncArray(double now) {
		if(isTimeImhomogeneous) {
			int index = 0;
			for(int i=0; i<alterPoints.length; i++) {
				if(i==0) {
					if(now<alterPoints[i]) {
						index = 0;
						break;
					}
				}else {
					if(now >= alterPoints[i-1] && now <= alterPoints[i]) {
						index = i;
						break;
					}
				}
			}
			
			for(Trans trans: transArray) {
				trans.setApparentRate(agentMap, index);
			}
		}else {
			for(Trans trans: transArray) {
				trans.setApparentRate(agentMap, 0);
			}
		}
		
		
	}
	
	void scheduleNextEvent(Simulator simulator) {
		initPropensityFuncArray(simulator.now());
		double r1 = Utality.getRandom().nextDouble();
		double r2 = Utality.getRandom().nextDouble();
		
		double totalRate = 0;
		for(int i=0; i<transArray.size(); i++) {
			Trans trans = transArray.get(i);
			totalRate += trans.getApparentRate();
		}
		if(totalRate <= 0) {
			return;
		}
		
		double curTotal = 0;
		for(int i=0; i<transArray.size(); i++) {
			Trans trans = transArray.get(i);
			curTotal += trans.getApparentRate();
			if(r2 >= (curTotal-trans.getApparentRate())/totalRate && r2 < curTotal/totalRate) {
				chosenIndex = i;
			}
		}
		
		double deltaT = 1/totalRate * Math.log(1/r1);
		time = simulator.now() + deltaT;
		simulator.insert(this);
	}
	
	public void clear() {
		transArray.clear();
		if(agentMap != null) {
			agentMap.clear();
		}
		
		initAgentMap.clear();
		alterPoints = null;
		isTimeImhomogeneous = false;
	}
}
