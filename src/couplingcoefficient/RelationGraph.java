package couplingcoefficient;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import trans.RPItem;
import trans.RateItem;
import trans.Trans;
import utality.Utality;

public class RelationGraph {
	private HashMap<String, RelationEdge> edgeMap = new HashMap<String, RelationEdge>();
	private ArrayList<String> agentList;
	//private int[] transFireCount;
	//private HashMap<String, Integer> transIndexMap = new HashMap<String, Integer>();
	
	private HashMap<String, Set<Trans>> agentTransMap = new HashMap<String, Set<Trans>>(); //key agent, value trans directed involved
	
	private Set<String> pairSet = new HashSet<String>(); //agent_agent
	
	public void buildGraph(ArrayList<String> agentList, ArrayList<Trans> transArray) {
		this.agentList = agentList;
		int i=0;
		for(Trans trans: transArray) {
			//transIndexMap.put(trans.getID(), i);
			trans.setIndex(i);
			i++;
			Set<String> involvedAgents = new HashSet<String>();
			for(RPItem item: trans.getReactants()) {
				involvedAgents.add(item.getName());
			}
			
			for(RPItem item: trans.getProducts()) {
				involvedAgents.add(item.getName());
			}
			
			for(RateItem item: trans.getRateFactors()) {
				involvedAgents.add(item.getName());
			}
			String[] invAgts = involvedAgents.toArray(new String[0]);
			for(int k=0; k<invAgts.length; k++) {
				if(agentTransMap.get(invAgts[k]) == null) {
					Set<Trans> transSet = new HashSet<Trans>();
					transSet.add(trans);
					agentTransMap.put(invAgts[k], transSet);
				}else {
					agentTransMap.get(invAgts[k]).add(trans);
				}
				for(int j=k+1; j<invAgts.length; j++) {
					String pairKey1 = invAgts[k] + "_" + invAgts[j];
					String pairKey2 = invAgts[j] + "_" + invAgts[k];
					if(pairSet.contains(pairKey1) == false && pairSet.contains(pairKey2) == false) {
						pairSet.add(pairKey1);
						pairSet.add(pairKey2);
					}
				}
			}
			
		}
		
		init();
//		System.out.println("edge count: " + edgeMap.keySet().size());
	}
	
	private void init() {
		for(String pairKey: pairSet) {
			String[] agents = pairKey.split("_");
			
			String Influence="";
			String Consumption = "";
			String Production = "";
			
			for(Trans trans: agentTransMap.get(agents[0])) {
				if(trans.consumed(agents[0]) > 0) {
					if(Consumption.length() > 0) {
						Consumption += "+";
					}
					Consumption += trans.consumed(agents[0]) + "*" + trans.getFireCount();
					if(trans.isInvolved(agents[1])) {
						Influence += "-" + trans.consumed(agents[0]) + "*" + trans.getFireCount();
					}
				}
				if(trans.produced(agents[0]) > 0) {
					if(Production.length() > 0) {
						Production += "+";
					}
					Production += trans.produced(agents[0]) + "*" + trans.getFireCount();
					if(trans.isInvolved(agents[1])) {
						Influence += "+" + trans.produced(agents[0]) + "*" + trans.getFireCount();
					}
				}
				
				if(Influence.length()>0) {
					String denominator = "";
					if(Consumption.length()>0 && Production.length()>0) {
						denominator = "max(" + Consumption + "," + Production + ")";
					}else if (Consumption.length()<=0) {
						denominator = "max(" + 0 + "," + Production + ")";
					}else if (Production.length()<=0) {
						denominator = "max(" + Consumption + "," + 0 + ")";
					}
					String weight = "abs(" + Influence + ")/" + denominator;
					String numerator = "abs(" + Influence + ")";
					RelationEdge rE = new RelationEdge(agents[0], agents[1], weight, numerator, denominator);
					//System.out.println(rE.toString());
					edgeMap.put(RelationEdge.genEdgeKey(agents[0], agents[1]), rE);
				}
			
			}
		}
		
	}
	
	public void evalEdgeWeights() {
		
		for(String key: edgeMap.keySet()) {
			RelationEdge edge = edgeMap.get(key);
			
			double numerator = Utality.evalDRGExp(edge.getNumerator());
			double denominator = Utality.evalDRGExp(edge.getDenominator());
			double value = 0;
			if(denominator == 0 ) {
				value = 0;
				//edge.setValue(0);
			}else {
				value = numerator/denominator;
				//edge.setValue(numerator/denominator);
			}
			//edge.getSampleValues().add(value);
			//if(value > edge.getValue()) {
			edge.setValue(value);
			//}
			
//			System.out.println(edge.getWeight());
//			System.out.println(edge.toString());
		}
	}
		
	
	public HashMap<String, RelationEdge> getEdgeMap() {
		return edgeMap;
	}
	public void setEdgeMap(HashMap<String, RelationEdge> edgeMap) {
		this.edgeMap = edgeMap;
	}
	
}
