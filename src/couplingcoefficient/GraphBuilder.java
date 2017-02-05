package couplingcoefficient;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import simulator.RealSimuator;
import trans.Trans;



public class GraphBuilder {
	public final static int NONE_NEIGHBOR = 4;
	public final static int ONE_HOP_NEIGHBOR = 1;
	public final static int TWO_HOP_NEIGHBOR = 2;
	public final static int THREE_HOP_NEIGHBOR = 3;
	public final static int ZERO_HOP_NEIGHBOR = 0;
//	private RelationGraph DRG;
    private int[][] ccMatrix;
	private ArrayList<String> agentList;
	private HashMap<String, Integer> agentIndexMap;
	
	HashMap<String, Set<String>> neighborMap = new HashMap<String, Set<String>>();
	
	public GraphBuilder() {
//		DRG = new RelationGraph();
		this.agentList = new ArrayList<String>();
	}
	
	public void init(HashMap<String, Integer> initAgentMap,ArrayList<Trans> transArray) {
		for(String agent: initAgentMap.keySet()) {
			agentList.add(agent);
			Set<String> set = new HashSet<String>();
			neighborMap.put(agent, set);
		}
		
		initNeigborMap(agentList, transArray);
		makeGraph();
		//DRG.buildGraph(agentList, transArray);
		
		agentIndexMap = new HashMap<String, Integer>();
		int i=0;
		for(String agent:  agentList) {
			agentIndexMap.put(agent, i);
			i++;
		}
	}
	
	private void initNeigborMap(ArrayList<String> agentList, ArrayList<Trans> transArray) {
		for(int i=0; i<agentList.size(); i++) {
			for(int j=0; j<agentList.size(); j++) {
				String agent1 = agentList.get(i);
				String agent2 = agentList.get(j);
				if(!agent1.equals(agent2) && i<j) {
					for(Trans pt: transArray) {
						if(pt.isInvolved(agent1) && pt.isInvolved(agent2)) {
							neighborMap.get(agent1).add(agent2);
							neighborMap.get(agent2).add(agent1);
							break;
						}
					}
				}
			}
		}
		
	}
	
	
	private void makeGraph() {
		ccMatrix = new int[agentList.size()][agentList.size()];
		for(int i=0; i<agentList.size(); i++) {
			ccMatrix[i][i] = ZERO_HOP_NEIGHBOR;
		}
		
		for(int i=0; i<agentList.size(); i++) {
			for(int j=i+1; j<agentList.size(); j++) {
				String agent1 = agentList.get(i);
				String agent2 = agentList.get(j);
				int correlation = checkCorrelation(agent1, agent2, ONE_HOP_NEIGHBOR);
				ccMatrix[i][j] = correlation;
				ccMatrix[j][i] = correlation; 
				
			}
		}
		
//		for(int i=0; i<agentList.size(); i++) {
//			for(int j=0; j<agentList.size(); j++) {
//				String agent1 = agentList.get(i);
//				String agent2 = agentList.get(j);
//				String loc1 = agent1.substring(agent1.indexOf("(")+1, agent1.indexOf(")"));
//				String loc2 = agent2.substring(agent2.indexOf("(")+1, agent2.indexOf(")"));
//				int index1 = Integer.parseInt(loc1);
//				int index2 = Integer.parseInt(loc2);
//				ccMatrix[i][j] = dependenceMatrix[index1][index2];
//				if(index1 == index2) {
//					ccMatrix[i][j] = 1;
//				}
//			}
//		}
		
//		for(String key: DRG.getEdgeMap().keySet()) {
//			RelationEdge edge = DRG.getEdgeMap().get(key);
//			Integer headIndex = agentIndexMap.get(edge.getHeadNodeName());
//			Integer tailIndex = agentIndexMap.get(edge.getTailNodeName());
//			if(headIndex != null && tailIndex != null) {
//				ccMatrix[headIndex][tailIndex] = edge.getValue();
//			}
//		}
//		
//
//		for(int k=0; k<agentList.size(); k++) {
//			if(k%100 == 0) {
//				System.out.println("K="+k);
//			}
//			for(int i=0; i<agentList.size(); i++) {
//				for(int j=0; j<agentList.size(); j++) {
//					if(ccMatrix[i][j] < ccMatrix[i][k] * ccMatrix[k][j]) {
//						ccMatrix[i][j] = ccMatrix[i][k] * ccMatrix[k][j];
//					}
//				}
//			}
//		}

		
		
	}
	
	private int checkCorrelation(String st1, String st2,  int depth) {
		if(depth == NONE_NEIGHBOR) {
			return NONE_NEIGHBOR;
		}
		
		if(neighborMap.get(st1).contains(st2) || neighborMap.get(st2).contains(st1)) {
			return depth;
		}
		
		int min = 1000;
		
		for(String st: neighborMap.get(st1)) {
			int value = checkCorrelation(st, st2, depth+1);
			if( value < min){
				min = value;
			}
		}
		
		for(String st: neighborMap.get(st2)) {
			int value = checkCorrelation(st1, st, depth+1);
			if( value < min){
				min = value;
			}
		}
			
		return min;
	}
	
	
//	public RelationGraph getRelationGraph() {
//		return DRG;
//	}
	
	
	public  int[][] getccMatrix() {
		return ccMatrix;
	}
	
//	public int getAgentIndex(String agent) {
//		return agentIndexMap.get(agent);
//	}
//	
	public int getCouplingCoefficient(String agt1, String agt2) {
		int i = agentIndexMap.get(agt1);
		int j = agentIndexMap.get(agt2);
		int ret = Math.max(ccMatrix[i][j], ccMatrix[j][i]);
//		ret = (ccMatrix[i][j]+ccMatrix[j][i])/2;
		return ret;
	}
	
	public void clear() {
		ccMatrix = null;
		agentList.clear();;
		agentIndexMap.clear();
		neighborMap.clear();
	}
}
