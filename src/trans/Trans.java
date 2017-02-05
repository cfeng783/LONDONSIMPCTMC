package trans;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.UUID;

import moment.MomentScripter;
import moment.MomentVariable;
import simulator.RealSimuator;
import utality.Utality;

public class Trans {
	final static int LOG_NORMAL = 101;
	final static int NORMAL = 102;
	final static int MEAN_FIELD = 103;
	
	int closure = NORMAL;
	
	
	String id;
	private int index;
	
	ArrayList<RPItem> reactants;
	ArrayList<RPItem> products;
	double[] constRateFactor;
	ArrayList<RateItem> rateFactors;
	
	private double apparentRate;
	
	private int fireCount;
	
	ArrayList<String> missFactors = new ArrayList<String>();
	
	private ArrayList<Infl> inflList = new ArrayList<Infl>();
	
	public Trans(ArrayList<RPItem> reactants, ArrayList<RPItem> products,
			double constRateFactor, ArrayList<RateItem> rateFactors) {
		this.id = UUID.randomUUID().toString();
		this.reactants = reactants;
		this.products = products;
		this.constRateFactor = new double[1]; 
		this.constRateFactor[0] = constRateFactor;
		this.rateFactors = rateFactors;
		setMissFactors();
	}
	
	public Trans(ArrayList<RPItem> reactants, ArrayList<RPItem> products,
			double[] constRateFactor, ArrayList<RateItem> rateFactors) {
		this.id = UUID.randomUUID().toString();
		this.reactants = reactants;
		this.products = products; 
		this.constRateFactor = constRateFactor;
		this.rateFactors = rateFactors;
		setMissFactors();
	}
	
	private void setMissFactors() {
		Set<String> ratefactSet = new HashSet<String>();
		for(RateItem item: rateFactors) {
			ratefactSet.add(item.getName());
		}
		for(RPItem item: reactants) {
			if(ratefactSet.contains(item.getName()) == false) {
				missFactors.add(item.getName());
			}
		}
		
//		 System.out.println(str);
	}
	
	public void setApparentRate(HashMap<String, Integer> agentMap, int index) {
		for(int i=0; i<reactants.size(); i++) {
			RPItem pi = reactants.get(i);
			int curCount = agentMap.get(pi.getName());
			if(curCount < pi.getCount()) {
				apparentRate = 0;
				return;
			}
		}
		
		double ret = constRateFactor[0];
		if(index < constRateFactor.length) {
			ret = constRateFactor[index];
		}
		
		for(int i=0; i<rateFactors.size(); i++) {
			RateItem ri = rateFactors.get(i);
			int curCount = agentMap.get(ri.getName());
			ret = ret * Math.pow(curCount, ri.getOrder());
		}
		apparentRate = ret;
	}
	
	public void fire(HashMap<String, Integer> agentMap) {
		for(int i=0; i<reactants.size(); i++) {
			RPItem pi = reactants.get(i);
			int curCount = agentMap.get(pi.getName());
			agentMap.put(pi.getName(), curCount-pi.getCount());
		}
		
		for(int i=0; i<products.size(); i++) {
			RPItem pi = products.get(i);
			int curCount = agentMap.get(pi.getName());
			agentMap.put(pi.getName(), curCount+pi.getCount());
		}
//		System.out.println("Trans "+this.toString() + " fired!");
	}
	
	public double getApparentRate() {
		return this.apparentRate;
	}
	
	public int consumed(String agent) {
		for(int i=0; i<this.reactants.size(); i++) {
			if(reactants.get(i).getName().equals(agent)) {
				return reactants.get(i).getCount();
			}
		}
		return 0;
	}
	
	public int produced(String agent) {
		for(int i=0; i<this.products.size(); i++) {
			if(products.get(i).getName().equals(agent)) {
				return products.get(i).getCount();
			}
		}
		return 0;
	}
	
	public boolean isContributor(String agent) {
		for(int i=0; i<this.reactants.size(); i++) {
			if(reactants.get(i).getName().equals(agent)) {
				return true;
			}
		}
		
		for(int i=0; i<this.rateFactors.size(); i++) {
			if(rateFactors.get(i).getName().equals(agent)) {
				return true;
			}
		}
		return false;
	}
	
	public boolean isInvolved(String agent) {
		for(int i=0; i<this.reactants.size(); i++) {
			if(reactants.get(i).getName().equals(agent)) {
				return true;
			}
		}
		
		for(int i=0; i<this.rateFactors.size(); i++) {
			if(rateFactors.get(i).getName().equals(agent)) {
				return true;
			}
		}
		
		for(int i=0; i<this.products.size(); i++) {
			if(products.get(i).getName().equals(agent)) {
				return true;
			}
		}
		
//		for(int i=0; i<this.inflList.size(); i++) {
//			if(inflList.get(i).getAgent().equals(agent)) {
//				return true;
//			}
//		}
		
		return false;
	}
	
	public String getID() {
		return this.id;
	}
	
	public String makeFirstMoment(String agent, HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		String ret = "";
		int change = this.produced(agent) - this.consumed(agent);
		
		if(change < 0) {
			ret += change;
		}else if(change > 0) {
			ret += "+" + change;	
		}else {
			return ret;
		}
		
		ret += "*"+this.constRateStr(indexMap);
		
		ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(this.rateFactors);
//		System.out.println("num of islands:" + islands.size());
		
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			ret += "*(" + this.resolveMomentIsland(island, indexMap, secIndexMap, covIndexMap) +")";
		}
		
		
		return ret;
	}
	
	
	
	public String makeSecondMoment(String agent, HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		String ret = "";
		int change = this.produced(agent) - this.consumed(agent);
		
		if(change == 0) {
			return ret;
		}
		
		ret += "+" + Math.pow(change, 2);
		ret += "*"+this.constRateStr(indexMap);
		ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(this.rateFactors);
		
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			ret += "*(" + this.resolveMomentIsland(island, indexMap, secIndexMap, covIndexMap) +")";
		}
		
		if(change >0) {
			ret += "+2*" + change;
		}else {
			ret += "-2*" + Math.abs(change);
		}
		ret += "*"+this.constRateStr(indexMap);
		
		ArrayList<RateItem> momentfactors = new ArrayList<RateItem>();
		boolean added = false;
		for(int i=0; i<this.rateFactors.size(); i++) {
			if(rateFactors.get(i).getName().equals(agent)) {
				RateItem mf = new RateItem(agent, rateFactors.get(i).getOrder()+1);
				momentfactors.add(mf);
				added = true;
			}else {
				RateItem mf = new RateItem(rateFactors.get(i).getName(), rateFactors.get(i).getOrder());
				momentfactors.add(mf);
			}
		}
		if(added == false) {
			RateItem mf = new RateItem(agent, 1);
			momentfactors.add(mf);
		} 
		
		
		
		islands = this.getMomentIslands(momentfactors);
		
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			ret += "*(" + this.resolveMomentIsland(island, indexMap, secIndexMap, covIndexMap) +")";
		}
		
		String print="";
		for(int i=0; i<momentfactors.size(); i++) {
			print += momentfactors.get(i).getName() + " " + momentfactors.get(i).getOrder() + " ";
		}
//island print
//		System.out.println(print);
		
		for(int i=0;i<momentfactors.size();i++) {
			for(int j=i+1; j<momentfactors.size();j++) {
				int cp =RealSimuator.getGraphBuilder().getCouplingCoefficient(momentfactors.get(i).getName(), momentfactors.get(j).getName());
//				System.out.println(momentfactors.get(i).getName() + " with " + momentfactors.get(j).getName() +" = "+cp);
			}
		}
		
		print="";
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			print += "island " + i + ":";
			for(int j=0; j<island.size(); j++) {
				print += island.get(j).getName() + " ";
			}
		}
//		System.out.println(print);
//		System.out.println();
		return ret;
	}
	
	public String makeCovMoment(String agent1, String agent2, HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		String ret = "";
		int change1 = this.produced(agent1) - this.consumed(agent1);
		int change2 = this.produced(agent2) - this.consumed(agent2);
		if(change1 == 0 && change2 ==0) {
			return ret;
		}
		
		if(change2 != 0) {
			if(change2 > 0)
				ret += "+";
				
			ret += change2;
			ret += "*"+this.constRateStr(indexMap);
			ArrayList<RateItem> momentfactors = new ArrayList<RateItem>();
			boolean added = false;
			for(int i=0; i<this.rateFactors.size(); i++) {
				if(rateFactors.get(i).getName().equals(agent1)) {
					RateItem mf = new RateItem(agent1, rateFactors.get(i).getOrder()+1);
					momentfactors.add(mf);
					added = true;
				}else {
					RateItem mf = new RateItem(rateFactors.get(i).getName(), rateFactors.get(i).getOrder());
					momentfactors.add(mf);
				}
			}
			if(added == false) {
				RateItem mf = new RateItem(agent1, 1);
				momentfactors.add(mf);
			} 
			
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				ret += "*(" + this.resolveMomentIsland(island, indexMap, secIndexMap, covIndexMap) +")";
			}
		}
		
		if(change1 != 0) {
			if(change1 > 0)
				ret += "+";
			
			ret += change1;
			ret += "*"+this.constRateStr(indexMap);
			ArrayList<RateItem> momentfactors = new ArrayList<RateItem>();
			boolean added = false;
			for(int i=0; i<this.rateFactors.size(); i++) {
				if(rateFactors.get(i).getName().equals(agent2)) {
					RateItem mf = new RateItem(agent2, rateFactors.get(i).getOrder()+1);
					momentfactors.add(mf);
					added = true;
				}else {
					RateItem mf = new RateItem(rateFactors.get(i).getName(), rateFactors.get(i).getOrder());
					momentfactors.add(mf);
				}
			}
			if(added == false) {
				RateItem mf = new RateItem(agent2, 1);
				momentfactors.add(mf);
			} 
			
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				ret += "*(" + this.resolveMomentIsland(island, indexMap, secIndexMap, covIndexMap) +")";
			}
		}
		
		if(change1 != 0 && change2 != 0) {
			int product = change1 * change2;
			if(product > 0)
				ret += "+";
			
			ret += product;
			ret += "*"+this.constRateStr(indexMap);
			
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(this.rateFactors);
			
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				ret += "*(" + this.resolveMomentIsland(island, indexMap, secIndexMap, covIndexMap) +")";
			}
		}
		
		
		return ret;
	}
	
	public String make3rdMoment(String agent, HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		String ret = "(";
		if(closure == LOG_NORMAL) {
			ret += "(y(" + secIndexMap.get(agent) + ")/" + "(y(" + indexMap.get(agent) + ")+"
					+ Utality.sigma + "))^3";
		}else if (closure == NORMAL) {
			ret += "(3*y(" + indexMap.get(agent) + ")*y(" + secIndexMap.get(agent) + ")" 
						+ "-2*y(" + indexMap.get(agent) + ")^3)"; 
		} else if (closure == MEAN_FIELD) {
			ret += "y(" + indexMap.get(agent) + ")^3";
		}	
		ret += ")";
		
		return ret;
	}
	
	
	public String make1plus2moment(String st1, String st2, HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		String ret = "(";
		if(closure == LOG_NORMAL) {
			ret += "y(" + secIndexMap.get(st2) + ")*" + "((" + MomentScripter.getCovIndex(st1, st2, covIndexMap,indexMap) + ")^2)";
			ret += "/" + "(y(" + indexMap.get(st1) + ")"+ "+" + Utality.sigma + ")"
					+ "/" + "(y(" + indexMap.get(st2) + ")^2+" + Utality.sigma + ")";
		}else if (closure == NORMAL) {
			ret += "(2*y(" + indexMap.get(st2) + ")*" + MomentScripter.getCovIndex(st1, st2, covIndexMap, indexMap)
						+ "-2*y(" + indexMap.get(st1) + ")*y(" + indexMap.get(st2) + ")^2" 
						+ "+y(" + indexMap.get(st1) + ")*y(" + secIndexMap.get(st2) + "))"; 
			
		} else if (closure == MEAN_FIELD) {
			ret += "y(" + indexMap.get(st1) + ")*y(" + indexMap.get(st2) + ")^2";
		}	
		ret += ")";
		
		return ret;
	}
	
	public String make1plus1plus1moment(String st1, String st2, String st3, HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		String ret = "(";
		if(closure == LOG_NORMAL) {
			ret += MomentScripter.getCovIndex(st1, st2, covIndexMap, indexMap) + "*"
					+ MomentScripter.getCovIndex(st1, st3, covIndexMap, indexMap) + "*" +
					MomentScripter.getCovIndex(st2, st3, covIndexMap, indexMap);
			ret += "/" + "(y(" + indexMap.get(st1) + ")"+ "+" + Utality.sigma + ")"
					+ "/" + "(y(" + indexMap.get(st2) + ")+" + Utality.sigma + ")" 
					+ "/" + "(y(" + indexMap.get(st3) + ")+" + Utality.sigma + ")";
		}else if (closure == NORMAL) {
			ret += "(y(" + indexMap.get(st1) + ")*" + MomentScripter.getCovIndex(st2, st3, covIndexMap, indexMap)
					+ "+y(" + indexMap.get(st2) + ")*" + MomentScripter.getCovIndex(st1, st3, covIndexMap, indexMap)
					+ "+y(" + indexMap.get(st3) + ")*" + MomentScripter.getCovIndex(st1, st2, covIndexMap, indexMap)
					+ "-2*y(" + indexMap.get(st1) + ")*" + "y(" + indexMap.get(st2) + ")*" + "y(" + indexMap.get(st3) + "))";
		}else {
			ret += "y(" + indexMap.get(st1) + ")*y(" + indexMap.get(st2) + ")*y(" + indexMap.get(st3) + ")";
		}	
		ret += ")";		
		return ret;
	}
	
	private String resolveMomentIsland(ArrayList<RateItem> island, HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		String ret = "";
		int totalOrder = 0;
		for(int j=0; j<island.size(); j++) {
			totalOrder += island.get(j).getOrder();
//			System.out.println("total order: " + totalOrder);
		}
		if(totalOrder == 1) {
			ret += "y(" + indexMap.get(island.get(0).getName()) + ")";
		}else if(totalOrder == 2) {
			if(island.size() == 2) {
				ret += MomentScripter.getCovIndex(island.get(0).getName(), island.get(1).getName(), covIndexMap, indexMap);
			}else {
				ret += "y(" + secIndexMap.get(island.get(0).getName()) + ")";
			}
		}else if(totalOrder == 3) {
			if(island.size() == 3) {
				ret += this.make1plus1plus1moment(island.get(0).getName(), island.get(1).getName(), island.get(2).getName(), indexMap, secIndexMap, covIndexMap);
			}else if(island.size() == 2) {
				if(island.get(0).getOrder() == 2) {
					ret += this.make1plus2moment(island.get(1).getName(), island.get(0).getName(), indexMap, secIndexMap, covIndexMap);
				}else if(island.get(1).getOrder() == 2) {
					ret += this.make1plus2moment(island.get(0).getName(), island.get(1).getName(), indexMap, secIndexMap, covIndexMap);
				}
			}else {
				ret += this.make3rdMoment(island.get(0).getName(), indexMap, secIndexMap, covIndexMap);
			}
		}else {
			System.out.println("order larger than 3");//to do 
		}
		return ret;
	}
	
	private ArrayList<ArrayList<RateItem>> getMomentIslands(ArrayList<RateItem> momentItems) {
		ArrayList<ArrayList<RateItem>> islands = new ArrayList<ArrayList<RateItem>>();
		
		ArrayList<MomentVariable> mvList = new ArrayList<MomentVariable>();
		for(int i=0; i<momentItems.size(); i++) {
			MomentVariable mv = new MomentVariable(momentItems.get(i));
			for(int j=0; j<momentItems.size(); j++) {
				if(i == j)	continue;
				if(RealSimuator.getGraphBuilder().getCouplingCoefficient(momentItems.get(i).getName(), momentItems.get(j).getName()) <= MomentScripter.couplingThreshold) {
					mv.addNeighbour(momentItems.get(j).getName());
				}
			}
			mvList.add(mv);
		}
		
		Set<String> visitedSet = new HashSet<String>();
		for(int i=0; i<mvList.size(); i++) {
			MomentVariable mv = mvList.get(i);
			ArrayList<RateItem> island = new ArrayList<RateItem>();
			closure(mv, island, visitedSet, mvList);
			if(island.size() > 0) {
				islands.add(island);
			}
		}
//		if(islands.size() > 1) {
//			System.out.println("island num: " + islands.size() + ", ");
//		}
		return islands;
	}
	
	private void closure(MomentVariable mv, ArrayList<RateItem> island, Set<String> visitedSet, ArrayList<MomentVariable> mvList) {
		if(visitedSet.contains(mv.getName())) {
			return;
		}else {
			visitedSet.add(mv.getName());
			island.add(mv.getHomeVar());
			for(int j=0; j<mvList.size(); j++) {
				if(mv.isNeighbour(mvList.get(j).getName())) {
					closure(mvList.get(j), island, visitedSet, mvList);
				}
			}
		}
	}
	
	public String toString() {
		String ret = "";
		for(int i=0; i<this.reactants.size(); i++) {
			ret += " " + this.reactants.get(i).getCount() + "*" + this.reactants.get(i).getName();
		}
		ret += " -->";
		
		for(int i=0; i<this.products.size(); i++) {
			ret += " " + this.products.get(i).getCount() + "*" + this.products.get(i).getName();
		}
		
		ret += " at ";
		ret += this.constRateFactor[0];
		for(int i=0; i<this.rateFactors.size(); i++) {
			ret += "*" + this.rateFactors.get(i).getName() + "^" + this.rateFactors.get(i).getOrder();
		}
		return ret;
	}
	
	public String constRateStr(HashMap<String, Integer> indexMap) {
		String str = "";
		if(this.constRateFactor.length == 1) {
			str += constRateFactor[0] +"";
		}else {
//			str += "R(t," + this.index + ")";
			str += "R" + this.index + "(t";
			str += ")";
		}
		
//		for(int i=0; i<missFactors.size(); i++) {
//			str += "*((abs(y(" + indexMap.get(missFactors.get(i)) + "))" + "+" + "y(" + indexMap.get(missFactors.get(i)) + "))/(y(" 
//					+ indexMap.get(missFactors.get(i))  + ")*2"+ "+eps))";
//		}
		
//		for(int i=0; i<missFactors.size(); i++) {
//			str += "*(max(y(" + indexMap.get(missFactors.get(i)) + "),0)"+"/(y(" 
//					+ indexMap.get(missFactors.get(i))  + ")"+ "+eps))";
//		}
		
		for(int i=0; i<missFactors.size(); i++) {
			str += "*((y(" + indexMap.get(missFactors.get(i)) + ")-0.01)/(y(" 
					+ indexMap.get(missFactors.get(i))  + ")"+ "+0.01))";
		}
		
//		for(int i=0; i<missFactors.size(); i++) {
//			str += "*y(" + indexMap.get(missFactors.get(i)) + ")" + "/(0.01+y(" + indexMap.get(missFactors.get(i)) + "))";
//		}
		return str;
	}
	
	
	public ArrayList<String> getRateFunction(int[] alterPoints) {
		if(this.constRateFactor.length == 1) {
			return null;
		}
		ArrayList<String> vars = new ArrayList<String>();
		
		String strFunc = "function r = " + " R" + this.index + "(t";
		strFunc += ")";
		vars.add(strFunc);
				
		
		
		for(int i=0; i<alterPoints.length; i++) {
			String str = "";
			if(i==0) {				
				str = "if t<" + alterPoints[i];
				vars.add(str);

				str = "r=" + this.constRateFactor[0] + ";";
				vars.add(str);
			}else {
				str = "elseif t>=" + alterPoints[i-1] + " && " + "t<=" + alterPoints[i]; 
				vars.add(str);
				
				str = "r=" + constRateFactor[i] + ";";
				vars.add(str);
			}
		}
		
		
		vars.add("end");
		return vars;
	}
	
	
	public ArrayList<RPItem> getReactants() {
		return this.reactants;
	}
	public ArrayList<RPItem> getProducts() {
		return this.products;
	}
	public ArrayList<RateItem> getRateFactors() {
		return this.rateFactors;
	}

	public double getFireCount() {
		return fireCount*1.0;
	}

	public void setFireCount(int fireCount) {
		this.fireCount = fireCount;
		
	}

	public int getIndex() {
		return index;
	}

	public void setIndex(int index) {
		this.index = index;
	}
	
	public double[] getConstRateFactor() {
		return this.constRateFactor;
	}

	public ArrayList<Infl> getInflList() {
		return inflList;
	}

	public void addInfl(Infl item) {
		this.inflList.add(item);
	}
	
}
