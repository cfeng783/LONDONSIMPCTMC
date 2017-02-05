package trans;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import simulator.RealSimuator;
import moment.MomentGenerator;
import moment.MomentVariable;

public class FluidTrans {

	ArrayList<RPItem> reactants;
	ArrayList<RPItem> products;
	double[] constRateFactor;
	ArrayList<RateItem> rateFactors;
	
	private ArrayList<String> missFactors;
	
	private ArrayList<Guard> guards = new ArrayList<Guard>();
	private ArrayList<Infl> inflList;
	
	public FluidTrans(Trans trans) {
		this.reactants = trans.reactants;
		this.products = trans.products;
		this.constRateFactor = trans.constRateFactor; 
		this.rateFactors = trans.rateFactors;
		this.missFactors = trans.missFactors;
		this.inflList = trans.getInflList();
		
		for(String agent: missFactors) {
			Guard g = new Guard(agent, Guard.LESS_EQUAL, 0);
			guards.add(g);
		} 
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
	
	
	public double makeFirstMoment(double t, double[] y, int[] alterPoints,
			String agent, HashMap<String, Integer> indexMap, 
			HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		double ret = 0;
		int change = this.produced(agent) - this.consumed(agent);
		
		if(change == 0) {
			return ret;
		}
		
		if(this.checkGuards(indexMap, y) == true) {
			return 0;
		}
		
		ret = change * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
		
		ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(this.rateFactors);
//		System.out.println("num of islands:" + islands.size());
		
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			ret = ret * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap);
		}
		
		return ret;
	}
	
	
	public double makeSecondMoment(double t, double[] y, int[] alterPoints,
			String agent, HashMap<String, Integer> indexMap, 
			HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		double ret = 0;
		int change = this.produced(agent) - this.consumed(agent);
		
		if(change == 0) {
			return ret;
		}
		
		if(this.checkGuards(indexMap, y) == true) {
			return 0;
		}
		
		ret = Math.pow(change, 2);
		ret = ret * this.constRate(t,y,indexMap,secIndexMap, alterPoints);
		ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(this.rateFactors);
		
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			ret = ret*this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap);
		}
		
		double term =  2*change*this.constRate(t,y,indexMap,secIndexMap, alterPoints);
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
			term = term * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap);
		}
		
		ret += term;
		return ret;
	}
	
	public double makeCovMoment(double t, double[] y, int[] alterPoints,
			String agent1, String agent2, HashMap<String, Integer> indexMap, 
			HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		double ret = 0;
		int change1 = this.produced(agent1) - this.consumed(agent1);
		int change2 = this.produced(agent2) - this.consumed(agent2);
		if(change1 == 0 && change2 ==0) {
			return 0;
		}
		
		
		if(this.checkGuards(indexMap, y) == true) {
			return 0;
		}
		
		if(change2 != 0) {
			double term1 = change2*this.constRate(t,y,indexMap,secIndexMap, alterPoints);
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
				term1 = term1 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap);
			}
			ret += term1;
		}
		
		if(change1 != 0) {
			double term2 = change1 * this.constRate(t,y,indexMap,secIndexMap, alterPoints);
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
				term2 = term2 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap);
			}
			ret += term2;
		}
		
		if(change1 != 0 && change2 != 0) {
			double term3 = change1 * change2 * this.constRate(t,y,indexMap,secIndexMap, alterPoints);
			
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(this.rateFactors);
			
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term3 = term3 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap);
			}
			ret += term3;
		}
		
		return ret;
	}
	

	public boolean checkSingleMoment(String agent) {
		int change = this.produced(agent) - this.consumed(agent);
		if(change == 0) {
			return false;
		}else {
			return true;
		}
		
	}
	
	public boolean checkCovMoment(String agent1, String agent2) {
		int change1 = this.produced(agent1) - this.consumed(agent1);
		int change2 = this.produced(agent2) - this.consumed(agent2);
		if(change1 == 0 && change2 ==0) {
			return false;
		} else {
			return true;
		}
	}
	
	public boolean checkCovMoment(String agent1, String agent2, String agent3) {
		int change1 = this.produced(agent1) - this.consumed(agent1);
		int change2 = this.produced(agent2) - this.consumed(agent2);
		int change3 = this.produced(agent3) - this.consumed(agent3);
		if(change1 == 0 && change2 ==0 && change3 == 0) {
			return false;
		} else {
			return true;
		}
	}
	
	public double make3rdMoment(double t, double[] y, int[] alterPoints,
			String agent, HashMap<String, Integer> indexMap, 
			HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap,
			HashMap<String, Integer> indexMap111, HashMap<String, Integer> indexMap21, HashMap<String, Integer> indexMap3) 
	{
		double ret = 0;
		int change = this.produced(agent) - this.consumed(agent);
		
		if(change == 0) {
			return ret;
		}
		
		double term1 = 3*change * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
		
		ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
		otherFactors.add(new RateItem(agent, 2));
		ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
		
		ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			term1 = term1 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
		}
		
		double term2 = 3*Math.pow(change, 2) * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
		otherFactors.clear();
		momentfactors.clear();
		islands.clear();
		otherFactors.add(new RateItem(agent, 1));
		momentfactors = this.getMomentfactors(otherFactors);
		islands = this.getMomentIslands(momentfactors);
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			term2 = term2 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
		}
		
		double term3 = Math.pow(change, 3) * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
		otherFactors.clear();
		momentfactors.clear();
		islands.clear();
		momentfactors = this.getMomentfactors(otherFactors);
		islands = this.getMomentIslands(momentfactors);
		for(int i=0; i<islands.size(); i++) {
			ArrayList<RateItem> island = islands.get(i);
			term3 = term3 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
		}
		
		ret = term1 + term2 + term3;
		return ret;
						
	}
	
	
	
	
	public double make2plus1moment(double t, double[] y, int[] alterPoints,
			String agent1, String agent2, HashMap<String, Integer> indexMap, 
			HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap,
			HashMap<String, Integer> indexMap111, HashMap<String, Integer> indexMap21, HashMap<String, Integer> indexMap3) {
		double ret = 0;
		int change1 = this.produced(agent1) - this.consumed(agent1);
		int change2 = this.produced(agent2) - this.consumed(agent2);
		if(change1 == 0 && change2 ==0) {
			return 0;
		}
		
		if(this.checkGuards(indexMap, y) == true) {
			return 0;
		}
		
		if(change1 != 0) {
			double term1 = 2*change1 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent1, 1));
			otherFactors.add(new RateItem(agent2, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term1 = term1 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term1;
		}
		
		if(change1 != 0) {
			double term2 = Math.pow(change1, 2) * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent2, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term2 = term2 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term2;
		}
		
		
		if(change2 != 0) {
			double term3 = change2 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent1, 2));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term3 = term3 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term3;
		}
		
		if(change1 !=0 && change2 != 0) {
			double term4 = 2*change1*change2 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent1, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term4 = term4 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term4;
		}
		
		if(change1 !=0 && change2 != 0) {
			double term5 = Math.pow(change1,2)*change2 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term5 = term5 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term5;
		}
		
		return ret;
	}
	
	public double make1plus1plus1moment(double t, double[] y, int[] alterPoints,
			String agent1, String agent2, String agent3, HashMap<String, Integer> indexMap, 
			HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap,
			HashMap<String, Integer> indexMap111, HashMap<String, Integer> indexMap21, HashMap<String, Integer> indexMap3) 
	{
		double ret = 0;
		int change1 = this.produced(agent1) - this.consumed(agent1);
		int change2 = this.produced(agent2) - this.consumed(agent2);
		int change3 = this.produced(agent3) - this.consumed(agent3);
		if(change1 == 0 && change2 ==0 && change3 == 0) {
			return 0;
		}
		
		if(this.checkGuards(indexMap, y) == true) {
			return 0;
		}
		
		if(change1 != 0) {
			double term1 = change1 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent2, 1));
			otherFactors.add(new RateItem(agent3, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term1 = term1 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term1;
		}
		
		if(change2 != 0) {
			double term2 = change2 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent1, 1));
			otherFactors.add(new RateItem(agent3, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term2 = term2 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term2;
		}
		
		
		if(change3 != 0) {
			double term3 = change3 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent1, 1));
			otherFactors.add(new RateItem(agent2, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term3 = term3 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term3;
		}
		
		if(change1 !=0 && change2 != 0) {
			double term4 = change1*change2 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent3, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term4 = term4 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term4;
		}
		
		if(change1 !=0 && change3 != 0) {
			double term5 = change1*change3 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent2, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term5 = term5 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term5;
		}
		
		if(change2 !=0 && change3 != 0) {
			double term6 = change2*change3 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			otherFactors.add(new RateItem(agent1, 1));
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term6 = term6 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term6;
		}
		
		if(change1 != 0 && change2 !=0 && change3 != 0) {
			double term7 = change1*change2*change3 * this.constRate(t, y, indexMap, secIndexMap, alterPoints);
			ArrayList<RateItem> otherFactors = new ArrayList<RateItem>();
			ArrayList<RateItem> momentfactors = this.getMomentfactors(otherFactors);
			ArrayList<ArrayList<RateItem>> islands = this.getMomentIslands(momentfactors);
			for(int i=0; i<islands.size(); i++) {
				ArrayList<RateItem> island = islands.get(i);
				term7 = term7 * this.resolveMomentIsland(y, island, indexMap, secIndexMap, covIndexMap,indexMap111, indexMap21, indexMap3);
			}
			ret += term7;
		}
		
		return ret;
	}
	
	
	private ArrayList<RateItem> getMomentfactors(ArrayList<RateItem> otherFactors) {
		ArrayList<RateItem> momentfactors = new ArrayList<RateItem>();
		for(RateItem item: this.rateFactors) {
			RateItem mf = new RateItem(item.getName(), item.getOrder());
			momentfactors.add(mf);
		}
		
		for(RateItem otherFactor: otherFactors) {
			boolean added = false;
			for(int i=0; i<momentfactors.size(); i++) {
				if(momentfactors.get(i).getName().equals(otherFactor.getName())) {
					momentfactors.get(i).setOrder(momentfactors.get(i).getOrder() + otherFactor.getOrder());
					added = true;
				}
			}
			if(added == false) {
				RateItem mf = new RateItem(otherFactor.getName(), otherFactor.getOrder());
				momentfactors.add(mf);
			} 
		}
		return momentfactors;
		
	}
	
	private double resolveMomentIsland(double[] y, ArrayList<RateItem> island, 
			HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap,
			HashMap<String, Integer> indexMap111, HashMap<String, Integer> indexMap21, HashMap<String, Integer> indexMap3) {
		double ret = 0;
		int totalOrder = 0;
		for(int j=0; j<island.size(); j++) {
			totalOrder += island.get(j).getOrder();
//			System.out.println("total order: " + totalOrder);
		}
		if(totalOrder == 1) {
			ret = y[indexMap.get(island.get(0).getName())];
		}else if(totalOrder == 2) {
			if(island.size() == 2) {
				ret = MomentGenerator.getCovMoment(y,island.get(0).getName(), island.get(1).getName(), covIndexMap);
			}else {
				ret =  y[secIndexMap.get(island.get(0).getName())];
			}
		}else if(totalOrder == 3) {
			if(island.size() == 3) {
				ret = MomentGenerator.get111moment(y, island.get(0).getName(), island.get(1).getName(), island.get(2).getName(), indexMap111);
			}else if(island.size() == 2) {
				if(island.get(0).getOrder() == 2) {
					ret = y[indexMap21.get(MomentGenerator.makeCovKey(island.get(0).getName(), island.get(1).getName()))];
				}else if(island.get(1).getOrder() == 2) {
					ret = y[indexMap21.get(MomentGenerator.makeCovKey(island.get(1).getName(), island.get(0).getName()))];
				}
			}else {
				ret = y[indexMap3.get(island.get(0).getName())];
			}
		}else {
			System.err.println("order larger than 3");//to do 
		}
		return ret;
	}
	
	private double resolveMomentIsland(double[] y, ArrayList<RateItem> island, 
			HashMap<String, Integer> indexMap, HashMap<String, Integer> secIndexMap, HashMap<String, Integer> covIndexMap) {
		double ret = 0;
		int totalOrder = 0;
		for(int j=0; j<island.size(); j++) {
			totalOrder += island.get(j).getOrder();
//			System.out.println("total order: " + totalOrder);
		}
		if(totalOrder == 1) {
			ret = y[indexMap.get(island.get(0).getName())];
		}else if(totalOrder == 2) {
			if(island.size() == 2) {
				ret = MomentGenerator.getCovMoment(y,island.get(0).getName(), island.get(1).getName(), covIndexMap);
			}else {
				ret =  y[secIndexMap.get(island.get(0).getName())];
			}
		}else {
			System.err.println("order larger than 2");//to do 
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
				if(RealSimuator.getGraphBuilder().getCouplingCoefficient(momentItems.get(i).getName(), momentItems.get(j).getName()) <= MomentGenerator.couplingThreshold) {
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
	
	public double constRate(double t, double[] y, HashMap<String, Integer> indexMap, HashMap<String, Integer> secondIndexMap, int[] alterPoints) {
		double ret = 0;
		if(this.constRateFactor.length == 1) {
			ret =  constRateFactor[0];
		}else {
			for(int i=0; i<alterPoints.length; i++) {
				if(i==0) {
					if(t<alterPoints[i]) {
						ret = constRateFactor[0];
						for(int k=0; k<this.inflList.size(); k++) {
							ret -= inflList.get(k).getInfluence(0, y, indexMap, secondIndexMap);
						}
						break;
					}
				}else {
					if(t >= alterPoints[i-1] && t <= alterPoints[i]) {
						ret = constRateFactor[i];
						for(int k=0; k<this.inflList.size(); k++) {
							ret -= inflList.get(k).getInfluence(i, y, indexMap, secondIndexMap);
						}
						break;
					}
				}
			}
		}

		return ret;
	}
	
	public boolean checkGuards(HashMap<String, Integer> indexMap, double[] y) {
		if(this.guards.size() == 0) {
			return false;
		}
		
		for(Guard g: this.guards) {
			int index = indexMap.get(g.getAgent());
			if(g.check(y[index]) == true) {
				return true;
			}
		}
		
		return false;
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

	
	public double[] getConstRateFactor() {
		return this.constRateFactor;
	}

//	public ArrayList<Integer> getODEIndices() {
//		return ODEIndices;
//	}
//
//	public void setODEIndices(ArrayList<Integer> oDEIndices) {
//		ODEIndices = oDEIndices;
//	}
	
}
