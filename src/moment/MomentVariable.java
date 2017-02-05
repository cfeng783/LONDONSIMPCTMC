package moment;

import java.util.HashSet;
import java.util.Set;

import trans.RateItem;

public class MomentVariable {
	RateItem homeVar;
	Set<String> neighbours;
	
	
	public MomentVariable(RateItem homeVar) {
		this.homeVar = homeVar;
		neighbours = new HashSet<String>();
	}
	
	public void addNeighbour(String var) {
		neighbours.add(var);
	}
	
	public void addNeighbour(RateItem var) {
		neighbours.add(var.getName());
	}
	
	public boolean isNeighbour(String var) {
		return neighbours.contains(var);
	}
	
	public boolean isNeighbour(RateItem var) {
		return neighbours.contains(var.getName());
	}
	
	public String getName() {
		return homeVar.getName();
	}
	
	public RateItem getHomeVar() {
		return homeVar;
	}
}
