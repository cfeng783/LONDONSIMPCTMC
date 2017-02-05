package trans;

import java.util.HashMap;

import distribution.ApproximateDistribution;

public class Infl {
	private double[] rates;
	private String agent;
	private int capacity;
	
	public Infl(String agent, int capacity, double[] rates) {
		this.agent = agent;
		this.capacity = capacity;
		this.rates = rates;
	}
	
	public double getInfluence(int slot, double[] y, 
			HashMap<String, Integer> indexMap,  HashMap<String, Integer> secondIndexMap) {
		if(secondIndexMap != null) {
			double u1 = y[indexMap.get(agent)];
			double u2 = y[secondIndexMap.get(agent)];
			double beta[] = ApproximateDistribution.generateBetaBinomial(u1,u2, capacity);
			double emptyPro = ApproximateDistribution.getBetaBinomialProb(beta[0], beta[1], capacity);
			if(emptyPro!=emptyPro || emptyPro < 0.6) {
				return 0;
			}else {
				return rates[slot];
			}
		}else {
			double u1 = y[indexMap.get(agent)];
			double p = ApproximateDistribution.generateBinomial(u1, capacity);
			double emptyPro = ApproximateDistribution.getBinomialProb(p, capacity);
			if(emptyPro!=emptyPro || emptyPro < 0.6) {
				return 0;
			}else {
				return rates[slot];
			}
		}
		
		
	}
	
	
	public double[] getRates() {
		return rates;
	}
	public void setRates(double[] rates) {
		this.rates = rates;
	}
	public String getAgent() {
		return agent;
	}
	public void setAgent(String agent) {
		this.agent = agent;
	}
	public int getCapacity() {
		return capacity;
	}
	public void setCapacity(int capacity) {
		this.capacity = capacity;
	}
}
