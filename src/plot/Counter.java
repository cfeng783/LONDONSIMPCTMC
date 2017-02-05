package plot;


import java.util.ArrayList;
import java.util.HashMap;

import simulator.RealSimuator;


public class Counter {
	final static double DEFAULT_TIME_POINTS = 199.0;
	double finalTime;
	int runs;
	
	double[] timeTrajectory;
	double[][] populationTrajectory;
	double[][] varianceTrajectory;
	double[][] extraTrajectory; //for combined variables
	
	ArrayList<Double> time = new ArrayList<Double>();
	ArrayList<ArrayList<Double>> population = new ArrayList<ArrayList<Double>>();
	
	ArrayList<ArrayList<Double>> statistics_mean = new ArrayList<ArrayList<Double>>();
	ArrayList<ArrayList<Double>> statistics_variance = new ArrayList<ArrayList<Double>>();
	
	
	//theses two variable count for number of agents of in specific states
	private HashMap<String, Integer> agentIndexMap = new HashMap<String, Integer>();
	
	
	ArrayList<String> extraMeanVars = new ArrayList<String>();
	ArrayList<String> extraSecMmtVars = new ArrayList<String>();
	HashMap<String, ArrayList<String>> extraMap = new HashMap<String, ArrayList<String>>();
	
	private HashMap<String, Integer> extraMeanVarIndexMap = new HashMap<String, Integer>();
	private HashMap<String, Integer> extraSecMmtVarIndexMap = new HashMap<String, Integer>();
	
	ArrayList<ArrayList<Double>> statistics_extra = new ArrayList<ArrayList<Double>>();
	
	double[] lastRecPopu;
	double timeStep;
	
	double currentStep=0;
	
	public Counter() {}
	
	public void setUp(int runs, double finalTime, HashMap<String, Integer> initAgentMap, ArrayList<String> extraVars) {
		this.finalTime = finalTime;
		this.runs = runs;
		timeStep = this.finalTime/DEFAULT_TIME_POINTS;
		setupAgentIndexMap(initAgentMap);
		setExtraVars(extraVars);
	}
	
	private void setupAgentIndexMap(HashMap<String, Integer> initAgentMap) {
		int i = 0;
		for(String key: initAgentMap.keySet()) {
			agentIndexMap.put(key, i);
			i++;
			ArrayList<Double> pop = new ArrayList<Double>();
			population.add(pop);
			ArrayList<Double> po2 = new ArrayList<Double>();
			statistics_mean.add(po2);
			ArrayList<Double> po3 = new ArrayList<Double>();
			statistics_variance.add(po3);
		}
		lastRecPopu = new double[i];
	}
	
	private void setExtraVars(ArrayList<String> extraVars) {
		int ind = 0;
		for(String var: extraVars) {
				extraMeanVars.add(var);
				extraMeanVarIndexMap.put(var, ind);
				ind ++;
				statistics_extra.add(new ArrayList<Double>());
				if(extraMap.get(var) == null) {
					ArrayList<String> mapArray = new ArrayList<String>();
					for(String key: agentIndexMap.keySet()) {
						if(key.substring(0, key.indexOf("(")).equals(var)) {
							mapArray.add(key);
						}
					}
					extraMap.put(var, mapArray);
				}
		}
	
		for(String var: extraVars) {
			extraSecMmtVars.add(var);
			extraSecMmtVarIndexMap.put(var, ind);
			ind++;
			statistics_extra.add(new ArrayList<Double>());
			if(extraMap.get(var) == null) {
				ArrayList<String> mapArray = new ArrayList<String>();
				for(String key: agentIndexMap.keySet()) {
					if(key.substring(0, key.indexOf("(")).equals(var)) {
						mapArray.add(key);
					}
				}
				extraMap.put(var, mapArray);
			}
			
		}
	}

	public int getRuns() {
		return this.runs;
	}
	
	public double getFinalTime() {
		return this.finalTime;
	}
	
	public void preRun(int curRun, HashMap<String, Integer> agentMap) {
		currentStep = 0;
		for(int i=0; i<lastRecPopu.length; i++) {
			lastRecPopu[i]=0;
		}
		if(curRun==0) {
		}else{
			//System.out.println("time size before run: " + time.size());
			time.clear();
			for(int i=0; i<population.size();i++) {
				population.get(i).clear();
			}
		}
		record(0, agentMap);
	}
	
	public void afterRun(int curRun){
		while(time.size()<DEFAULT_TIME_POINTS+1) {
			double t = time.get(time.size()-1);
			t += timeStep;
			//time.add(t);
			time.add(t);
		}
		//System.out.println("time: " + time.size());
		
		for(int i=0; i<population.size();i++) {
			while(population.get(i).size()<DEFAULT_TIME_POINTS+1) {
				population.get(i).add(lastRecPopu[i]);
			}
			//System.out.println("population "+i + ": " + population.get(i).size());
		}
		
		//System.out.println("time size after run:"+time.size());
		if(curRun == 0) {
			//System.out.println("population size after run:"+population.get(0).size());
			for(int i=0; i<population.size();i++) {
				for(int j=0;j<population.get(i).size();j++) {
					double num = population.get(i).get(j);
					//System.out.println(num);
					statistics_mean.get(i).add(num);
					statistics_variance.get(i).add(Math.pow(num,2));
				}
			}
			
			for(int i=0; i<extraMeanVars.size(); i++) {
				ArrayList<String> childVars = extraMap.get(extraMeanVars.get(i));
				
				for(int j=0; j<population.get(0).size(); j++) {
					double num = 0;
					for(String child: childVars) {
						num += population.get(agentIndexMap.get(child)).get(j);
					}
					statistics_extra.get(extraMeanVarIndexMap.get(extraMeanVars.get(i))).add(num);
				}
			}
			
			for(int i=0; i<extraSecMmtVars.size(); i++) {
				ArrayList<String> childVars = extraMap.get(extraSecMmtVars.get(i));
				
				for(int j=0; j<population.get(0).size(); j++) {
					double num = 0;
					for(String child: childVars) {
						num += population.get(agentIndexMap.get(child)).get(j);
					}
					statistics_extra.get(extraSecMmtVarIndexMap.get(extraSecMmtVars.get(i))).add(Math.pow(num, 2));
				}
			}
			
		}else {
			//System.out.println("population size after run:"+population.get(0).size());
			for(int i=0; i<population.size();i++) {
				for(int j=0;j<population.get(i).size();j++) {
					double num = population.get(i).get(j);
					statistics_variance.get(i).set(j,Math.pow(num, 2)+statistics_variance.get(i).get(j));
					double real = (num+statistics_mean.get(i).get(j)*curRun)/(curRun+1); 
					statistics_mean.get(i).set(j, real);
				}
			}
			
			for(int i=0; i<extraMeanVars.size(); i++) {
				ArrayList<String> childVars = extraMap.get(extraMeanVars.get(i));
				
				for(int j=0; j<population.get(0).size(); j++) {
					double num = 0;
					for(String child: childVars) {
						num += population.get(agentIndexMap.get(child)).get(j);
					}
					double real = num+statistics_extra.get(extraMeanVarIndexMap.get(extraMeanVars.get(i))).get(j);
					statistics_extra.get(extraMeanVarIndexMap.get(extraMeanVars.get(i))).set(j, real);
				}
			}
			
			for(int i=0; i<extraSecMmtVars.size(); i++) {
				ArrayList<String> childVars = extraMap.get(extraSecMmtVars.get(i));
				
				for(int j=0; j<population.get(0).size(); j++) {
					double num = 0;
					for(String child: childVars) {
						num += population.get(agentIndexMap.get(child)).get(j);
					}
					statistics_extra.get(extraSecMmtVarIndexMap.get(extraSecMmtVars.get(i))).set(j, Math.pow(num,2)+statistics_extra.get(extraSecMmtVarIndexMap.get(extraSecMmtVars.get(i))).get(j));
				}
			}
			
		}
//		boolean breaked = false;
//		if(curRun > 100 && curRun%10==0) {
//			for(int i=0; i<statistics_mean.size(); i++) {
//				if(i != agentIndexMap.get("Bike1@6") && i != agentIndexMap.get("Bike1@5")) {
//					continue;
//				}
//				for(int j=20; j<statistics_mean.get(i).size(); j++) {
//					double mean = statistics_mean.get(i).get(j);
//					double std = statistics_variance.get(i).get(j) / (curRun+1)-Math.pow(mean, 2);
//					std = Math.sqrt(std);
//					double se = std / Math.sqrt(curRun+1);
//					double ci = 1.96*se;
//					double percentage = ci / mean;
//					if(percentage == percentage && percentage > 0.01) {
//						System.out.println(percentage);
//						breaked = true;
//						break;
//					}
//				}
//				if(breaked) {
//					break;
//				}
//				
//			}
//			if(breaked == false) {
//				RealSimuator.FLAG_TERMINATE = true;
//				this.runs = curRun+1;
//			}
//		}
		
//		if(curRun > 100 && curRun%10==0) {
//			for(int i=0; i<extraMeanVars.size(); i++) {
//				for(int j=20; j<statistics_extra.get(extraMeanVarIndexMap.get(extraMeanVars.get(i))).size(); j++) {
//					double mean = statistics_extra.get(extraMeanVarIndexMap.get(extraMeanVars.get(i))).get(j) / (curRun+1);
//					double std = statistics_extra.get(extraSecMmtVarIndexMap.get(extraSecMmtVars.get(i))).get(j) / (curRun+1)-Math.pow(mean, 2);
//					std = Math.sqrt(std);
//					double se = std / Math.sqrt(curRun+1);
//					double ci = 1.96*se;
//					double percentage = ci / mean;
//					if(percentage == percentage && percentage > 0.01) {
//						System.out.println(percentage);
//						breaked = true;
//						break;
//					}
//				}
//				if(breaked) {
//					break;
//				}
//				
//			}
//			if(breaked == false) {
//				RealSimuator.FLAG_TERMINATE = true;
//				this.runs = curRun+1;
//			}
//		}
//		
		
		
		
	}
	
	
	
	public int getAgentIndex(String state){
		if(agentIndexMap.get(state) != null) {
			return agentIndexMap.get(state);
		}else {
			return -1;
		}
	}
	
	public HashMap<String, Integer> getAgentIndexMap() {
		return agentIndexMap;
	}
	
	
	public void record(double t, HashMap<String, Integer> agentMap) {
		if(t>=currentStep && time.size()>0) {
			int forwardSteps = 0;
			while(currentStep <= t) {
				time.add(currentStep);
				currentStep = currentStep + timeStep;
				forwardSteps ++;
			}
			
			for(int i=0; i<lastRecPopu.length; i++) {
				double num = lastRecPopu[i];
				for(int j=0; j<forwardSteps; j++) {
					population.get(i).add(num);
				}
			}
			//just for information print
//			for(String key: agentIndexMap.keySet()) {
//				System.out.print(key + ":" +  agentMap.get(key) + " ");
//			}
//			System.out.println();
		}
		
		
		for(String key: agentIndexMap.keySet()) {
			lastRecPopu[agentIndexMap.get(key)] = agentMap.get(key);//count
		}
		
		if(time.size() == 0) {
			time.add(currentStep);
			for(int i=0; i<lastRecPopu.length; i++) {
				double num = lastRecPopu[i];
				//System.out.println(""+i+" "+lastRecPopu[i]);
				population.get(i).add(num);
			}
		}
	}
	
	
	
	public void prePlot() {
		//System.out.println("time size: "+time.size());
		timeTrajectory = new double[time.size()];
		for(int i=0; i<time.size();i++) {
			timeTrajectory[i] = time.get(i);
		}
		populationTrajectory = new double[statistics_mean.size()][];
		for(int i=0; i<statistics_mean.size();i++) {
			//System.out.println("pop si: "+statistics_mean.get(i).size());
			populationTrajectory[i] = new double[statistics_mean.get(i).size()];
			for(int j=0;j<statistics_mean.get(i).size();j++) {
				populationTrajectory[i][j] = statistics_mean.get(i).get(j);
			}
		}
		
		varianceTrajectory = new double[statistics_variance.size()][];
		for(int i=0; i<statistics_variance.size();i++) {
			//System.out.println("variance size: "+this.runs);
			varianceTrajectory[i] = new double[statistics_variance.get(i).size()];
			for(int j=0;j<statistics_variance.get(i).size();j++) {
//				varianceTrajectory[i][j] = (statistics_variance.get(i).get(j)/this.runs) 
//					- Math.pow(populationTrajectory[i][j], 2);
				varianceTrajectory[i][j] = (statistics_variance.get(i).get(j)/this.runs);//for second moment
			}
		}
		
		extraTrajectory = new double[statistics_extra.size()][];
		for(int i=0; i<statistics_extra.size(); i++) {
			extraTrajectory[i] = new double[statistics_extra.get(i).size()];
			for(int j=0;j<statistics_extra.get(i).size();j++) {
				extraTrajectory[i][j] = (statistics_extra.get(i).get(j)/this.runs);
			}
		}
		
	}
	
//	public ArrayList<String> getPlotVars() {
//		return this.plotVars;
//	}
	
	public double[] getTimeTrajectory() {
		return timeTrajectory;
	}
	
	public double[][] getPopulationTrajectory(){
		return populationTrajectory;
	}
	
	public double[][] getVarianceTrajectory(){
		return varianceTrajectory;
	}
	
	public double[][] getExtraTrajectory() {
		return extraTrajectory;
	}
	
	public HashMap<String, Integer> getExtraMeanVarIndexMap() {
		return extraMeanVarIndexMap;
	}
	
	public HashMap<String, Integer> getExtraSecMmtVarIndexMap() {
		return extraSecMmtVarIndexMap;
	}
	
	public void clear() {
		finalTime = 0;
		runs = 0;
		
		timeTrajectory = null;
		populationTrajectory = null;
		varianceTrajectory = null;
		
		time.clear();
		population.clear();
		
//		plotVars.clear();
		
		statistics_mean.clear();
		statistics_variance.clear();
		
		//theses two variable count for number of agents of in specific states
		agentIndexMap.clear();
		
		lastRecPopu = null;
		timeStep = 0;
		
		currentStep=0;
	}
}

