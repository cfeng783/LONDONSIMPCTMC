
package framework;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import simulator.Model;
import simulator.RealSimuator;
import trans.RPItem;
import trans.RateItem;
import trans.Trans;
import utality.Utality;

public class ParserLondonBike {
	public final static int interval = 20;
	public final static int slots = 60*1/interval;
	public final static String snapshot = "1421741187919";
	private int startSlot;
	private int simulationLength;
	
	private int num = 30;
	double pickup[][]; //from slot
	double choice[][][]; // from to slot
	double transit[][]; // from to
	double otherReturn[][]; // to slot
	double otherChoice[][]; //from slot
	private int capacity[];
	
	int initBike[];
	int initBikeInTrans[][];
	int finalBike[];
	
	double returnRate[][][]; //from to slot
	
	int numOfTrips[][]; // from to
	int numOfTripsToUnknown[]; //from
	int numOfTripsFromUnknown[]; //to
	
	int numOfInwardTrips[]; //to
	int numOfOutwardTrips[]; //from
	
	double directDpMatrix[][];
	double dpMatrix[][];
	double maxDpMatrix[][];
	public final static double maxDpThreshold = 0.03;
//	final double maxDpThreshold = 1.01;
	int[] keyStations;
	
	int valid[][]; //from to
	static DecimalFormat df = new DecimalFormat("##.000");
	
	private Map<String, Integer> stationMap = new HashMap<String, Integer>();
	private Map<Integer, String> indexMap = new HashMap<Integer, String>();
	
	int alterPoints[];
	
	double min_pickup1 = 999;
	double min_pickup2 = 999;
	double min_return1 = 999;
	
	int phase[][];
	
	double[][] insideReturn;
	
	public ParserLondonBike(int[] keyStations) {
		this.keyStations = keyStations;
		ArrayList<Station> stList = initStationList();
		num = stList.size();
		capacity = new int[num];
		
		
		for(int i=0; i<stList.size(); i++) {
			Station st = stList.get(i);
			capacity[i] = st.capacity;
			stationMap.put(st.name, i);
			indexMap.put(i, st.name);
		}
		
		pickup = new double[num][slots];
		choice = new double[num][num][slots];
		transit = new double[num][num];
		otherReturn = new double[num][slots];
		otherChoice = new double[num][slots];
		valid = new int[num][num];
		
		//for dependence graph
		numOfTrips = new int[num][num];
		numOfTripsToUnknown = new int[num];
		numOfTripsFromUnknown = new int[num];
		numOfInwardTrips = new int[num];
		numOfOutwardTrips = new int[num];
		returnRate = new double[num][num][slots];
		
		initMetrics();
		
		for(int i=0; i<num; i++) {
			for(int j=0; j<num; j++) {
				numOfInwardTrips[i] += numOfTrips[j][i];
				numOfOutwardTrips[i] += numOfTrips[i][j]; 
			}
			numOfInwardTrips[i] += numOfTripsFromUnknown[i];
			numOfOutwardTrips[i] += numOfTripsToUnknown[i];
		}
		DependenceGraph DG = new DependenceGraph(num, numOfInwardTrips, numOfOutwardTrips, numOfTrips);
		dpMatrix = DG.getDependenceMatrix();
		maxDpMatrix = DG.getMaxDpMatrix();
		directDpMatrix = DG.getDirectDpMatrix();
		Set<Integer> importantSet = getImportantStations();
		
		insideReturn = new double[num][slots];
		for(Integer i: importantSet) {
			for(int j=0; j<num; j++) {
				if(!this.isImportant(j)) {
					for(int k=0; k<slots; k++) {
						otherChoice[i][k] += choice[i][j][k];
						otherReturn[i][k] += returnRate[j][i][k];
					}
				}else {// j is a station in skeletal set
					if(i == keyStations[0]) {
						System.out.println("trips between " + indexMap.get(i) + " and " + indexMap.get(j) + ": " + numOfTrips[i][j] + " " + numOfTrips[j][i]);
						
					}
					
					if(directDpMatrix[i][j] < maxDpThreshold) {
						for(int k=0; k<slots; k++) {
							otherReturn[i][k] += returnRate[j][i][k];
						}
					}else {
						for(int k=0; k<slots; k++) {
							insideReturn[i][k] += returnRate[j][i][k];
						}
					}
					
					if(directDpMatrix[j][i] > maxDpThreshold) {
						
						for(int k=0; k<slots; k++) {
//							if(j==keyStations[0]) {
//								System.out.println("choice: " +  choice[i][j][k]);
//							}
							otherChoice[i][k] += choice[i][j][k];
						}
					}
					
					
					
					
				}
				
			}
		}
		
		for(Integer i: importantSet) {
			String flowToUnknown = "";
			String flowFromUnknown = "";
			for(int k=0; k<slots; k++) {
				flowToUnknown += otherChoice[i][k] + " ";
				flowFromUnknown += otherReturn[i][k]/(otherReturn[i][k]+insideReturn[i][k]) + " ";
			}
			if(i == keyStations[0]) {
				System.out.println("from unknown:" + flowFromUnknown);
				System.out.println("to unknown:" + flowToUnknown);
			}
			
//			if(i == keyStations[0])
//			
		}
		
		InitStateParser isp = new InitStateParser();
		isp.parseFile(snapshot);
		
		this.alterPoints = isp.getAlterPoints();
		this.startSlot = isp.getStartSlot();
		this.initBike = isp.getNumOfBikes();
		this.initBikeInTrans = isp.getNumOfBikeInTrans();
		this.simulationLength = isp.getTimeLength();
		this.finalBike = isp.getNumOfBikesFinal();
		this.phase = isp.getPhases();
		System.out.println("model initialised");
		System.out.println("capacity: " + capacity[keyStations[0]]);
		System.out.println("true: " + finalBike[keyStations[0]]);
	}
	
	public void parse(Model model) {
		ArrayList<Trans> transArray = new ArrayList<Trans>();
		
		transArray.addAll(this.makeTrans());

		
		HashMap<String, Integer> agentMap = new HashMap<String, Integer>();
		for(int i=0; i<num; i++) {
			if(isImportant(i)) {
				agentMap.put("Bike" + "(" + i + ")", initBike[i]);
				agentMap.put("Slot" + "(" + i + ")", capacity[i]-initBike[i]);
			}
		}
		
		for(int i=0; i<num; i++) {
			for(int j=0; j<num; j++) {
				if(valid[i][j] == 1){
					for(int k=1; k<=phase[i][j]; k++) {
						agentMap.put("P" + k + "BikeTo" + j + "(" + i + ")", 0);
					}
					
					for(int k=0; k<initBikeInTrans[i][j]; k++) {
						int chosen = Utality.nextInt(phase[i][j])+1;
						int prev = agentMap.get("P" + chosen + "BikeTo" + j + "(" + i + ")");
						agentMap.put("P" + chosen + "BikeTo" + j + "(" + i + ")", prev+1);
					}
				}
					
			}
		}
		
		
		System.out.println("Trans count: " + transArray.size());
		System.out.println("agent count: " + agentMap.keySet().size());
		
		model.init(transArray, agentMap);
		model.setAlterPoints(alterPoints);
		RealSimuator.finaltime = this.simulationLength;
//		for(Trans trans: transArray) {
//			System.out.println(trans.toString());
//		}
	}
	
	public ArrayList<Trans> makeTrans() {
		ArrayList<Trans> ret = new ArrayList<Trans>();
		
		for(int from=0; from<num; from++) {
			for(int to=0; to<num; to++) {
				if(isImportant(from) && isImportant(to) && directDpMatrix[to][from]>=maxDpThreshold) {
					Trans t1 = this.makeTransForPickup(from, to);
					Trans t2 = this.makeTransForReturn(from, to);
					if(t1 != null && t2 != null) {
						ret.add(t1);
						ret.add(t2);
						ret.addAll(this.makeTransInJourney(from, to));
						valid[from][to] = 1;
						if(t1.getFireCount() != t2.getFireCount()) {
							System.err.println("error init");
						}else {
							double firecount = t2.getFireCount();
							if(this.min_pickup1 > firecount) {
								this.min_pickup1 = firecount;
							}
						}
					}
				}
			}
			if(isImportant(from)) {
				Trans otherPick = this.makeTransForPickup(from);
				Trans otherReturn = this.makeTransForReturn(from);
				if(otherPick != null) {
					ret.add(otherPick);
					double firecount = otherPick.getFireCount();
					if(this.min_pickup2 > firecount) {
						this.min_pickup2 = firecount;
					}
				}
				if(otherReturn != null) {
					ret.add(otherReturn);
					double firecount = otherReturn.getFireCount();
					if(this.min_return1 > firecount) {
						this.min_return1 = firecount;
					}
				}
			}
			
		}
		
		return ret;
	}
	
	public Trans makeTransForPickup(int from) {
		ArrayList<RPItem> reactants = new ArrayList<RPItem>();
		ArrayList<RPItem> products = new ArrayList<RPItem>();
		ArrayList<RateItem> rateItems = new ArrayList<RateItem>();
		
		RPItem reactItem = new RPItem("Bike" + "(" + from + ")", 1);
		reactants.add(reactItem);
		
		RPItem productItem2 = new RPItem("Slot" + "(" + from + ")", 1);
		products.add(productItem2);
		
		double pickupRate[] = new double[slots-this.startSlot];
		boolean isValid = false;
		for(int j=this.startSlot; j<slots; j++) {
			int k = j-this.startSlot;
			pickupRate[k] = pickup[from][j]*otherChoice[from][j];
			if(pickupRate[k] > 0) {
				isValid = true;
			}
		}
		if(isValid) {
			Trans trans = new Trans(reactants, products, pickupRate,rateItems);
			trans.setFireCount(numOfTripsToUnknown[from]);
			return trans;
		}else {
			return null;
		}
	}
	
	public Trans makeTransForPickup(int from, int to) {
		ArrayList<RPItem> reactants = new ArrayList<RPItem>();
		ArrayList<RPItem> products = new ArrayList<RPItem>();
		ArrayList<RateItem> rateItems = new ArrayList<RateItem>();
		
		RPItem reactItem = new RPItem("Bike" + "(" + from + ")", 1);
		reactants.add(reactItem);
		
		RPItem productItem2 = new RPItem("Slot" + "(" + from + ")", 1);
		products.add(productItem2);
		
		RPItem productItem = new RPItem("P" + 1 + "BikeTo" + to + "(" + from + ")", 1);
		products.add(productItem);
		
		double pickupRate[] = new double[slots-this.startSlot];
		boolean isValid = false;
		for(int j=this.startSlot; j<slots; j++) {
			int k = j-this.startSlot;
			pickupRate[k] = Double.parseDouble(df.format(pickup[from][j]*choice[from][to][j]));
			if(pickupRate[k] > 0) {
				isValid = true;
			}
		}
		if(isValid) {
			Trans trans = new Trans(reactants, products, pickupRate,rateItems);
			trans.setFireCount(numOfTrips[from][to]);
			return trans;
		}else {
			return null;
		}
	}
	
	public Trans makeTransForReturn(int from, int to) {
		
		ArrayList<RPItem> reactants = new ArrayList<RPItem>();
		ArrayList<RPItem> products = new ArrayList<RPItem>();
		ArrayList<RateItem> rateItems = new ArrayList<RateItem>();
		
		RPItem reactItem = new RPItem("P" + phase[from][to] + "BikeTo" + to + "(" + from + ")", 1);
		reactants.add(reactItem);
		
		RPItem reactItem2 = new RPItem("Slot" + "(" + to + ")", 1);
		reactants.add(reactItem2);
		
		RateItem rateItem = new RateItem("P" + phase[from][to] + "BikeTo" + to + "(" + from + ")", 1);
		rateItems.add(rateItem);
		
		RPItem productItem = new RPItem("Bike" + "(" + to + ")", 1);
		products.add(productItem);
		
		if(transit[from][to] > 0) {
			Trans trans = new Trans(reactants, products, transit[from][to]*phase[from][to],rateItems);
			trans.setFireCount(numOfTrips[from][to]);
			return trans;
		}else {
			return null;
		}
	}
	
	public Trans makeTransForReturn(int to) {
		
		ArrayList<RPItem> reactants = new ArrayList<RPItem>();
		ArrayList<RPItem> products = new ArrayList<RPItem>();
		ArrayList<RateItem> rateItems = new ArrayList<RateItem>();
		
		
		RPItem reactItem2 = new RPItem("Slot" + "(" + to + ")", 1);
		reactants.add(reactItem2);
		
		
		RPItem productItem = new RPItem("Bike" + "(" + to + ")", 1);
		products.add(productItem);
		
		double[] rates = new double[slots-this.startSlot];
		
		boolean isValid = false;
		for(int j=this.startSlot; j<slots; j++) {
			int k = j-this.startSlot;
			if(otherReturn[to][j] > 0) {
				isValid = true;
			}
			rates[k] = otherReturn[to][j];
		}
		if(isValid) {
			Trans trans = new Trans(reactants, products, rates, rateItems);
			trans.setFireCount(numOfTripsFromUnknown[to]);
			return trans;
		}else {
			return null;
		}
	}
	
	public ArrayList<Trans> makeTransInJourney(int from, int to) {
		ArrayList<Trans> ret = new ArrayList<Trans>();
		for(int i=1; i<phase[from][to]; i++) {
			ArrayList<RPItem> reactants = new ArrayList<RPItem>();
			ArrayList<RPItem> products = new ArrayList<RPItem>();
			ArrayList<RateItem> rateItems = new ArrayList<RateItem>();
			
			RPItem reactItem = new RPItem("P" + i + "BikeTo" + to + "(" + from + ")", 1);
			reactants.add(reactItem);
			
			RateItem rateItem = new RateItem("P" + i + "BikeTo" + to + "(" + from + ")", 1);
			rateItems.add(rateItem);
			
			int j = i+1;
			RPItem productItem = new RPItem("P" + j + "BikeTo" + to + "(" + from + ")", 1);
			products.add(productItem);
			
//			System.out.println(from+"#"+to + " :" + transit[from][to]);
			
			Trans trans = new Trans(reactants, products, transit[from][to]*phase[from][to],rateItems);
			ret.add(trans);
		}
		return ret;
	}
	
	private boolean isImportant(int station) {
		for(int i=0; i<keyStations.length;i++) {
			if(( maxDpMatrix[keyStations[i]][station] > maxDpThreshold) || keyStations[i] == station) {
				return true;
			}
		}
		return false;
	}
	
	private Set<Integer> getImportantStations() {
		Set<Integer> importantSet = new HashSet<Integer>();
		String str = "";
		int total = 0;
		for(int j=0;j<num;j++) {
			for(int i=0; i<keyStations.length;i++) {
				if(( maxDpMatrix[keyStations[i]][j] > maxDpThreshold)
						|| keyStations[i] == j) {
					str += j + " ";
					total++;
					importantSet.add(j);
				}
			}
		}
		System.out.println(str);
		System.out.println("num of key stations: " + total);
		return importantSet;
	}
	
	private void initMetrics() {
		for(String stName: stationMap.keySet()) {
			BufferedReader br = null;
			try {
				br = new BufferedReader(new FileReader(System.getProperty("user.home")+"/Desktop/cycle data/processed2014/" 
						+ stName + ".txt"));
				String line = br.readLine();
				String[] strPickups = line.split(":");
				for(int i=0; i<slots; i++) {
					pickup[stationMap.get(stName)][i] = Double.parseDouble(strPickups[i]);
				}
				
				while ((line = br.readLine()) != null) {
					line = line.trim();
					if(line.startsWith("other")) {
				    	String[] tempt = line.split("#");
				    	
				    	String[] strOtherChoices = tempt[1].split(":");
				    	for(int i=0; i<slots; i++) {
				    		otherChoice[stationMap.get(stName)][i] = Double.parseDouble(strOtherChoices[i]);
				    	}
				    	
				    	String[] strReturns = tempt[2].split(":");
				    	for(int i=0; i<slots; i++) {
							otherReturn[stationMap.get(stName)][i] = Double.parseDouble(strReturns[i]);
						}
				    	
				    	int numberOfOutwardTrips = Integer.parseInt(tempt[3]);
				    	int numberOfInwardTrips = Integer.parseInt(tempt[4]);
				    	numOfTripsToUnknown[stationMap.get(stName)] = numberOfOutwardTrips;
				    	numOfTripsFromUnknown[stationMap.get(stName)] = numberOfInwardTrips;
				    }else {
				    	String[] splits = line.split("#");
//				    	for(int i=0; i<splits.length; i++) {
//				    		System.out.println(splits[i]);
//				    	}
				    	String endSt = splits[0];
				    	transit[stationMap.get(stName)][stationMap.get(endSt)] = Double.parseDouble(splits[1]);
				    	String[] probs = splits[2].split(":");
				    	for(int i=0; i<slots; i++) {
				    		choice[stationMap.get(stName)][stationMap.get(endSt)][i] = Double.parseDouble(probs[i]);
				    	}
				    	int numberOfTrips = Integer.parseInt(splits[3]);
				    	numOfTrips[stationMap.get(stName)][stationMap.get(endSt)] = numberOfTrips;
				    	
				    	String[] retRates = splits[4].split(":");
				    	for(int i=0; i<slots; i++) {
				    		returnRate[stationMap.get(endSt)][stationMap.get(stName)][i] = Double.parseDouble(retRates[i]);
				    	}
				    	
				    }
				}
				
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} catch (Exception e) {
				e.printStackTrace();
			}finally {
				if (br != null) {
					try {
						br.close();
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
			}
		}
	}
	
	private ArrayList<Station> initStationList() {
		BufferedReader br = null;
		String line = "";
		ArrayList<Station> stList = new ArrayList<Station>();
		try {
			br = new BufferedReader(new FileReader(System.getProperty("user.home")+"/Desktop/cycle data/" + "capacity.txt"));
			while ((line = br.readLine()) != null) {
			    line = line.trim();
			    String[] splits = line.split(":");
				Station st = new Station(splits[0], Integer.parseInt(splits[1]),Integer.parseInt(splits[2]));
				stList.add(st);
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		}finally {
			if (br != null) {
				try {
					br.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		return stList;
	}
	
	public double[][] getDpMatrix() {
		return this.dpMatrix;
	}
	
	private class Station {
		String name;
		int capacity;
		int nbBikes;
		
		public Station(String name, int capacity, int nbBikes) {
			this.name = name;
			this.capacity = capacity;
			this.nbBikes = nbBikes;
		}
	}
	
	
	//the following methods are only used to debug
	public void printMsg() {
		System.out.println("capacity: " + capacity[keyStations[0]]);
		System.out.println("true: " + finalBike[keyStations[0]]);
	}
	
	public int getTrueNum() {
		return finalBike[keyStations[0]];
	}
	
	public int getInitNum() {
		return initBike[keyStations[0]];
	}
	
	public int getCapacity() {
		return capacity[keyStations[0]];
	}
	
	public double[] getPickupRates() {
		double pickupRate[] = new double[slots-this.startSlot];
		for(int j=this.startSlot; j<slots; j++) {
			int k = j-this.startSlot;
			pickupRate[k] = Double.parseDouble(df.format(pickup[keyStations[0]][j]));
		}
		return pickupRate;
	}
	
	public double[] getReturnRates() {
		double[] rates = new double[slots-this.startSlot];
		
		for(int j=this.startSlot; j<slots; j++) {
			int k = j-this.startSlot;
			rates[k] = otherReturn[keyStations[0]][j]+insideReturn[keyStations[0]][j];
		}
		return rates;
	}
}

