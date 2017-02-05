package moment;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;

import odesolver.TestSolver;

import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;

import couplingcoefficient.GraphBuilder;
import simulator.RealSimuator;
import trans.Trans;


public class MomentScripter {
	
	ArrayList<String> agentList;

	HashMap<String, Integer> indexMap = new HashMap<String, Integer>(); //indexMap for first moment
	
	HashMap<String, Integer> secondIndexMap = new HashMap<String, Integer>(); //indexMap for second moment
	
	HashMap<String, Integer> covIndexMap = new HashMap<String, Integer>(); //indexMap for covariance
	
	HashMap<Integer, String> covStateMap = new HashMap<Integer,String>();
	
	ArrayList<Trans> transArray;
	
	int finalOdeIndex;
	
	
	public static int couplingThreshold = 1;
	
	String colorArray[] = {	"'r-'", "'g-'", "'b-'","'y-'","'m-'","'c-'","'k-'"};
	
	public MomentScripter(HashMap<String, Integer> initAgentMap, ArrayList<Trans> transArray) {
		this.transArray = transArray;
		agentList = new ArrayList<String>();
		for(String agent: initAgentMap.keySet()) {
			agentList.add(agent);
		}
		for(int i=0;i<agentList.size();i++) {
			indexMap.put(agentList.get(i), i+1);
			secondIndexMap.put(agentList.get(i), i+1+agentList.size());
		}
		
		int current = 2*agentList.size() + 1;
		System.out.println("current: " + current);
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
		finalOdeIndex = current;
		System.out.println("finalOdeIndex: " + finalOdeIndex);
	}
	
	
	
	public ArrayList<String> genOdes() {
		//initPTransitions();
		ArrayList<String> var = new ArrayList<String>();
		String strFunc = "function dy = " + " fluidflow" + "(t,y)";
		var.add(strFunc);
		int size = this.finalOdeIndex - 1;
		String strDy = "dy=zeros(" + size +",1);";
		var.add(strDy);
		
		for(int n=1; n<=agentList.size(); n++) {	
			String ode = "dy(" + n +  ")=0";
			//String ode = stateList.get(n-1).getID()+ " dy(" + n +  ")=0";
			String agent = agentList.get(n-1);
			for(Trans trans: transArray) {
				ode += trans.makeFirstMoment(agent, indexMap, secondIndexMap, covIndexMap);
			}
			ode += ";";
			//ode += st.getID();
			var.add(ode);
		}
		for(int i=0; i<agentList.size(); i++) {	
			String agent = agentList.get(i);
			String ode = "dy(" + secondIndexMap.get(agent) +  ")=0";
			for(Trans trans: transArray) {
				ode += trans.makeSecondMoment(agent, indexMap, secondIndexMap, covIndexMap);
			}
			ode += ";";
			//ode += st.getID();
			var.add(ode);
		}
		
		for(int i=2*agentList.size()+1; i<finalOdeIndex; i++) {
			String ode = "dy(" + i +  ")=0";
			String st[] = resolveCovKey(covStateMap.get(i));
			for(Trans trans: transArray) {
				ode += trans.makeCovMoment(st[0], st[1], indexMap, secondIndexMap, covIndexMap);
			}
			ode += ";";
			//ode += covStateMap.get(i);
			var.add(ode);
		}
		
		return var;
	}
	
	
	public ArrayList<String> genMatlabScript(int timelength, ArrayList<String> plotVars, ArrayList<String> secMmtVars, ArrayList<String> extraVars) {
		ArrayList<String> var = new ArrayList<String>();
		int num[] = new int[agentList.size()+1];
		for(String agent: RealSimuator.getModel().getInitAgentMap().keySet()) {
			int count = RealSimuator.getModel().getInitAgentMap().get(agent);
			int ind = indexMap.get(agent);
			num[ind] += count;
		}
		
		String func = "[T,Y]=ode45(@fluidflow,[0 "+ timelength +"],[";
		for(int i=1; i<num.length; i++) {
			func += " " + num[i];
		}
		for(int i=1; i<num.length; i++) {
			int trueNum = num[i] * num[i];
			func += " " + trueNum;
		}
		
		for(int i=2*agentList.size()+1; i<finalOdeIndex; i++) {
			String st[] = resolveCovKey(covStateMap.get(i));
			int pos1 = indexMap.get(st[0]);
			int pos2 = indexMap.get(st[1]);
			int trueNum = num[pos1] * num[pos2];
			func += " " + trueNum;
		}
		
		func += "]);";
		var.add(func);
		
		if(plotVars.size()>colorArray.length) {
			System.err.println("too many vars to plot!");
			//return null;
		}
		
		for(int i=0; i<plotVars.size();i++) {
			String plotVar = plotVars.get(i);
			Integer index = indexMap.get(plotVar);
			
			String str = "plot(T,Y(:," + index.toString() +"),"+colorArray[i%colorArray.length]+",'LineWidth',1" + ");";
			var.add(str);
			
			String strHold = "hold on;";
			var.add(strHold);
		}
		
//		for(int i=0; i<secMmtVars.size();i++) {
//			String plotVar = secMmtVars.get(i);
//			Integer index = secondIndexMap.get(plotVar);
//			
//			String str2 = "plot(T,Y(:," + index +"),"+colorArray[(i+plotVars.size())%colorArray.length]+",'LineWidth',1" + ");";
//			var.add(str2);
//			
//			String strHold = "hold on;";
//			var.add(strHold);
//		}
		
		for(int i=0; i<extraVars.size(); i++) {
			String str = "plot(T,";
			for(String agentKey: indexMap.keySet()) {
				if(agentKey.substring(0, agentKey.indexOf("(")).equals(extraVars.get(i))) {
					if(str.length()==7) {
						str += "Y(:," + indexMap.get(agentKey) + ")";
					}else {
						str += "+Y(:," + indexMap.get(agentKey) + ")";
					}
				}
			}
//			str+=","+colorArray[i%colorArray.length]+ ",'LineWidth',1" + ");";
			var.add(str);
		}
		
		for(int i=0; i<extraVars.size(); i++) {
			String str = "plot(T,";
			ArrayList<String> matchedSts = new ArrayList<String>();
			for(String agentKey: indexMap.keySet()) {
				if(agentKey.substring(0, agentKey.indexOf("(")).equals(extraVars.get(i))) {
					matchedSts.add(agentKey);
				}
			}
			
			for(int m=0; m<matchedSts.size(); m++) {
				for(int n=0; n<matchedSts.size(); n++) {
					if(m == n) {
						if(str.length()==7) {
							str += "Y(:," + secondIndexMap.get(matchedSts.get(m)) + ")";
						}else {
							str += "+Y(:," + secondIndexMap.get(matchedSts.get(m)) + ")";
						}
					}
					if(m<n) {
						String st1 = matchedSts.get(m);
						String st2 = matchedSts.get(n);
						
						if(covIndexMap.get(makeCovKey(st1, st2)) != null) {
							str += "+2*Y(:," + covIndexMap.get(makeCovKey(st1, st2)) + ")";
						} else if(covIndexMap.get(makeCovKey(st2, st1)) != null) {
							str += "+2*Y(:," + covIndexMap.get(makeCovKey(st2, st1)) + ")";
						}else {
							str += "+2*Y(:," + indexMap.get(st1) + ")" + ".*Y(:," + indexMap.get(st2) + ")";
						}
					}
				}
			}
				
//			str+=","+colorArray[i%colorArray.length]+ ",'LineWidth',1" + ");";
			var.add(str);
			
			String strHold = "hold on;";
			var.add(strHold);
		}
		
		String xL = "xlabel('time');";
		String yL = "ylabel('population');";
		var.add(xL);
		var.add(yL);
		
		String strLegend = "legend(";
		for(int i=0; i<plotVars.size();i++) {
			String plotVar = plotVars.get(i);
			if(i==plotVars.size()-1) {
				//strLegend += "'" + plotVar + "',";
				strLegend += "'" + plotVar + "');";
			}else{
				//strLegend += "'" + plotVar + "',";
				strLegend += "'" + plotVar + "',";
			}
		}
		var.add(strLegend);
		return var;
	}
	
	public ArrayList<String> genRateFunction(int[] alterPoints) {
		//initPTransitions();
		ArrayList<String> var = new ArrayList<String>();
		String strFunc = "function r = " + " R" + "(t,n)";
		var.add(strFunc);
		
		int index = 0;
		for(Trans trans: this.transArray) {
			if(trans.getConstRateFactor().length > 1) {
				if(index == 0) {
					String tempt = "if n==" + trans.getIndex();
					var.add(tempt);
					index++;
				}else {
					String tempt = "elseif n==" + trans.getIndex();
					var.add(tempt);
				}
				
				
				for(int i=0; i<alterPoints.length; i++) {
					String str = "";
					if(i==0) {
						str = "if t<" + alterPoints[i];
						var.add(str);
						
						str = "r=" + trans.getConstRateFactor()[0] + ";";
						var.add(str);
					}else {
						str = "elseif t>=" + alterPoints[i-1] + " && " + "t<" + alterPoints[i]; 
						var.add(str);
						
						str = "r=" + trans.getConstRateFactor()[i] + ";";
						var.add(str);
					}
				}
				var.add("end");
			}
		}
		var.add("end");
		
		return var;
	}
	
	public void genRateFunction(int[] alterPoints, String foldername) throws IOException {
		//initPTransitions();
		
		for(Trans trans: this.transArray) {
			ArrayList<String> vars = trans.getRateFunction(alterPoints);
			if(vars != null) {
				String filename = foldername + "/R" + trans.getIndex() + ".m";
				PrintWriter sw = new PrintWriter(new OutputStreamWriter(new FileOutputStream(filename)),true);
				for(String var: vars) {
					sw.println(var);
				}
				sw.close();
			}
		}
		
		
	}
	
	public static String makeCovKey(String st1, String st2) {
		return st1 + "*" + st2;
	}
	
	public static String[] resolveCovKey(String covKey) {
		String st[] = new String[2];
		int index = covKey.indexOf("*");
		st[0] = covKey.substring(0, index);
		st[1] = covKey.substring(index+1);
		return st;
	}
	
	//looke for a cov item in covIndexMap, return true for existence
	public boolean checkCovItem(String st1, String st2) {
		if(covIndexMap.get(makeCovKey(st1, st2)) != null || covIndexMap.get(makeCovKey(st2, st1)) != null) {
			return true;
		}else {
			return false;
		}
	}
	
	public static String getCovIndex(String st1, String st2, HashMap<String, Integer> covMap, HashMap<String, Integer> indexMap) {
		if(covMap.get(makeCovKey(st1, st2)) != null) {
			return "y(" + covMap.get(makeCovKey(st1, st2)) + ")";
		}
		if(covMap.get(makeCovKey(st2, st1)) != null) {
			return "y(" + covMap.get(makeCovKey(st2, st1)) + ")";
		}
		return "y(" + indexMap.get(st1) + ")*y(" + indexMap.get(st2) + ")";
	}
	
	public static double getCovItem(double[] y, String st1, String st2, HashMap<String, Integer> covMap, HashMap<String, Integer> indexMap) {
		if(covMap.get(makeCovKey(st1, st2)) != null) {
			return y[covMap.get(makeCovKey(st1, st2))];
		}
		if(covMap.get(makeCovKey(st2, st1)) != null) {
			return y[covMap.get(makeCovKey(st2, st1))];
		}
		return y[indexMap.get(st1)] * y[indexMap.get(st2)];
	}
	
	public FirstOrderDifferentialEquations getDiffEquations(int[] alterPoints) {
		FirstOrderDifferentialEquations ode = new TestSolver(agentList, transArray, finalOdeIndex, indexMap, 
				secondIndexMap, covIndexMap, covStateMap,alterPoints);
		return ode;
	}
	
	public double[] getInitValues() {
		double[] initValues = new double[finalOdeIndex-1];
		
		for(String agent: RealSimuator.getModel().getInitAgentMap().keySet()) {
			int count = RealSimuator.getModel().getInitAgentMap().get(agent);
			int ind = indexMap.get(agent)-1;
			initValues[ind] = count;
			
			int secIndex = secondIndexMap.get(agent)-1;
			initValues[secIndex] = count*count;
		}
		
		for(int i=2*agentList.size()+1; i<finalOdeIndex; i++) {
			String st[] = resolveCovKey(covStateMap.get(i));
			int pos1 = indexMap.get(st[0]);
			int pos2 = indexMap.get(st[1]);
			double trueNum = initValues[pos1-1] * initValues[pos2-1];
			initValues[i-1] = trueNum;
		}
		return initValues;
	}
	
	public int getIndex(String agent) {
		return indexMap.get(agent);
	}
	
	public int getSecondIndex(String agent) {
		return secondIndexMap.get(agent);
	}
}