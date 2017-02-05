package plot;

import java.util.ArrayList;

public class PlotVarDefiner {
	private ArrayList<String> plotVars = new ArrayList<String>();
	private ArrayList<String> secMmtVars = new ArrayList<String>();
	private ArrayList<String> extraVars = new ArrayList<String>();
	
	public void insertPlotVars(ArrayList<String> vars) {
		for(String var: vars) {
			plotVars.add(var);
		}
	}
	
	public void insertPlotVars(String[] vars) {
		for(int i=0; i<vars.length; i++) {
			plotVars.add(vars[i]);
		}
	}
	
	public void insertSecMmtVars(String[] vars) {
		for(String var: vars) {
			secMmtVars.add(var);
		}
	}
	
	public void insertSecMmtVars(ArrayList<String> vars) {
		for(String var: vars) {
			secMmtVars.add(var);
		}
	}
	
	public void insertExtraVars(ArrayList<String> vars) {
		for(String var: vars) {
			extraVars.add(var);
		}
	}
	
	public void insertExtraVars(String[] vars) {
		for(String var: vars) {
			extraVars.add(var);
		}
	}
	
	public ArrayList<String> getPlotVars(){
		return this.plotVars;
	}
	
	public ArrayList<String> getSecMmtVars() {
		return this.secMmtVars;
	}
	
	public ArrayList<String> getExtraVars() {
		return extraVars;
	}
	
	public String[] getPlots(){
		String plots[] = new String[plotVars.size()];
		for(int i=0; i<plotVars.size(); i++) {
			plots[i] = plotVars.get(i);
		}
		return plots;
	}
	
	public void clear() {
		this.plotVars.clear();
		this.secMmtVars.clear();
	}
}
