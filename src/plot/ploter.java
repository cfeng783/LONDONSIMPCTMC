package plot;

import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.util.ArrayList;

import javax.swing.JFrame;
import simulator.RealSimuator;

public class ploter {

	public void plot(String[] agents) {
		if(agents == null || agents.length==0) {
			return;
		}
		RealSimuator.getCounter().prePlot();
		double[] timeTraj = RealSimuator.getCounter().getTimeTrajectory();
		double[][] populationTraj = RealSimuator.getCounter().getPopulationTrajectory();
		
		ArrayList<Plot2DTrajectory> trajectories = new ArrayList<Plot2DTrajectory> ();
		
		for(String agent: agents) {
			if(RealSimuator.getCounter().getAgentIndex(agent) != -1) {
				Plot2DTrajectory traj = new Plot2DTrajectory(agent, 
						timeTraj, populationTraj[RealSimuator.getCounter().getAgentIndex(agent)]);
				trajectories.add(traj);
			}else {
				int index = RealSimuator.getCounter().getExtraMeanVarIndexMap().get(agent);
				Plot2DTrajectory traj = new Plot2DTrajectory(agent, 
						timeTraj, RealSimuator.getCounter().getExtraTrajectory()[index]);
				trajectories.add(traj);
			}
		}
		
		//System.out.println(timeTraj.length);
		LineChart chart = new LineChart(trajectories, "Time", "Population");
		JFrame frame = chart.show();
		
	}
	
	public void plotVariance(String[] agents) {
		if(agents == null || agents.length==0) {
			return;
		}
		RealSimuator.getCounter().prePlot();
		double[] timeTraj = RealSimuator.getCounter().getTimeTrajectory();
		double[][] varTraj = RealSimuator.getCounter().getVarianceTrajectory();
		
		ArrayList<Plot2DTrajectory> trajectories = new ArrayList<Plot2DTrajectory> ();
		
		for(String agent: agents) {
			if(RealSimuator.getCounter().getAgentIndex(agent) != -1) {
				Plot2DTrajectory traj = new Plot2DTrajectory(agent, 
						timeTraj, varTraj[RealSimuator.getCounter().getAgentIndex(agent)]);
				trajectories.add(traj);
			}else {
				int index = RealSimuator.getCounter().getExtraSecMmtVarIndexMap().get(agent);
				Plot2DTrajectory traj = new Plot2DTrajectory(agent, 
						timeTraj, RealSimuator.getCounter().getExtraTrajectory()[index]);
				trajectories.add(traj);
			}
		}
		LineChart chart = new LineChart(trajectories, "Time", "Variance");
		chart.show();
		
	}
	
	
	/**
	 * export trajectory to files
	 * */
	public void export(String[] agents, String fileFolder) {
		try{
			RealSimuator.getCounter().prePlot();
			PrintWriter sw = new PrintWriter(new OutputStreamWriter(new FileOutputStream(fileFolder + "/timeTraj")),true);  
			double[] timeTraj = RealSimuator.getCounter().getTimeTrajectory();
			for(int i=0; i<timeTraj.length; i++) {
				//System.out.println(str);
				sw.println(timeTraj[i]);
			}
			sw.close();
			
			double[][] populationTraj = RealSimuator.getCounter().getPopulationTrajectory();
			
			
			for(String agent: agents) {
				sw = new PrintWriter(new OutputStreamWriter(new FileOutputStream(fileFolder+"/"+agent+"_mean")),true); 
				int i = RealSimuator.getCounter().getAgentIndex(agent);
				if(i != -1) {
					for(int j=0; j<populationTraj[i].length; j++) {
						sw.println(populationTraj[i][j]);
					}
				}else {
					int index = RealSimuator.getCounter().getExtraMeanVarIndexMap().get(agent);
					for(int j=0; j<RealSimuator.getCounter().getExtraTrajectory()[index].length; j++) {
						sw.println(RealSimuator.getCounter().getExtraTrajectory()[index][j]);
					}
				}
				
				sw.close();
			}
			
		}catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	public void exportVariance(String[] agents, String fileFolder) {
		try{
			RealSimuator.getCounter().prePlot();
			PrintWriter sw = new PrintWriter(new OutputStreamWriter(new FileOutputStream(fileFolder + "/timeTraj")),true);  
			double[] timeTraj = RealSimuator.getCounter().getTimeTrajectory();
			for(int i=0; i<timeTraj.length; i++) {
				//System.out.println(str);
				sw.println(timeTraj[i]);
			}
			sw.close();
			
			double[][] varianceTraj = RealSimuator.getCounter().getVarianceTrajectory();
			
			for(String agent: agents) {
				sw = new PrintWriter(new OutputStreamWriter(new FileOutputStream(fileFolder+"/"+agent+"_variance")),true); 
				int i = RealSimuator.getCounter().getAgentIndex(agent);
				if(i != -1) {
					for(int j=0; j<varianceTraj[i].length; j++) {
						sw.println(varianceTraj[i][j]);
					}
				}else {
					int index = RealSimuator.getCounter().getExtraSecMmtVarIndexMap().get(agent);
					for(int j=0; j<RealSimuator.getCounter().getExtraTrajectory()[index].length; j++) {
						sw.println(RealSimuator.getCounter().getExtraTrajectory()[index][j]);
					}
				}
				sw.close();
			}
			
		}catch(Exception e) {
			e.printStackTrace();
		}
	}
	
}
