package ctmc;


import Jama.Matrix;

public class preOperator {
	private double[] pickupRates;
	private double[] returnRates;
	private int stationCap;
	private int initNum;
	
	public preOperator(double[] pickupRates, double[] returnRates, int stationCap, int initNum) {
		this.pickupRates = pickupRates;
		this.returnRates = returnRates;
		this.stationCap = stationCap;
		this.initNum = initNum;
	}
	
	
	public Matrix generatePI() {
		double[] pi = new double[stationCap+1];
		pi[initNum] = 1;
		double[][] PI = {pi};
		Matrix PIM = new Matrix(PI);
		return PIM;
	}
	
	
	public Matrix[] generateQArray(){
		Matrix[] QArray = new Matrix[pickupRates.length];
		for(int i=0; i<pickupRates.length; i++) {
			Matrix Q = generateQ(returnRates[i], pickupRates[i], stationCap+1);
			QArray[i] = Q;
		}
		return QArray;
	}
	
	public Matrix generateQ(double lambda, double mu, int dem) {
		double[][] Q = new double[dem][dem];
		for(int i=0; i<dem; i++) {
			for(int j=0; j<dem; j++) {
				if(i-j==1) {
					Q[i][j] = mu;
				}
				if(i-j==-1) {
					Q[i][j] = lambda;
				}
				if(i==j) {
					if(i==0)
						Q[i][j] = -lambda;
					else if(i==dem-1) 
						Q[i][j] = -mu;
					else 
						Q[i][j] = -mu-lambda;
				}
			}
		}
		Matrix MQ = new Matrix(Q);
		return MQ;
		
	}
	
}
