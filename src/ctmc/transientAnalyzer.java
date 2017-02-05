package ctmc;

import Jama.Matrix;

public class transientAnalyzer {
	
	
	public static double[] getFinalState(double[] pickupRates, double[] returnRates, int[] alterPoints, int initNum, int capacity){
		preOperator op = new preOperator(pickupRates, returnRates, capacity, initNum);
		Matrix PI = op.generatePI();
		Matrix[] QArray = op.generateQArray();
		
		for(int i=0; i<QArray.length; i++) {
			double t;
			if(i== 0) {
				t = alterPoints[0];
			}else {
				t = alterPoints[i]-alterPoints[i-1];
			}
			
			PI = compute(PI, QArray[i], t);
		}
		double finalstates[] = new double[capacity+1];
		for(int i=0; i<finalstates.length; i++) {
			finalstates[i] = PI.get(0, i);
		}
		return finalstates;
	}
	

	
	private static Matrix compute(Matrix PI0, Matrix Q, double t) {
		
		int dem = Q.getColumnDimension();
		double[][] arrayI = new double[dem][dem];
		for(int i=0; i<dem; i++) {
			for(int j=0; j<dem; j++) {
				if(i==j) {
					arrayI[i][j] = 1;
				}
			}
		}
		
		double lambda = 0;
		for(int i=0; i<dem; i++) {
			for(int j=0; j<dem; j++) {
				if(i==j) {
					if(Math.abs(Q.get(i, j)) >= lambda) {
						lambda = Math.abs(Q.get(i, j));
					}
				}
			}
		}
		//lambda = lambda + 1;
		
		Matrix I = new Matrix(arrayI);
		//System.out.println("I:");
		//I.print(3, 3);
		
		//System.out.println("lambda:" + lambda);
		
		Matrix P = I.plus(Q.times(1/lambda));
		
		//System.out.println("P:");
		//P.print(ConstValues.stationCap+1, ConstValues.stationCap+1);
		
		//System.out.println(fac(3));
		
		Matrix ret = PI0.times(I).times(Math.pow(Math.E, -lambda*t));
		//System.out.println("stepMatrix:");
		//ret.print(ConstValues.stationCap+1, ConstValues.stationCap+1);
		for(int n=1; n<21; n++) {
			Matrix np = P.copy();
			for(int i=1; i<n; i++) {
				np = np.times(P);
			}
			double muti = Math.pow(Math.E, -lambda*t) * Math.pow(lambda*t, n) / fac(n);
			Matrix stepMatrix = PI0.times(np).times(muti);
			//System.out.println("stepMatrix:");
			//stepMatrix.print(ConstValues.stationCap+1, ConstValues.stationCap+1);
			ret = ret.plus(stepMatrix);
		}
		return ret;
		//System.out.println("ret:");
		//ret.print(3, 3);
		//Matrix I = 
		
	}
	
	public static long fac(int n) {
		if(n == 0) {
			return 0;
		}
		long ret = 1;
		for(int i=1; i<=n; i++) {
			ret = ret * i;
		}
		//System.out.println("ret:" +ret);
		return ret;
	}
	
	public static void compute(Matrix P, Matrix Q) {
		
		Matrix ret = P.times(Q);
		ret.print(2, 5);
	}
}
