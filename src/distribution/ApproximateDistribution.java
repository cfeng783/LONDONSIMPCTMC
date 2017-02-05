package distribution;

import org.apache.commons.math3.special.Beta;

public class ApproximateDistribution {
	public static double generateBinomial(double u1, int n) {
		double p = u1/n;
		
		return p;
	}
	
	
	public static double getBinomialProb(double p, int n) {
		return Math.pow(1-p, n);
	}
	
	public static double[] generateBetaBinomial(double u1, double u2, int n) {
		double a = (u1*u2-n*u1*u1)/( n*u1*u1 + n*u1 -n*u2 - u1*u1);
		double b = - ((n-u1)*(n*u1-u2)/(n*u1*u1 + n*u1 - n*u2 -u1*u1));
		return new double[]{a,b};
	}
	
	public static double getBetaBinomialProb(double a, double b, int n) {
		double logBeta1 = Beta.logBeta(a, n+b);
		double logBeta2 = Beta.logBeta(a, b);
		return Math.exp(logBeta1-logBeta2);
	}
}
