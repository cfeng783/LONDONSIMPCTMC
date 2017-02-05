package distribution;

import liblbfgs.Function;

public class DescentFunction implements Function, GenerateDistribution {
	int xrange; 
	double[] mu;
	
	public DescentFunction(int xrange, double[] mu) {
		this.xrange = xrange;
		this.mu = mu;
	}

	@Override
	public int getDimension() {
		return mu.length;
	}

	@Override
	public double valueAt(double[] lambda) {
		double value = Math.log(sumExp(xrange, lambda));
		for(int i=0; i<lambda.length; i++) {
			value += lambda[i] * mu[i];
		}
		return value;
	}

	@Override
	public double[] gradientAt(double[] lambda) {
		double[] gradidents = new double[lambda.length];
		for(int i=1; i<=lambda.length; i++) {
			gradidents[i-1] = derivative(xrange, lambda, mu, i);
		}
		return gradidents;
	}
	
	
	private double derivative(int xrange, double[] lambda, double[] mu, int i) {
		double ret = sumExp(xrange, lambda, i) / sumExp(xrange, lambda) + mu[i-1];
//		System.out.println("derivative: " + ret);
		return ret;
	}
	
	
	private double exp(int x, double[] lambda) {
		double ret = 0;
		
		for(int k=1; k<=lambda.length; k++) {
			ret -= lambda[k-1]*Math.pow(x, k);
		}
		
		ret = Math.exp(ret);
		return ret;
	}
	
	private double sumExp(int xrange, double[] lambda) {
		double ret = 0;
		for(int x=0; x<=xrange; x++) {
			ret += exp(x, lambda);
		}
		return ret;
	}
	
	private double sumExp(int xrange, double[] lambda, int i) {
		double ret = 0;
		for(int x=0; x<=xrange; x++) {
			ret += exp(x, lambda)*(-Math.pow(x, i));
		}
		return ret;
	}
	
	private double prob(int x, int xrange, double[] lambda) {
		double ret = exp(x,lambda) / sumExp(xrange, lambda);
		return ret;
	}

	@Override
	public double[] generate(int xrange, double[] lambda) {
		double ret[] = new double[xrange+1];
		for(int i=0; i<=xrange; i++) {
			ret[i] = prob(i,xrange, lambda);
		}
		return ret;
	}

}
