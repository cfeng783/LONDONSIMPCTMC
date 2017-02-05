package distribution;

public class DistributionEstimator implements GradientDescent, GenerateDistribution{

	@Override
	public double[] run(int xrange, double[] lambda, double[] mu, double delta,
			int iterNum) {
		double history[] = new double[iterNum];
		int current = 0;
		double tempt[] = new double[lambda.length];
		while(current < iterNum) {
			for(int i=1; i<=lambda.length; i++) {
				tempt[i-1] = lambda[i-1] - delta*derivative(xrange, lambda, mu, i);
			}
			
			if(current % 1000 == 0) {
				double norm = this.computeNorm(lambda, tempt);
//				System.out.println("norm:" + norm);
				if(norm < 1E-6) {
					double ret[] = new double[current];
					for(int i=0; i<ret.length; i++) {
						ret[i] = history[i];
					}
					return ret;
				}
			}
			
			
			for(int i=0; i<lambda.length; i++) {
				lambda[i] = tempt[i];
			}
			history[current] = Math.log(sumExp(xrange, lambda));
			for(int i=0; i<lambda.length; i++) {
				history[current] += lambda[i] * mu[i];
			}
			
			if((current > 1 && history[current]-history[current-1] > 0) || history[current]!=history[current]) {return null;}
			
			current++;
		}
		
		return history;
	}

	@Override
	public double derivative(int xrange, double[] lambda, double[] mu, int i) {
		double ret = sumExp(xrange, lambda, i) / sumExp(xrange, lambda) + mu[i-1];
//		System.out.println("derivative: " + ret);
		return ret;
	}
	
	private double computeNorm(double[] data1, double[] data2) {
		double norm = 0;
		for(int i=0; i<data1.length; i++) {
			norm += Math.pow(data1[i]-data2[i],2);
		}
		norm = Math.sqrt(norm);
		return norm;
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
