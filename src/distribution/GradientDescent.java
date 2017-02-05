package distribution;

public interface GradientDescent {
	public double[] run(int xrange, double[] lambda, double[] mu, double delta, int iterNum);
	
	public double derivative(int xrange, double[] lambda, double[] mu, int i);
}
