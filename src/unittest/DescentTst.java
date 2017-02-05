package unittest;

import static org.junit.Assert.*;
import lbfgs4j.LbfgsMinimizer;

import org.junit.Test;

import distribution.DescentFunction;
import distribution.DistributionEstimator;

public class DescentTst {

	@Test
	public void test() {
		
//		double lambda[] = {0,0,0};
		double mu[] = {2.742589337189586, 11.51804604391782};
		int xrange = 19;
		
		DescentFunction df = new DescentFunction(xrange, mu);
		
		 boolean verbose = true;
		 LbfgsMinimizer minimizer = new LbfgsMinimizer(verbose);
		 double[] lambda = minimizer.minimize(df);
		    
		
		
		for(int i=0;i<lambda.length;i++) {
			System.out.println("lambda " + i + ": " +lambda[i]);
		}
//		System.out.println("cost in secs: " + cost);
		double[] prob = df.generate(xrange, lambda);
		String str = "guess1->";
		for(int i=0; i<1; i++) {
			str+= i + ":" +prob[i] +";";
		}
		System.out.println(str);
		
	}
	

}
