package unittest;

import static org.junit.Assert.*;
import odesolver.TestSolver;

import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.ode.sampling.StepHandler;
import org.apache.commons.math3.ode.sampling.StepInterpolator;
import org.apache.commons.math3.special.Beta;
import org.junit.Test;

import distribution.ApproximateDistribution;
import distribution.DistributionEstimator;
import distribution.GradientDescent;
import framework.DependenceGraph;
import framework.ParserLondonBike;

public class PaserTest {
	
	@Test
	public void test() {
		
		
		double lambda[] = {0,0,0};
		double mu[] = {23.334309683506948, 556.6139537299879, 13557.34443463226};
		int xrange = 28;
		
		double p = ApproximateDistribution.generateBinomial(mu[0], xrange);
		double guess1 = ApproximateDistribution.getBinomialProb(p, xrange);
		System.out.println(guess1);
		
		double beta[] = ApproximateDistribution.generateBetaBinomial(mu[0],mu[1],xrange);
		System.out.println("a: " + beta[0]);
		System.out.println("b: " + beta[1]);
		
		double guess2 = ApproximateDistribution.getBetaBinomialProb(beta[0], beta[1], xrange);
		System.out.println(guess2);
	}

}
