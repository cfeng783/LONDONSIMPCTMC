package utality;

import java.util.Random;

import bsh.Interpreter;


public class Utality {
	static Random rnd = new Random();
	static Interpreter interpreter;
	public final static double ERROR_NUM = -18943.43439;
	public final static String sigma = "eps";
	static Cache cache;
	
	public static void init(int cacheMode) {
		rnd.setSeed(System.currentTimeMillis());
		interpreter = new Interpreter();
		cache = new Cache(cacheMode);
	}
	
	public static Random getRandom() {
		return rnd;
	}
	
	public static int nextInt(int n) {
		if(cache.mode == Cache.READ) {
			return cache.nextInt();
		}else {
			int ret = rnd.nextInt(n);
			cache.cacheInt(ret);
			return ret;
		}
	}
	
	public static double nextDouble() {
		if(cache.mode == Cache.READ) {
			return cache.nextDouble();
		}else {
			double ret = rnd.nextDouble();
			cache.cacheDouble(ret);
			return ret;
		}
	}
	
	public static void close() {
		cache.close();
	}
	
	public static double evalDRGExp(String expr) {
		expr = expr.trim();
		double ret = Utality.evalParamExpressionForSimulation(expr);
		return ret;
	}
	
	public static double evalParamExpressionForSimulation(String expr) {
		double ret = 0;
		try{
			expr = "ret=(double) " + expr;
			expr = expr.replaceAll("'", "\"");
			expr = expr.replaceAll("sqrt", "Math.sqrt");
			expr = expr.replaceAll("pow", "Math.pow");
			expr = expr.replaceAll("abs", "Math.abs");
			expr = expr.replaceAll("max", "Math.max");
			//System.out.println(expr);	
			Object res = interpreter.eval(expr);
			ret = (Double)interpreter.get("ret");
			//System.out.println("ret: " + ret);
			//System.out.println("res: " + res.toString());
			return ret;
		}catch(Exception e) {
			e.printStackTrace();
			return ERROR_NUM;
		}
	}
}
