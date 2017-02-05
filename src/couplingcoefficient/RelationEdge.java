package couplingcoefficient;

import java.util.ArrayList;

public class RelationEdge {
	private String headNodeName;
	private String tailNodeName;
	private String weight;
	private String numerator;
	private String denominator;
	private double value;
	//private ArrayList<Double> sampleValues;
	
	public RelationEdge(String head, String tail, String weight, String numerator, String denominator) {
		this.headNodeName = head;
		this.tailNodeName = tail;
		this.weight = weight;
		this.numerator = numerator;
		this.denominator = denominator;
		//sampleValues = new ArrayList<Double>();
	}
	
	
	public static String genEdgeKey(String headNode, String tailNode) {
		return headNode + "_to_" + tailNode;
	}

	public String getHeadNodeName() {
		return headNodeName;
	}

	public void setHeadNodeName(String headNodeName) {
		this.headNodeName = headNodeName;
	}

	public String getTailNodeName() {
		return tailNodeName;
	}

	public void setTailNodeName(String tailNodeName) {
		this.tailNodeName = tailNodeName;
	}

	public String getWeight() {
		return weight;
	}

	public void setWeight(String weight) {
		this.weight = weight;
	}
	
	@Override
	public String toString() {
		return "From " + headNodeName + " to " + tailNodeName + ": " + value ; 
	}


	public double getValue() {
		return value;
	}


	public void setValue(double value) {
		this.value = value;
	}


	public String getNumerator() {
		return numerator;
	}


	public void setNumerator(String numerator) {
		this.numerator = numerator;
	}


	public String getDenominator() {
		return denominator;
	}


	public void setDenominator(String denominator) {
		this.denominator = denominator;
	}


//	public ArrayList<Double> getSampleValues() {
//		return sampleValues;
//	}
//	
//
//	public void setSampleValues(ArrayList<Double> sampleValues) {
//		this.sampleValues = sampleValues;
//	}
//	
//	public double getSampleMean() {
//		double total = 0;
//		for(int i=0; i<sampleValues.size(); i++) {
//			if(Double.isNaN(sampleValues.get(i))) {
//				sampleValues.set(i, 0.0);
//			}
//			total += sampleValues.get(i);
////			if(this.headNodeName.equals("Bike(8,8)") && this.tailNodeName.equals("ReturnBike(8,8)")) {
////				System.out.print(sampleValues.get(i) + " ");
////			}
//		}
//		if(total == 0) {
//			return 0;
//		}
//		
//		return total / sampleValues.size();
//	}
//	
//	public double getSampleVariance() {
//		double total = 0;
//		double mean = getSampleMean();
//		if(mean == 0) {
//			return 0;
//		}
//		for(int i=0; i<sampleValues.size(); i++) {
//			total += Math.pow(sampleValues.get(i)-mean, 2) ;
//		}
//		return total / sampleValues.size();
//	}
}
