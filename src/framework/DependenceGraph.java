package framework;

import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;


public class DependenceGraph {
	double[][] dependenceMatrix;
	double[][] directDpMatrix;
	double[][] maxDpMatrix;
	int num;
	final double threshold = 0;
	
	public DependenceGraph(int num, int numOfInwardTrips[], int numOfOutwardTrips[], int numOfTrips[][]) {
		this.num = num;
		dependenceMatrix = new double[num][num];
		this.maxDpMatrix = new double[num][num];
		this.directDpMatrix = new double[num][num];
		
		for(int i=0; i<num; i++) {
			for(int j=0; j<num; j++) {
				if(i == j) {
					if(numOfInwardTrips[i] == 0) {
						directDpMatrix[i][j] = 0;
					}else {
						directDpMatrix[i][j] = (numOfTrips[j][i]*1.0) / numOfInwardTrips[i];
						if(directDpMatrix[i][j] < threshold) {
							directDpMatrix[i][j] = 0;
						}
					}
				}else {
					if(numOfInwardTrips[i] == 0) {
						directDpMatrix[i][j] = 0;
					}else {
						directDpMatrix[i][j] = (numOfTrips[j][i]*1.0) / numOfInwardTrips[i];
						if(directDpMatrix[i][j] < threshold) {
							directDpMatrix[i][j] = 0;
						}
					}
					
				}
			}
		}
		
//		for(int i=0; i<num; i++) {
//			for(int j=0; j<num; j++) {
//				if(i == j) {
//					if(numOfInwardTrips[i]+numOfOutwardTrips[i] == 0) {
//						directDpMatrix[i][j] = 0;
//					}else {
//						directDpMatrix[i][j] = (numOfTrips[i][j]*1.0 + numOfTrips[j][i]) / (numOfInwardTrips[i]+numOfOutwardTrips[i]) * 1.0;
//						if(directDpMatrix[i][j] < threshold) {
//							directDpMatrix[i][j] = 0;
//						}
//					}
//				}else {
//					if(numOfInwardTrips[i]+numOfOutwardTrips[i] == 0) {
//						directDpMatrix[i][j] = 0;
//					}else {
//						directDpMatrix[i][j] = (numOfTrips[i][j]*1.0 + numOfTrips[j][i]) / (numOfInwardTrips[i]+numOfOutwardTrips[i]) * 1.0;
//						if(directDpMatrix[i][j] < threshold) {
//							directDpMatrix[i][j] = 0;
//						}
//					}
//					
//				}
//			}
//		}
//		validateDirectDpMatrix();
//		print(dependenceMatrix);
//		print(directDpMatrix[1]);
		for(int i=0; i<num; i++) {
			directDpMatrix[i][i] = 0;
		}
		
		for(int i=0; i<num; i++) {
			for(int j=0; j<num; j++) {
				dependenceMatrix[i][j] = directDpMatrix[i][j];
				maxDpMatrix[i][j] = directDpMatrix[i][j];
			}
		}
		
		for(int k=0; k<num; k++) {
			for(int i=0; i<num; i++) {
				for(int j=0; j<num; j++) {
					dependenceMatrix[i][j] += dependenceMatrix[i][k] * dependenceMatrix[k][j];
					if(maxDpMatrix[i][j] < maxDpMatrix[i][k] * maxDpMatrix[k][j]) {
						maxDpMatrix[i][j] = maxDpMatrix[i][k] * maxDpMatrix[k][j];
					}
				}
			}
		}
		
//		for(double sig=0.1; sig>0; sig=sig-0.01) {
//			checkCorrelation(sig);
//		}
		
//		print(maxDpMatrix[1]);
//		validateDependenceMatrix();
//		print(dependenceMatrix[1]);
//		print(maxDpMatrix);
	}
	
	public double[][] getDependenceMatrix() {
		return this.dependenceMatrix;
	}
	
	public double[][] getMaxDpMatrix() {
		return this.maxDpMatrix;
	}
	
	public double[][] getDirectDpMatrix() {
		return this.directDpMatrix;
	}
	
	public void validateDirectDpMatrix() {
		double min = 100;
		double max = -1;
		double avg = 0;
		for(int i=0; i<directDpMatrix.length; i++) {
			double total = 0;
			for(int j=0; j<directDpMatrix[i].length; j++) {
				total += directDpMatrix[i][j];
			}
			if(total != 0 && total<min) {
				min = total;
			}
			if(total != 0 && total>max) {
				max = total;
			}
			avg += total;
//			System.out.println(total);
			
		}
		avg = avg/directDpMatrix.length;
		System.out.println("min:" +min);
		System.out.println("max:" +max);
		System.out.println("avg:" +avg);
	}
	
	public void validateDependenceMatrix() {
		double min = 100;
		double max = -1;
		for(int i=0; i<dependenceMatrix.length; i++) {
			for(int j=0; j<dependenceMatrix[i].length; j++) {
				if(dependenceMatrix[i][j]<min) {
					min = dependenceMatrix[i][j];
				}
				if(dependenceMatrix[i][j]>max) {
					max = dependenceMatrix[i][j];
				}
			}
			
		}
		System.out.println("min:" +min);
		System.out.println("max:" +max);
	}
	
	public void print(double matrix[][]) {
		
		try{
			
			PrintWriter sw = new PrintWriter(new OutputStreamWriter(new FileOutputStream("matrix.txt")),true);  
			for(int i=0; i<matrix.length; i++) {
				String str = "";
				double min = 100;
				double max = -1;
				for(int j=0; j<matrix[i].length; j++) {
					str += matrix[i][j] + " ";
					if(matrix[i][j] > max) {
						max = matrix[i][j];
					}
				}
				System.out.println("max="+max);
				sw.println(str);
			}
			sw.close();
			
			
		}catch(Exception e) {
			e.printStackTrace();
		}
		
		
	}
	
	
	public void print(int matrix[][]) {
		
		try{
			
			PrintWriter sw = new PrintWriter(new OutputStreamWriter(new FileOutputStream("matrix.txt")),true);  
			for(int i=0; i<matrix.length; i++) {
				String str = "";
				for(int j=0; j<matrix[i].length; j++) {
					str += matrix[i][j] + " ";
				}
				sw.println(str);
			}
			sw.close();
			
			
		}catch(Exception e) {
			e.printStackTrace();
		}
		
		
	}

	public void print(double matrix[]) {
		double tempt[] = new double[matrix.length];
		for(int i=0; i<matrix.length; i++) {
				tempt[i] = matrix[i];
		}
		Arrays.sort(tempt);
		
		String str = "";
		for(int i=0; i<matrix.length; i++) {
				str += tempt[i] + " ";
			
		}
		System.out.println(str);
	}
	
	public void checkCorrelation(double sig) {
		int hit = 0;
		int fpos = 0;
		int fneg = 0;
		for(int i=0; i<num; i++) {
			for(int j=0; j<num; j++) {
				if(maxDpMatrix[i][j] <= sig) {
					if(dependenceMatrix[i][j] <= 0.1) {
						hit++;
					}else {
						fneg++;
					}
				}else if(maxDpMatrix[i][j] >= sig) {
					if(dependenceMatrix[i][j] >= 0.1) {
						hit++;
					}else {
						fpos++;
					}
				}
			}
		}
		int miss = fneg + fpos;
		System.out.println("sig: " + sig);
		System.out.println("hit: " + hit);
		System.out.println("miss: " + miss);
		double missProb = miss*1.0/(hit+miss);
		System.out.println("missProb: " + missProb);
		System.out.println("false negative: " + fneg);
		double fnegProb = fneg*1.0/(hit+miss);
		System.out.println("false negative prob: " + fnegProb);
		
	}
}
