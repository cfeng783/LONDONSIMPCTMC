package framework;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Calendar;


public class InitStateParser {
	private int[] numOfBikes;
	private int[][] numOfBikeInTrans;
	private String time;
	private int[] numOfBikesFinal;
	
	private final int startHour = 8;
	private final int endHour = 9;
	private int[][] phases;
	
	public static double[] fullprob;
	public static double[] emptyprob;
	
	
	public void initFullandEmptyProbs(int[] capacity) {
		if(fullprob != null) {
			return ;
		}
		String foldername2 = System.getProperty("user.home")+"/Desktop/cycle data/processedSnapshots/";
		File folder2 = new File(foldername2);
		File[] listOfFiles2 = folder2.listFiles(); 
		int[] emptyCases = new int[capacity.length];
		int[] fullCases = new int[capacity.length];
		int total = 0;
		
		for(int f = 0; f < listOfFiles2.length; f++) {
		   if (listOfFiles2[f].isFile()) {
			   String filename = foldername2 + "/" + listOfFiles2[f].getName();
			   BufferedReader br = null;
			   try {
					br = new BufferedReader(new FileReader(filename));
					String line = br.readLine();
					String[] strInitBikes = line.split(",");
					int[] nums = new int[strInitBikes.length];
					for(int i=0; i<nums.length; i++) {
						nums[i] = Integer.parseInt(strInitBikes[i]);
						if(nums[i] == capacity[i]) {
							fullCases[i] ++;
						}
						if(nums[i] == 0) {
							emptyCases[i] ++;
						}
					}
					total ++;
					
				} catch (FileNotFoundException e) {
					e.printStackTrace();
				} catch (IOException e) {
					e.printStackTrace();
				} catch (Exception e) {
					e.printStackTrace();
				}finally {
					if (br != null) {
						try {
							br.close();
						} catch (IOException e) {
							e.printStackTrace();
						}
					}
				}
			   
		   }
		}
		
		fullprob = new double[capacity.length];
		emptyprob = new double[capacity.length];
		for(int i=0; i<capacity.length; i++) {
			fullprob[i] = fullCases[i]*1.0/total;
			emptyprob[i] = emptyCases[i]*1.0/total;
			System.out.println(i+": " + fullprob[i] + " , " + emptyprob[i]);
		}
	}
	
	public InitStateParser() {}
	
	public void parseFile(String filename) {
		this.time = filename;
		BufferedReader br = null;
		try {
			br = new BufferedReader(new FileReader(System.getProperty("user.home")+"/Desktop/cycle data/processedSnapshots/"  + filename));
			String line = br.readLine();
			String[] strInitBikes = line.split(",");
			numOfBikes = new int[strInitBikes.length];
			for(int i=0; i<numOfBikes.length; i++) {
				numOfBikes[i] = Integer.parseInt(strInitBikes[i]);
			}
			
			numOfBikeInTrans = new int[numOfBikes.length][numOfBikes.length];
			
			while ((line = br.readLine()) != null) {
				line = line.trim();
			    String[] splits = line.split("#");
			    int startIndex = Integer.parseInt(splits[0]);
			    
			    String[] num = splits[1].split(",");
			    if(startIndex < numOfBikes.length) {
			    	for(int i=0; i<numOfBikes.length; i++) {
				    	numOfBikeInTrans[startIndex][i] = Integer.parseInt(num[i]);
				    }
			    }	
			}
			
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		}finally {
			if (br != null) {
				try {
					br.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		
		initFinalBikes();
		initPhases();
	}
	
	
	private void initPhases() {
		
		phases = new int[numOfBikes.length][numOfBikes.length];
		BufferedReader br = null;
		try {
			br = new BufferedReader(new FileReader(System.getProperty("user.home")+"/Desktop/cycle data/phases/phase.txt"));
			int i=0;
			
			String line = null;
			while((line= br.readLine()) != null) {
				String[] strPhases = line.split(" ");
				
				for(int j=0; j<strPhases.length; j++) {
					phases[i][j] = Integer.parseInt(strPhases[j]);
					if(phases[i][j] < 1)
					System.out.print(phases[i][j]+" ");
				}
				i++;
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		}finally {
			if (br != null) {
				try {
					br.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
	}
	
	
	private void initFinalBikes() {
		long timepoint = Long.parseLong(time);
		Calendar cal = Calendar.getInstance();
		cal.setTimeInMillis(timepoint);
		int day = cal.get(Calendar.DAY_OF_YEAR);
		BufferedReader br = null;
		try {
			br = new BufferedReader(new FileReader(System.getProperty("user.home")+"/Desktop/cycle data/nineSnapshots/"  + day));
			String line = br.readLine();
			String[] strFinalBikes = line.split(",");
			numOfBikesFinal = new int[strFinalBikes.length];
			for(int i=0; i<numOfBikesFinal.length; i++) {
				numOfBikesFinal[i] = Integer.parseInt(strFinalBikes[i]);
			}
			
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		}finally {
			if (br != null) {
				try {
					br.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		
	}
	
	public int getHour() {
		long timepoint = Long.parseLong(time);
		Calendar cal = Calendar.getInstance();
		cal.setTimeInMillis(timepoint);
		int hour = cal.get(Calendar.HOUR_OF_DAY);
		System.out.println(cal.get(Calendar.MONTH) + ":"+ cal.get(Calendar.DAY_OF_MONTH)+":"+hour + ":" + cal.get(Calendar.MINUTE));
		return hour;
	}
	
	public int getMinute() {
		long timepoint = Long.parseLong(time);
		Calendar cal = Calendar.getInstance();
		cal.setTimeInMillis(timepoint);
		int hour = cal.get(Calendar.MINUTE);
		return hour;
	}
	
	public int getStartSlot() {
		int hour = this.getHour();
		int minute = this.getMinute();
		if(hour < this.startHour) {
			return -1;
		}
		if(hour >= this.endHour) {
			return -1;
		}
		return minute/ParserLondonBike.interval;
	}
	
	public int[] getAlterPoints() {
		int minute = this.getMinute();
		int first = (60*(endHour-startHour)-minute) % ParserLondonBike.interval;
		int slots = (60*(endHour-startHour)-minute) / ParserLondonBike.interval + 1;
		
		if(first == 0) {
			slots--;
		}
		
		int[] ret = new int[slots];
		String str = "";
		for(int i=0; i<slots; i++) {
			if(first == 0) {
				ret[i] = first + ParserLondonBike.interval * (i+1);
				str += ret[i] + " ";
			}else {
				ret[i] = first + ParserLondonBike.interval * i;
				str += ret[i] + " ";
			}
			
		}
		System.out.println("alterPoints: " + str);
		return ret;
	}
	
	public int getTimeLength() {
		int minute = this.getMinute();
		return	60*(endHour-startHour)-minute;
	}

	public int[] getNumOfBikes() {
		return numOfBikes;
	}

	public void setNumOfBikes(int[] numOfBikes) {
		this.numOfBikes = numOfBikes;
	}

	public int[][] getNumOfBikeInTrans() {
		return numOfBikeInTrans;
	}
	
	public int[] getNumOfBikesFinal() {
		return numOfBikesFinal;
	}

	public void setNumOfBikeInTrans(int[][] numOfBikeInTrans) {
		this.numOfBikeInTrans = numOfBikeInTrans;
	}
	
	public int[][] getPhases() {
		return this.phases;
	}

	public double[] getFullprob() {
		return fullprob;
	}

	public void setFullprob(double[] fullprob) {
		this.fullprob = fullprob;
	}

	public double[] getEmptyprob() {
		return emptyprob;
	}

	public void setEmptyprob(double[] emptyprob) {
		this.emptyprob = emptyprob;
	}
}
