package utality;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.util.ArrayList;

public class Cache {
	public final static int READ = 101;
	public final static int WRITE = 102;
	
	private final String filename = "random.txt";
	ArrayList<String> cache;
	int curIndex = 0;
	int mode;
	
	public Cache(int mode) {
		this.mode = mode;
		cache = new ArrayList<String>();
		if(mode == READ) {
			try {
				BufferedReader bf = new BufferedReader( new InputStreamReader(new FileInputStream(filename) )  );
				String data;
				
				while((data = bf.readLine())!=null)  
				{  	
					cache.add(data);
				}
				
				bf.close();
			}catch(Exception e) {
				e.printStackTrace();
			}
			
		}
	}
	
	public int nextInt() {
		int ret = Integer.parseInt(cache.get(curIndex));
		curIndex ++;
		return ret;
	}
	
	public double nextDouble() {
		double ret = Double.parseDouble(cache.get(curIndex));
		curIndex ++;
		return ret;
	}
	
	public void cacheInt(int num) {
		cache.add(num+"");
	}
	
	public void cacheDouble(double num) {
		cache.add(num + "");
	}
	
	public void close() {
		if(mode == WRITE) {
			try {
				PrintWriter sw = new PrintWriter(new OutputStreamWriter(new FileOutputStream(filename)),true); 
				for(int i=0; i<cache.size(); i++) {
					sw.println(cache.get(i));
				}
				sw.close();
			}catch(Exception e) {
				e.printStackTrace();
			}
			
			
		}
	}
}
