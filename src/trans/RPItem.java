package trans;

public class RPItem {
	private String name;
	private int count;
	
	public RPItem(String name, int count) {
		this.name = name;
		this.count = count;
	}

	public int getCount() {
		return count;
	}

	public void setCount(int count) {
		this.count = count;
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}
}
