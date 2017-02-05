package trans;

public class Guard {
	public final static int LESS = 101;
	public final static int LESS_EQUAL = 102;
	public final static int EQUAL = 103;
	public final static int GREATER = 104;
	public final static int GREATER_EQUAL = 105;
	
	private String agent;
	private int operator;
	private int value;
	
	
	public Guard(String agent, int operator, int value) {
		this.agent = agent;
		this.operator = operator;
		this.value = value;
	}
	
	
	public boolean check(double agentValue) {
		if(this.operator == LESS) {
			if(agentValue < value) {
				return true;
			}else {
				return false;
			}
		}
		
		if(this.operator == LESS_EQUAL) {
			if(agentValue <= value) {
				return true;
			}else {
				return false;
			}
		}
		
		if(this.operator == EQUAL) {
			if(agentValue == value) {
				return true;
			}else {
				return false;
			}
		}
		
		if(this.operator == GREATER) {
			if(agentValue > value) {
				return true;
			}else {
				return false;
			}
		}
		
		if(this.operator == GREATER_EQUAL) {
			if(agentValue > value) {
				return true;
			}else {
				return false;
			}
		}
		
		return false;
	}


	public String getAgent() {
		return agent;
	}


	public void setAgent(String agent) {
		this.agent = agent;
	}


	public int getOperator() {
		return operator;
	}


	public void setOperator(int operator) {
		this.operator = operator;
	}


	public int getValue() {
		return value;
	}


	public void setValue(int value) {
		this.value = value;
	}
	
	
}
