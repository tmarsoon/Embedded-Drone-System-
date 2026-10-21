package application;

//class to represent troop's name and military branch. 
public class Troop {
	//troop name
    private String name;
    //troop branch
    private String branch;

    //creating constructor for initializing the troop's name and branch
    public Troop(String name, String branch) {
        this.name = name;
        this.branch = branch;
    }
    //getter for troop name
    public String getName() {
        return name;
    }
    //getter for branch
    public String getBranch() {
        return branch;
    }
    //overriding default toString with formatted troop's name and branch
    @Override
    public String toString() {
        return name + " (" + branch + ")";
    }
}

