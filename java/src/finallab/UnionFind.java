package finallab;

public class UnionFind 
{
	private int[] data;

	public UnionFind(int size)
	{
		data = new int[size];
		//A value of -1 indicates this element is a root node.
		for(int i=0;i<size;i++)
		{
			data[i] = -1;
		}
	}

	public void join(int a, int b)
	{
		//Get the root nodes of the requested values.
		int roota = find(a);
		int rootb = find(b);
		
		//Check if two indices are already part of the same union.
		if(roota == rootb)
			return;
		//Merge the two, actual process of "merging" is put off until a find is run on a member with roota
		if (data[rootb] < data[roota]) 
		{
      		data[rootb] += data[roota];
      		data[roota] = rootb;
		}
		else 
		{
      		data[roota] += data[rootb];
      		data[rootb] = roota;
    	}
	}

	public int find(int a)
	{
		if(data[a] < 0)
			return a;
		//Make future accesses quicker by updating the structure values to be that of its root.
		data[a] = find(data[a]);
		return data[a];
	}
}
