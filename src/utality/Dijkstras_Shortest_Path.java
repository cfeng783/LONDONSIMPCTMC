package utality;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
 
public class Dijkstras_Shortest_Path
{
    private double          weights[];
    private Set<Integer> settled;
    private Set<Integer> unsettled;
    private int          number_of_nodes;
    private double          weightMatrix[][];
 
    public Dijkstras_Shortest_Path(int number_of_nodes)
    {
        this.number_of_nodes = number_of_nodes;
        weights = new double[number_of_nodes];
        settled = new HashSet<Integer>();
        unsettled = new HashSet<Integer>();
        weightMatrix = new double[number_of_nodes][number_of_nodes];
    }
 
    public void dijkstra_algorithm(double weightMatrix[][], int source)
    {
        int evaluationNode;
        for (int i = 0; i < number_of_nodes; i++)
            for (int j = 0; j < number_of_nodes; j++)
            	weightMatrix[i][j] = weightMatrix[i][j];
 
        for (int i = 0; i < number_of_nodes; i++)
        {
        	weights[i] = 0;
        }
 
        unsettled.add(source);
        weights[source] = 1;
        while (!unsettled.isEmpty())
        {
            evaluationNode = getNodeWithMaximumWeightFromUnsettled();
            unsettled.remove(evaluationNode);
            settled.add(evaluationNode);
            evaluateNeighbours(evaluationNode);
        }
    }
 
    private int getNodeWithMaximumWeightFromUnsettled()
    {
        double max;
        int node = 0;
 
        Iterator<Integer> iterator = unsettled.iterator();
        node = iterator.next();
        max = weights[node];
        for (int i = 0; i < weights.length; i++)
        {
            if (unsettled.contains(i))
            {
                if (weights[i] >= max)
                {
                    max = weights[i];
                    node = i;
                }
            }
        }
        return node;
    }
 
    private void evaluateNeighbours(int evaluationNode)
    {
        double edgeWeight = -1;
        double newWeight = -1;
 
        for (int destinationNode = 0; destinationNode < number_of_nodes; destinationNode++)
        {
            if (!settled.contains(destinationNode))
            {
                if (weightMatrix[evaluationNode][destinationNode] != 0)
                {
                    edgeWeight = weightMatrix[evaluationNode][destinationNode];
                    newWeight = weights[evaluationNode] * edgeWeight;
                    if (newWeight > weights[destinationNode])
                    {
                    	weights[destinationNode] = newWeight;
                    }
                    unsettled.add(destinationNode);
                }
            }
        }
    }
 
//    public static void main(String... arg)
//    {
//        int adjacency_matrix[][];
//        int number_of_vertices;
//        int source = 0, destination = 0;
//        Scanner scan = new Scanner(System.in);
//        try
//        {
//            System.out.println("Enter the number of vertices");
//            number_of_vertices = scan.nextInt();
//            adjacency_matrix = new int[number_of_vertices + 1][number_of_vertices + 1];
// 
//            System.out.println("Enter the Weighted Matrix for the graph");
//            for (int i = 1; i <= number_of_vertices; i++)
//            {
//                for (int j = 1; j <= number_of_vertices; j++)
//                {
//                    adjacency_matrix[i][j] = scan.nextInt();
//                    if (i == j)
//                    {
//                        adjacency_matrix[i][j] = 0;
//                        continue;
//                    }
//                    if (adjacency_matrix[i][j] == 0)
//                    {
//                        adjacency_matrix[i][j] = Integer.MAX_VALUE;
//                    }
//                }
//            }
// 
//            System.out.println("Enter the source ");
//            source = scan.nextInt();
// 
//            System.out.println("Enter the destination ");
//            destination = scan.nextInt();
// 
//            Dijkstras_Shortest_Path dijkstrasAlgorithm = new Dijkstras_Shortest_Path(
//                    number_of_vertices);
//            dijkstrasAlgorithm.dijkstra_algorithm(adjacency_matrix, source);
// 
//            System.out.println("The Shorted Path from " + source + " to " + destination + " is: ");
//            for (int i = 1; i <= dijkstrasAlgorithm.distances.length - 1; i++)
//            {
//                if (i == destination)
//                    System.out.println(source + " to " + i + " is "
//                            + dijkstrasAlgorithm.distances[i]);
//            }
//        } catch (InputMismatchException inputMismatch)
//        {
//            System.out.println("Wrong Input Format");
//        }
//        scan.close();
//    }
}