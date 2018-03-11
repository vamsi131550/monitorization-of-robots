/**
* Course: CS-684 Embedded Systems
* Project Title: Multi-Bot Controller
* Group Number: 4
* Group Members:
* (1) Gaurav Vijayvargia (133050031) - Team Leader
* (2) Thyagarajan Radhakrishnan (13305R004)
* (3) Vaibhav Dave (13V050002)
* (4) Jyoti Shankar (133050080)
*
* Problem Statement: Development of a central controller for allocation of minimally shared shortest path in a multi-robot scenario.
*
* Final System (Deliverables):
* (1) A graph based solution to generate the shortest path with minimal path sharing
* (2) A GUI for specifying the source and destination node (based on a greenhouse model arena) for a particular robot
* (3) Wireless communication (using ZigBee) for packet exchange between central controller and robots. Communication between central controller and ZigBee is established through serial port (COM port).
*
* NOTE: Prior knowledge of Java is essential for understanding the code. Some topics include fundamentals (classes, interfaces, inheritance), I/O, Serial Port communication, data structures like arrays & list, AWT and Swings (for GUI).
*/

/**
* Importing the necessary packages
*/

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.*;
import javax.swing.*;
import javax.imageio.ImageIO;
import javax.comm.*;

/**
* Shortest path calculation between source and destination node is done using "Dijkstra's shortest path algorithm".
*
* Reusable components in the code which are part of Dijkstra's algorithm:
*
* Classes: Vertex, Edge
*
* Functions in the main class BackgroundImageJframe:
* (1) void computePaths(Vertex)
* (2) List<Vertex> getShortestPathTo(Vertex)
* (3) void setPreviousNull()
* (4) double computePathWeight(List<Vertex>)
* (5) void updatePathWeights(List<Vertex>, int)
* (6) void updateAdjacencyWeights(Vertex, Vertex, int, int)
*/

/**
* Class Name: Vertex
* Description: Represents a vertex in a graph. Implements the Comparable interface (compareTo() method) for comparison between any 2 vertices based on minimum distance during shortest path calculation using Dijkstra algorithm.
* Purpose (for Project): Represents the sensing (and non-sensing) points in a greenhouse model arena for the purpose of graph representation
*/

class Vertex implements Comparable<Vertex>
{
    public final String name;
    public Edge[] adjacencies;
    public double minDistance = Double.POSITIVE_INFINITY;
    public Vertex previous;
    public Vertex(String argName) { name = argName; }
    public String toString() { return name; }
    public int compareTo(Vertex other)
    {
        return Double.compare(minDistance, other.minDistance);
    }
}


/**
* Class Name: Edge
* Description: Represents an edge between 2 vertices in a graph. Each edge is assigned a weight for shortest path calculation using Dijkstra algorithm.
* Purpose (for Project): The weight of an edge at any point in time gives a measure of the level of sharing (in use) of that edge among the robots.
*/

class Edge
{
    public final Vertex target;
    public double weight;
    public Edge(Vertex argTarget, double argWeight)
    {
	target = argTarget; weight = argWeight;
    }
}

/*
* Class Name: BackgroundImageJframe
* Description: The main class which implements the following:
* (1) Dijkstra's algorithm for computation of minimally shared shortest path
* (2) Mapping the generated path from (1) which are a sequence of vertices, into a set of commands that can be executed by the robots for path traversal
* (3) Serial port communication for sending the commands to the robots or receiving acknowledgement from them (via ZigBee)
* (4) A GUI for the user to specify inputs i.e. selecting a robot and based on it's current position, choosing it's destination. The "Robot Traversal Log"  in the GUI will indicate the (busy) status of the robot once a (traversal) task is assigned to it.
*/

public class BackgroundImageJframe extends JFrame implements ActionListener, SerialPortEventListener{

    private static final long serialVersionUID = 1L;
    JButton submit, clear;
    JLabel l1,l2,l3;
    JComboBox botList, src, dest;
    JTextArea log;
    JScrollPane scrollPane;
    int source, destination, botSelected, action=0;
    char pathc[];

// Indicates the number of robots in the system. It's value must be changed depending on the number of robots that need to be controlled. 
    int MAX_ROBOT = 2;	

// This is arena-specific. There are 3 sensing points in each lane along the X-direction based on the model arena.
    int MAX_X = 3;

// This is arena-specific. There are 5 such lanes in the model arena. 
    int MAX_Y = 5;

// Together, they are used to ID the sensing points (1 to MAX_X * MAX_Y)
    String[] sensei = new String[MAX_X*MAX_Y];

// Used to give ID to the robots (1 to MAX_ROBOT).
    String[] botStrings = new String[MAX_ROBOT];

/** Indicates the initial direction of the robots. As per the image of the model arena, right direction -> N, left direction -> S, up direction -> E, down direction -> W. We assume robots always start/end at some
* sensing point and hence can be in N/S direction only. 
*/
    char[] direction = {'N','S'}; 

// Status of the robots: 0 -> Not Busy, 1 -> Busy. The number of elements in the array will change depending on the number of robots.
    int[] botBusy = {0,0}; 

// Initial position of the robots. Can take any value from 1 to MAX_X*MAX_Y. The number of elements in the array will change depending on the number of robots. This value will get updated after every traversal as the destination of a previous traversal for a robot will be the source for the next traversal task assigned.
    int[] initPos = {1,9};

// For Serial Port communication	
    static Enumeration ports;
    static CommPortIdentifier pID;
    static InputStream inStream;
    static OutputStream outStream;
    static SerialPort serPort;

// Instantiating the sensing and non-sensing points in the model arena. Based on the arena, the naming convention followed for graph representation is:

/**  a0 ---- a1 ---- a2 ---- a3 ---- a4
*     |                               |
*    b0 ---- b1 ---- b2 ---- b3 ---- b4
*     |                               |
*    c0 ---- c1 ---- c2 ---- c3 ---- c4
*     |                               |
*    d0 ---- d1 ---- d2 ---- d3 ---- d4
*     |                               |
*    e0 ---- e1 ---- e2 ---- e3 ---- e4
*/

    Vertex a0 = new Vertex("a0");
    Vertex a1 = new Vertex("a1");
    Vertex a2 = new Vertex("a2");
    Vertex a3 = new Vertex("a3");
    Vertex a4 = new Vertex("a4");
    Vertex b0 = new Vertex("b0");
    Vertex b1 = new Vertex("b1");
    Vertex b2 = new Vertex("b2");
    Vertex b3 = new Vertex("b3");
    Vertex b4 = new Vertex("b4");
    Vertex c0 = new Vertex("c0");
    Vertex c1 = new Vertex("c1");
    Vertex c2 = new Vertex("c2");
    Vertex c3 = new Vertex("c3");
    Vertex c4 = new Vertex("c4");
    Vertex d0 = new Vertex("d0");
    Vertex d1 = new Vertex("d1");
    Vertex d2 = new Vertex("d2");
    Vertex d3 = new Vertex("d3");
    Vertex d4 = new Vertex("d4");
    Vertex e0 = new Vertex("e0");
    Vertex e1 = new Vertex("e1");
    Vertex e2 = new Vertex("e2");
    Vertex e3 = new Vertex("e3");
    Vertex e4 = new Vertex("e4");

    Vertex[] vertices = { a0, a1, a2, a3, a4, b0, b1, b2, b3, b4, c0, c1, c2, c3, c4, d0, d1, d2, d3, d4, e0, e1, e2, e3, e4 };

// Once a robot is assigned a path for traversal, all the edge weights except for the last edge are increased by some random amount.
    static int nonLastEdgeWeight = 30;

/** Once a robot is assigned a path for traversal, the weight of the last edge is increased by some random LARGE amount. This is because that edge is connected to the ultimate destination of that robot.
* At this point, collision possibility is maximum if any other robot is assigned path via that edge. Increasing the weight by a LARGE amount ensures that the shortest path calculated wont involve that route.
*/
    static int lastEdgeWeight = 1000;

// Used to store the path assigned to the robot. Number of variables will increase depending on the number of robots. (Future Work: Make it an array of Lists of size MAX_ROBOT).
    List<Vertex> busyPath1 = null;
    List<Vertex> busyPath2 = null;


// Dijkstra's algorithm: Computes the single 'source' shortest path (in the form of List of vertices)
    public static void computePaths(Vertex source)
    {
        source.minDistance = 0.;
        PriorityQueue<Vertex> vertexQueue = new PriorityQueue<Vertex>();
      	vertexQueue.add(source);

	while (!vertexQueue.isEmpty()) {
	    Vertex u = vertexQueue.poll();

            // Visit each edge exiting u
            for (Edge e : u.adjacencies)
            {
                Vertex v = e.target;
                double weight = e.weight;
                double distanceThroughU = u.minDistance + weight;
		if (distanceThroughU < v.minDistance)
		{
		    vertexQueue.remove(v);
		    v.minDistance = distanceThroughU ;
		    v.previous = u;
		    vertexQueue.add(v);
		}
            }
        }
    }


// Used to retrieve the shortest path to the target vertex i.e. destination
    public static List<Vertex> getShortestPathTo(Vertex target)
    {	
	int i=0;
        List<Vertex> path = new ArrayList<Vertex>();
        for (Vertex vertex = target; vertex != null; vertex = vertex.previous)
	{
            path.add(vertex);
            System.out.println("ABCD:"+ ++i);
	}
        Collections.reverse(path);
        return path;
    }


// This reinitialisation function needs to be called before every call (a new run) to Dijkstra's algorithm for shortest path calculation
    public void setPreviousNull()
    {
    	a0.previous = null; a0.minDistance = Double.POSITIVE_INFINITY;
    	a1.previous = null; a1.minDistance = Double.POSITIVE_INFINITY;
    	a2.previous = null; a2.minDistance = Double.POSITIVE_INFINITY;
    	a3.previous = null; a3.minDistance = Double.POSITIVE_INFINITY;
    	a4.previous = null; a4.minDistance = Double.POSITIVE_INFINITY;
    	b0.previous = null; b0.minDistance = Double.POSITIVE_INFINITY;
    	b1.previous = null; b1.minDistance = Double.POSITIVE_INFINITY;
    	b2.previous = null; b2.minDistance = Double.POSITIVE_INFINITY;
    	b3.previous = null; b3.minDistance = Double.POSITIVE_INFINITY;
    	b4.previous = null; b4.minDistance = Double.POSITIVE_INFINITY;
    	c0.previous = null; c0.minDistance = Double.POSITIVE_INFINITY;
    	c1.previous = null; c1.minDistance = Double.POSITIVE_INFINITY;
    	c2.previous = null; c2.minDistance = Double.POSITIVE_INFINITY;
    	c3.previous = null; c3.minDistance = Double.POSITIVE_INFINITY;
    	c4.previous = null; c4.minDistance = Double.POSITIVE_INFINITY;
    	d0.previous = null; d0.minDistance = Double.POSITIVE_INFINITY;
    	d1.previous = null; d1.minDistance = Double.POSITIVE_INFINITY;
    	d2.previous = null; d2.minDistance = Double.POSITIVE_INFINITY;
    	d3.previous = null; d3.minDistance = Double.POSITIVE_INFINITY;
    	d4.previous = null; d4.minDistance = Double.POSITIVE_INFINITY;
    	e0.previous = null; e0.minDistance = Double.POSITIVE_INFINITY;
    	e1.previous = null; e1.minDistance = Double.POSITIVE_INFINITY;
    	e2.previous = null; e2.minDistance = Double.POSITIVE_INFINITY;
    	e3.previous = null; e3.minDistance = Double.POSITIVE_INFINITY;
    	e4.previous = null; e4.minDistance = Double.POSITIVE_INFINITY;
    }


// Computes the weight of the calculated path (passed as argument). Useful to check for collision detection (weight > lastEdgeWeight).
    public static double computePathWeight(List<Vertex> path)
    {
    	double weight=0;
    	
    	for (int i=0; i<path.size()-1; i++)
    	{
    	    Edge[] e = path.get(i).adjacencies;
            for (int j=0; j<e.length; j++)
            {
        	if (e[j].target.name.equals(path.get(i+1).name))
        	{
        	    weight += e[j].weight;
        	    break;
        	}
            }
    	}
    	
    	return weight;
    }
    

// Used to update the 'path' weights once that path has been assigned/released to some robot. Value of 'done' indicates whether a path is being assigned (=0) or being released (=1).
    public static void updatePathWeights(List<Vertex> path, int done)
    {
        int i;
        for (i=0; i<path.size()-2; i++)
        {
            updateAdjacencyWeights(path.get(i),path.get(i+1),0,done);
        }
        updateAdjacencyWeights(path.get(i),path.get(i+1),1,done);
    }


// Called from the above function to update the (adjacency) edge weight between Vertex v1 & v2. Value of 'last' indicates whether it is the last edge (=1) of the path or not (=0) for increasing/decreasing the edge weights accordingly (based on value of 'done' passed from above)
    public static void updateAdjacencyWeights(Vertex v1, Vertex v2, int last, int done)
    {
        int i;
        Edge[] e = v1.adjacencies;
        for (i=0; i<e.length; i++)
        {
            if (last==0)
            {
        	if (e[i].target.name.equals(v2.name))
                {
        	    if (done==0)
        		e[i].weight = e[i].weight + nonLastEdgeWeight;
        	    else
        		e[i].weight = e[i].weight - nonLastEdgeWeight;
        	    break;
                }
            }
            else
            {
        	if (e[i].target.name.equals(v2.name))
                {
        	    if (done==0)
        		e[i].weight = e[i].weight + lastEdgeWeight;
        	    else
        		e[i].weight = e[i].weight - lastEdgeWeight;
        	    break;
                }
            }
    	}
     

    // Here target means vertex which form edges with v1 (above) or v2 (below). It is a variable in the Edge class.

    	e = v2.adjacencies;
	for (i=0; i<e.length; i++)
    	{
            if (last==0)
            {
            	if (e[i].target.name.equals(v1.name))
            	{
        	    if (done==0)
        	    	e[i].weight = e[i].weight + nonLastEdgeWeight;
        	    else
        	    	e[i].weight = e[i].weight - nonLastEdgeWeight;
        	    break;
            	}
            }
            else
            {
            	if (done==0)
        	    e[i].weight = e[i].weight + lastEdgeWeight;
            	else
        	    e[i].weight = e[i].weight - lastEdgeWeight;
            }
    	}
    }
    
    
/**
* Transformation/mapping of the shortest path (in the form of List of vertices) to a set of commands in the form of F - move forward, L - turn left, R - turn right & U - take U-turn, which will be sent to the robot.
*/

    public String vertexToFLRUMapping(List<Vertex> path)
    {
    	String flru="";
    	
    	for(int i=1; i<path.size(); i++)
    	{
    	    char[] ID1 = {path.get(i-1).name.charAt(0),path.get(i-1).name.charAt(1)};
    	    char[] ID2 = {path.get(i).name.charAt(0),path.get(i).name.charAt(1)};
    		
    	    if (ID2[1] > ID1[1])
	    {
		if (direction[botSelected-1] == 'N')
		{
		    flru = flru + "F";
		}
		else if (direction[botSelected-1] == 'S')
		{
		    flru = flru + "UF";
		    direction[botSelected-1] = 'N';
		}
		else if (direction[botSelected-1] == 'W')
		{
		    flru = flru + "LF";
		    direction[botSelected-1] = 'N';
		}
		else if (direction[botSelected-1] == 'E')
		{
		    flru = flru + "RF";
		    direction[botSelected-1] = 'N';
		}
	    }
    	    else if (ID2[1] < ID1[1])
	    {
    		if (direction[botSelected-1] == 'S')
		{
		    flru = flru + "F";
		}
		else if (direction[botSelected-1] == 'N')
		{
		    flru = flru + "UF";
		    direction[botSelected-1] = 'S';
		}
		else if (direction[botSelected-1] == 'W')
		{
		    flru = flru + "RF";
		    direction[botSelected-1] = 'S';
		}
		else if (direction[botSelected-1] == 'E')
		{
		    flru = flru + "LF";
		    direction[botSelected-1] = 'S';
		}
   	    }
	    else
	    {
		if (ID2[1] == '4')
		{
		    if(ID2[0] > ID1[0])
		    {
			if(direction[botSelected-1] == 'N')
			{
			    flru = flru + "RF";
			    direction[botSelected-1] = 'W';
			}
			else
			{
			    flru = flru + "F";
			}
		    }
		    else
		    {
			if(direction[botSelected-1] == 'N')
			{
			    flru = flru + "LF";
			    direction[botSelected-1] = 'E';
			}
			else
			{
			    flru = flru + "F";
			}
		    }
		}
		else
		{
		    if(ID2[0] > ID1[0])
		    {
			if(direction[botSelected-1] == 'S')
			{
			    flru = flru + "LF";
			    direction[botSelected-1] = 'W';
			}
			else
			{
			    flru = flru + "F";
			}
		    }
		    else
		    {
			if(direction[botSelected-1] == 'S')
			{
			    flru = flru + "RF";
			    direction[botSelected-1] = 'E';
			}
			else
			{
			    flru = flru + "F";
			}
		    }
		}
	    }
    	}
    	
    	return flru;
    }


// Initialisation code for Serial Port Communication
    public void portReader() throws Exception
    {
	serPort = (SerialPort) pID.open("PortReader", 2000);
	inStream = serPort.getInputStream();
	outStream = serPort.getOutputStream();
	serPort.addEventListener(this);
	serPort.notifyOnDataAvailable(true);
	serPort.setSerialPortParams(9600, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
    }


// For reading data from the port once data arrives (typically used here for receiving ACK from the robot and releasing the edge weights and updating certain parameters like location, etc.)
    public void serialEvent(SerialPortEvent event)
    {
	switch (event.getEventType())
        {
	    case SerialPortEvent.BI: System.out.println("SerialPortEvent.BI occurred");
	    case SerialPortEvent.OE: System.out.println("SerialPortEvent.OE occurred");
	    case SerialPortEvent.FE: System.out.println("SerialPortEvent.FE occurred");
	    case SerialPortEvent.PE: System.out.println("SerialPortEvent.PE occurred");
	    case SerialPortEvent.CD: System.out.println("SerialPortEvent.CD occurred");
	    case SerialPortEvent.CTS: System.out.println("SerialPortEvent.CTS occurred");
	    case SerialPortEvent.DSR: System.out.println("SerialPortEvent.DSR occurred");
	    case SerialPortEvent.RI: System.out.println("SerialPortEvent.RI occurred");
	    case SerialPortEvent.OUTPUT_BUFFER_EMPTY: System.out.println("SerialPortEvent.OUTPUT_BUFFER_EMPTY occurred"); break;
	    case SerialPortEvent.DATA_AVAILABLE:
	        byte[] readBuffer = new byte[25];
	        try
		{
		    int numBytes = 0;
		    while (inStream.available() > 0)
		    {
		        numBytes = inStream.read(readBuffer);
		    	System.out.println("numBytes:"+numBytes);
		    }
		    String robotDone = (new String(readBuffer)).substring(0,numBytes);
		    botBusy[Integer.parseInt(robotDone)-1] = 0;
		    src.setSelectedItem(Arrays.asList(sensei).indexOf(""+initPos[Integer.parseInt(robotDone)-1]));
		    dest.setEnabled(true);
		    submit.setEnabled(true);
		    if (Integer.parseInt(robotDone) == 1)
		    	updatePathWeights(busyPath1, 1);
		    else
		      	updatePathWeights(busyPath2, 1);
		    	  
		    log.append(++action + ": Robot " + robotDone + " has reached " + initPos[Integer.parseInt(robotDone)-1] +". Work done.\n");
		    System.out.println("PortRead: "+(new String(readBuffer)).substring(0,numBytes));
		}
		catch (IOException ioe)
		{
		    System.out.println("Exception " + ioe);
		}
		break;
	}
    }          
	 

/**
* Constructor called from the main() function once the correct COM port is detected.
*/
    public BackgroundImageJframe()
    {

// Each edge is initially assigned a default weight = 1. A higher edge weight indicates that a path through that edge has been assigned to a particular robot.

	a0.adjacencies = new Edge[]{ new Edge(a1, 1), new Edge(b0, 1),};
	a1.adjacencies = new Edge[]{ new Edge(a0, 1), new Edge(a2, 1) };
	a2.adjacencies = new Edge[]{ new Edge(a1, 1), new Edge(a3, 1) };
	a3.adjacencies = new Edge[]{ new Edge(a2, 1), new Edge(a4, 1) };
	a4.adjacencies = new Edge[]{ new Edge(a3, 1), new Edge(b4, 1)};
		
	b0.adjacencies = new Edge[]{ new Edge(a0, 1), new Edge(c0, 1), new Edge(b1, 1)};
	b1.adjacencies = new Edge[]{ new Edge(b0, 1), new Edge(b2, 1) };
	b2.adjacencies = new Edge[]{ new Edge(b1, 1), new Edge(b3, 1) };
	b3.adjacencies = new Edge[]{ new Edge(b2, 1), new Edge(b4, 1) };
	b4.adjacencies = new Edge[]{ new Edge(b3, 1), new Edge(a4, 1), new Edge(c4, 1)};

	c0.adjacencies = new Edge[]{ new Edge(b0, 1), new Edge(d0, 1), new Edge(c1, 1)};
	c1.adjacencies = new Edge[]{ new Edge(c0, 1), new Edge(c2, 1) };
	c2.adjacencies = new Edge[]{ new Edge(c1, 1), new Edge(c3, 1) };
	c3.adjacencies = new Edge[]{ new Edge(c2, 1), new Edge(c4, 1) };
	c4.adjacencies = new Edge[]{ new Edge(c3, 1), new Edge(b4, 1), new Edge(d4, 1)};

	d0.adjacencies = new Edge[]{ new Edge(c0, 1), new Edge(e0, 1), new Edge(d1, 1)};
	d1.adjacencies = new Edge[]{ new Edge(d0, 1), new Edge(d2, 1) };
	d2.adjacencies = new Edge[]{ new Edge(d1, 1), new Edge(d3, 1) };
	d3.adjacencies = new Edge[]{ new Edge(d2, 1), new Edge(d4, 1) };
	d4.adjacencies = new Edge[]{ new Edge(d3, 1), new Edge(c4, 1), new Edge(e4, 1)};
		
	e0.adjacencies = new Edge[]{ new Edge(d0, 1), new Edge(e1, 1)};
	e1.adjacencies = new Edge[]{ new Edge(e0, 1), new Edge(e2, 1) };
	e2.adjacencies = new Edge[]{ new Edge(e1, 1), new Edge(e3, 1) };
	e3.adjacencies = new Edge[]{ new Edge(e2, 1), new Edge(e4, 1) };
	e4.adjacencies = new Edge[]{ new Edge(e3, 1), new Edge(d4, 1)};
		
	try
	{
	    portReader();
	}
	catch(Exception f)
	{
	    f.printStackTrace();
	}

/**
* Code for GUI Initialisation
*/		
	setTitle("Background Color for JFrame");
	//setSize(400,400);
	setLocationRelativeTo(null);
	setDefaultCloseOperation(EXIT_ON_CLOSE);
	setVisible(true);
	setLayout(new FlowLayout());
	ImageIcon ii = new ImageIcon(BackgroundImageJframe.class.getResource("GHModel1.png"));
	//ImageIcon ii = new ImageIcon("GHModel1.png");
	BufferedImage bi = new BufferedImage(740, 400, BufferedImage.TYPE_INT_RGB);
	Graphics2D g2d = (Graphics2D)bi.createGraphics();
        g2d.addRenderingHints(new RenderingHints(RenderingHints.KEY_RENDERING,RenderingHints.VALUE_RENDER_QUALITY));
        //boolean b = g2d.drawImage(ii.getImage(), 0, 0, 740, 400, null);
        //System.out.println(b);

	for(int i=1; i<=MAX_X*MAX_Y; i++)
        {
            sensei[i-1] = "" + i;
        }
        
        for(int i=1; i<=MAX_ROBOT; i++)
        {
            botStrings[i-1] = "" + i;
        }
                
	try
	{
	    ImageIO.write(bi, "png", new File("GHModel1.png"));
	}
	catch (Exception e)
	{
	    e.printStackTrace();
	}

	setContentPane(new JLabel(ii));
	setLayout(new FlowLayout(FlowLayout.CENTER, 15, 10));
		
	submit=new JButton("Submit");
	submit.addActionListener(this);
		
	l1 = new JLabel("Robot:");
	botList = new JComboBox(botStrings);
	botList.setSelectedIndex(0);
	botList.addActionListener(this);
		
	l2 = new JLabel("Source:");
	src = new JComboBox(sensei);
	src.setSelectedIndex(Arrays.asList(sensei).indexOf(""+initPos[0]));
	src.setEnabled(false);

	l3 = new JLabel("Destination:");
	dest = new JComboBox(sensei);
				
	log = new JTextArea("Robot Traversal Log:\n",6,75);
	log.setEditable(false);
	log.setFont(new Font("Arial", Font.BOLD, 16));
	log.setLineWrap(true);
	log.setWrapStyleWord(true);
	log.setCaretPosition(log.getDocument().getLength());
	scrollPane = new JScrollPane(log);
	scrollPane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		
	clear=new JButton("Clear Log");
	clear.addActionListener(this);
		
	add(l1);
	add(botList);
	add(l2);
	add(src);
	add(l3);
	add(dest);
	add(submit);
	add(scrollPane);
	add(clear);
		
	//setSize(1100,1100);
	setExtendedState(JFrame.MAXIMIZED_BOTH);
    }

/**
* Action that needs to be performed on (1) Selecting the robot, (2) Clear the log, (3) Clicking the Submit button for sending FLRU commands to the robot.
*/

    public void actionPerformed(final ActionEvent e)
    {
	if (e.getSource() == botList)
	{
	    String robot = (String)botList.getSelectedItem();
	    botSelected = Integer.parseInt(robot);
	    System.out.println(botSelected);
	    //System.out.println(Arrays.asList(sensei).indexOf(""+initPos[botSelected-1]));
	    int index = Arrays.asList(sensei).indexOf(""+initPos[botSelected-1]);
	    src.setSelectedIndex(index);
			
	    if (botBusy[botSelected-1] == 0)
	    {
		dest.setEnabled(true);
		submit.setEnabled(true);
	    }
	    else
	    {
		dest.setEnabled(false);
		submit.setEnabled(false);
	    }
	}
	else if (e.getSource() == clear)
	{
	    log.setText("Robot Traversal Log:\n");
	    action = 0;
	}
	else
	{
	    source = Integer.parseInt((String)src.getSelectedItem());
	    destination = Integer.parseInt((String)dest.getSelectedItem());
	    String robot = (String)botList.getSelectedItem();
	    botSelected = Integer.parseInt(robot);
		
	    computePaths(vertices[(((source-1)/3)*5) + ((source-1)%3) + 1]);
        
            Vertex v = vertices[(((destination-1)/3)*5) + ((destination-1)%3)+1];
            System.out.println("Distance to " + v + ": " + v.minDistance);
	    List<Vertex> path = getShortestPathTo(v);
	    System.out.println("Path: " + path);
	    
	    double pathwt = computePathWeight(path);
	    System.out.println("Path Weight:"+pathwt);
	    if (pathwt>lastEdgeWeight)
	    {
	    	log.append(++action + ": Possible Collision detected for Robot " + robot + ".\n");
	    	setPreviousNull();
	    }
	    else
	    {
	        //System.out.println("Path0: " + path.get(0).adjacencies[path.get(1).adjacencies].weight);
	    	setPreviousNull();
	        updatePathWeights(path,0);
	    	String pth = vertexToFLRUMapping(path);
	    
	    	System.out.println("FLRU:" + pth);
		
	    	if (botSelected == 1)
	    	    busyPath1 = path;
	    	else
	    	    busyPath2 = path;
	    
/**
* Creating a packet containing the necessary data that will be sent to the robot
*/
	    	pth = "   " + pth; // 3 spaces are reserved for header info of the packet
	    	pathc = pth.toCharArray();
	    	pathc[0] = (char)255; // indicates start of the packet
	    	pathc[1] = (char)botSelected; // Robot for which the packet is intended  
	    	pathc[2] = (char)(pth.length()-3); // Length of the commands to be executed.
		
	    	//System.out.println("pathc:"+pathc);
	    	System.out.println("(int)pathc[0]:"+(int)pathc[0]);
	    	System.out.println("(int)pathc[1]:"+(int)pathc[1]);
	    	System.out.println("(int)pathc[2]:"+(int)pathc[2]);
	    	System.out.println("pathc.length:"+pathc.length);
				
	    	//System.out.println(pth);
		
	    	int i=pathc.length;
	    	int j=0;
	    	while (j<i)
	    	{
		    try
		    {
		    	outStream.write(pathc[j]); // Sending commands byte-by-byte through the COM port
		    	Thread.sleep(500);
		    	j++;
		    }
		    catch(Exception ee)
		    {
		    	ee.printStackTrace();
		    }
	        }
	        botBusy[botSelected-1] = 1;
	        initPos[botSelected-1] = destination;
	        submit.setEnabled(false);
	        dest.setEnabled(false);
	        log.append(++action + ": Robot " + robot + " is moving from " + source + " to " + destination + " using " + pth.trim() + "....\n");
	    }
    	}
    }

/**
* Main function. It identifies the COM ports that it can detect and calls the constructor (i.e. the entire program will run) only if the COM port specified matches any of the detected ones.
*/
	
    public static void main(String args[]) throws Exception
    {
	ports = CommPortIdentifier.getPortIdentifiers(); 
	
	while (ports.hasMoreElements()) 
	{
	    pID = (CommPortIdentifier) ports.nextElement();
	    System.out.println("Port " + pID.getName());
	    if (pID.getPortType() == CommPortIdentifier.PORT_SERIAL) 
	    {
	    	if (pID.getName().equals("COM3")) // IMPORTANT: COM3 must be replaced by COMx depending on the COM port detected in your system.
	    	{
		    new BackgroundImageJframe();
		    System.out.println("COM3 found");
		}
	    }
	}
    }
}