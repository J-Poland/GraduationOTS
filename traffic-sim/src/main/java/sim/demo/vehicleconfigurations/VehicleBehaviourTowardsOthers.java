package sim.demo.vehicleconfigurations;

import org.djunits.value.vdouble.scalar.Duration;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;

import nl.tudelft.simulation.jstats.distributions.DistTriangular;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;


/**
 * This class contains classes to adjust driving behaviour due to surrounding traffic. The traffic intensity 
 * and the kind of traffic around the driver do impact its behaviour. These impacts are defined here.
 */
public class VehicleBehaviourTowardsOthers {
	
	// method to draw a random value from distribution
	private static double drawFromDistribution(StreamInterface stream, double min, double mode, double max)
	{
		DistTriangular valueDist = new DistTriangular(stream, min, mode, max);
        double value = valueDist.draw();
        return value;
	}
	
	
	// class to change car-following interactions between vehicle types
    public static class CarFollowingBehavior
    {
    	private long uniqueSeed;
    	private LaneBasedGtu gtu;
    	private StreamInterface stream;
    	
    	public CarFollowingBehavior(LaneBasedGtu gtu) throws OperationalPlanException, ParameterException
    	{
    		// get gtu
    		this.gtu = gtu;
    		
    		// get simulator info
    		Duration simTime = gtu.getSimulator().getSimulatorTime();
    		StreamInterface simStream = gtu.getSimulator().getModel().getDefaultStream();
    		
    		// set unique seed (by simulation stream, simulation time, and vehicle id)
    		// this is necessary to draw different values for different GTUs and different times
    		// but keep reproducibility of the simulation
    		uniqueSeed = simStream.nextLong() + Double.doubleToLongBits(simTime.si) + Long.parseLong(gtu.getId());
    		StreamInterface stream = new MersenneTwister(Math.abs(uniqueSeed));
    		this.stream = stream;
    	}
    	
    	public void adaptToVehicleInFront() throws OperationalPlanException, ParameterException
    	{
    		// get vehicle in front
    		NeighborsPerception neighbors = gtu.getTacticalPlanner().getPerception().getPerceptionCategory(NeighborsPerception.class);
            PerceptionCollectable<HeadwayGtu, LaneBasedGtu> leaders = neighbors.getLeaders(RelativeLane.CURRENT);
            // only continue if leaders are present
            if (leaders.isEmpty()) {
            	return;
            }
            // get own gtu type
            String thisType = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
            // get first leader
            HeadwayGtu gtuLeader = leaders.first();
            String leaderType = gtuLeader.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
            // change behaviour if vehicle in front is of type Level-3 and this gtu is Level-0
            // example of gtu type string: "GtuType: NL.LEVEL3CAR"
            if (thisType.contains("LEVEL0") && leaderType.contains("LEVEL3")) {
            	// lower headway
            	//TODO: values for lower TMIN values!!!
            	double adjustedT = gtu.getParameters().getParameter(VehicleAutomationConfigurations.INITIAL_TMIN).si - drawFromDistribution(stream, 0.0, 0.15, 0.3);
            	gtu.getParameters().setParameter(ParameterTypes.TMIN, Duration.instantiateSI(adjustedT));
            }
            // keep initial headway value whenever the leader type is not level-3
            else {
            	double initial_value = gtu.getParameters().getParameter(VehicleAutomationConfigurations.INITIAL_TMIN).si;
            	gtu.getParameters().setParameter(ParameterTypes.TMIN, Duration.instantiateSI(initial_value));
            }
    	}
    }
    
    
    // class to change lane-change interactions between vehicle types
    public static class LaneChangingBehavior
    {
    	private long uniqueSeed;
    	private LaneBasedGtu gtu;
    	private StreamInterface stream;
    	
    	public LaneChangingBehavior(LaneBasedGtu gtu) throws OperationalPlanException, ParameterException
    	{	
    		// get gtu
    		this.gtu = gtu;
    		
    		// get simulator info
    		Duration simTime = gtu.getSimulator().getSimulatorTime();
    		StreamInterface simStream = gtu.getSimulator().getModel().getDefaultStream();
    		
    		// set unique seed (by simulation stream, simulation time, and vehicle id)
    		// this is necessary to draw different values for different GTUs and different times
    		// but keep reproducibility of the simulation
    		uniqueSeed = simStream.nextLong() + Double.doubleToLongBits(simTime.si) + Long.parseLong(gtu.getId());
    		StreamInterface stream = new MersenneTwister(Math.abs(uniqueSeed));
    		this.stream = stream;
    	}
    	
    	public Double adaptToLaneChangingVehicle(HeadwayGtu leaderGtu) throws ParameterException
    	{
            // get own gtu type string
            String thisType = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
    		// create string from leader vehicle type
    		String leaderType = leaderGtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
            // change behaviour parameter value if other vehicle is of type level-3 and this gtu is level-0
            // example of gtu type string: "GtuType: NL.LEVEL3CAR"
            if (thisType.contains("LEVEL0") && leaderType.contains("LEVEL3")) {
            	return drawFromDistribution(stream, 0.0, 0.05, 0.1);
            }
            // not found? return 0.0, no influence from traffic
            else {
            	return 0.0;
            }
    	}
    }
}
