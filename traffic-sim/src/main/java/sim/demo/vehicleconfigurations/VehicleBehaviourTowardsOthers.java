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
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;

import nl.tudelft.simulation.jstats.distributions.DistTriangular;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;


/**
 * This class contains classes to adjust driving behaviour due to surrounding traffic. The traffic intensity 
 * and the kind of traffic around the driver do impact its behaviour. These impacts are defined here.
 */
public class VehicleBehaviourTowardsOthers {
	
	// class to change lane change interactions between vehicle types
    public static class LaneChangingBehavior
    {
    	private LaneBasedGtu gtu;
    	
    	public LaneChangingBehavior(LaneBasedGtu gtu) throws OperationalPlanException, ParameterException
    	{	
    		// get gtu
    		this.gtu = gtu;
    	}
    	
    	public Double adaptToLaneChangingVehicle(double dCoop, HeadwayGtu leaderGtu) throws ParameterException
    	{
    		// set max lane change desire
    		double dCoopMin = dCoop * 0.9;
    		double dCoopMax = dCoop * 1.1;
            // get own gtu type string
            String thisType = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
    		// create string from leader vehicle type
    		String leaderType = leaderGtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
            // change behaviour parameter value if other GTU is of type level-3 and this GTU is level-0
            // example of GTU type string: "GtuType: NL.LEVEL3CAR"
            if (thisType.contains("LEVEL0") && leaderType.contains("LEVEL3")) {
            	double social_cf = gtu.getParameters().getParameter(VehicleAutomationConfigurations.SOCIO_CF);
            	return dCoopMin + (dCoopMax - dCoopMin) * (1 - social_cf);
            }
            // interaction between other vehicles? return 0.0
            else {
            	return dCoop;
            }
    	}
    }
	
    
	// changes in GTU headway due to surrounding vehicle types is performed in the CustomAdaptationHeadway class.
    
//	// class to change car-following interactions between vehicle types
//    public static class CarFollowingBehavior
//    {
//    	private LaneBasedGtu gtu;
//    	
//    	public CarFollowingBehavior(LaneBasedGtu gtu) throws OperationalPlanException, ParameterException
//    	{
//    		// get gtu
//    		this.gtu = gtu;
//    	}
//    	
//    	public Double adaptHeadwayToSurroundings() throws OperationalPlanException, ParameterException
//    	{
//    		return 0.0;
//    	}
//    }
    
}
