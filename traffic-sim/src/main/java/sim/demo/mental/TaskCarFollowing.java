package sim.demo.mental;

import org.djunits.value.vdouble.scalar.Duration;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDuration;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.base.parameters.constraint.NumericConstraint;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.TaskHeadwayCollector;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.AbstractTask;

import sim.demo.VehicleConfigurations;

public class TaskCarFollowing extends AbstractTask
{

    /** Car-following task parameter. */
    public static final ParameterTypeDuration HEXP = new ParameterTypeDuration("Hexp",
            "Exponential decay of car-following task by headway.", Duration.instantiateSI(4.0), NumericConstraint.POSITIVE);

    /**
     * Constructor.
     */
    public TaskCarFollowing()
    {
        super("car-following");
    }

    /** {@inheritDoc} */
    @Override
    public double calculateTaskDemand(final LanePerception perception, final LaneBasedGtu gtu, final Parameters parameters)
            throws ParameterException, GtuException
    {
    	// only add task demand for non automated car-following (automation level 0)
    	String gtuType = gtu.getParameters().getParameterOrNull(VehicleConfigurations.AUTOMATION_LEVEL);
    	if (gtuType != null && gtuType.contains("LEVEL0")) {
	        try
	        {
	            NeighborsPerception neighbors = perception.getPerceptionCategory(NeighborsPerception.class);
	            PerceptionCollectable<HeadwayGtu, LaneBasedGtu> leaders = neighbors.getLeaders(RelativeLane.CURRENT);
	            Duration headway = leaders.collect(new TaskHeadwayCollector(gtu.getSpeed()));
	            double demand = headway == null ? 0.0 : Math.exp(-headway.si / parameters.getParameter(HEXP).si);
	            
	            // update GTU parameter
	            gtu.getParameters().setParameter(VehicleConfigurations.CF_TASK_DEMAND, demand);
	            
	            // return demand
	            return demand;
	        }
	        catch (OperationalPlanException ex)
	        {
	            throw new GtuException(ex);
	        }
    	} else {
    		double demand = 0.0;
    		
    		// update GTU parameter
            gtu.getParameters().setParameter(VehicleConfigurations.CF_TASK_DEMAND, demand);
            
            // return demand
            return demand;
    	}
    }
}