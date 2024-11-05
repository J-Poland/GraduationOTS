package sim.demo.mental;

import java.util.LinkedHashSet;
import java.util.Set;

import org.djutils.exceptions.Throw;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypeDouble;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.base.parameters.constraint.DualBound;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.Task;
import org.opentrafficsim.road.gtu.lane.perception.mental.TaskManager;

/**
 * Task manager implementation of anticipation reliance.
 */
public class TaskManagerAr implements TaskManager
{
    /** Fraction of primary task that can be reduced by anticipation reliance. */
    public static final ParameterTypeDouble ALPHA = new ParameterTypeDouble("alpha",
            "Fraction of primary task that can be reduced by anticipation reliance.", 0.8, DualBound.UNITINTERVAL);

    /** Fraction of auxiliary tasks that can be reduced by anticipation reliance. */
    public static final ParameterTypeDouble BETA = new ParameterTypeDouble("beta",
            "Fraction of auxiliary tasks that can be reduced by anticipation reliance.", 0.6, DualBound.UNITINTERVAL);

    /** Primary task id. */
    private final String primaryTaskId;

    /**
     * Constructor.
     * @param primaryTaskId String; primary task id.
     */
    public TaskManagerAr(final String primaryTaskId)
    {
        Throw.whenNull(primaryTaskId, "Primary task id may not be null.");
        this.primaryTaskId = primaryTaskId;
    }

    /** {@inheritDoc} */
    @Override
    public void manage(final Set<Task> tasks, final LanePerception perception, final LaneBasedGtu gtu,
            final Parameters parameters) throws ParameterException, GtuException
    {
        Task primary = null;
        TaskDriverDistraction distractionTask = null;
        Set<Task> auxiliaryTasks = new LinkedHashSet<>();
        for (Task task : tasks)
        {
            if (task.getId().equals(this.primaryTaskId))
            {
                primary = task;
            }
            else if (task.getId().equals("driver distraction")) {
            	distractionTask = (TaskDriverDistraction) task;
            }
            else
            {
                auxiliaryTasks.add(task);
            }
        }
        Throw.whenNull(primary, "There is no task with id '%s'.", this.primaryTaskId);
        
        // max AR is alpha of TD, actual AR approaches 0 for increasing TD
        double a = parameters.getParameter(ALPHA);
        double b = parameters.getParameter(BETA);
        
        // set demand for primary driving task
        double primaryTaskDemand = primary.calculateTaskDemand(perception, gtu, parameters);
        primary.setTaskDemand(primaryTaskDemand);
        double primaryTaskAR = (a * primaryTaskDemand * (1.0 - primaryTaskDemand));
        primary.setAnticipationReliance(primaryTaskAR);
        
        // store sum of primary task demand and anticipation reliance
        double primaryRemainingTaskDemand = primaryTaskDemand - primaryTaskAR;
        
        // set demand for auxiliary tasks
        double auxiliaryRemainingTaskDemand = 0;
        for (Task auxiliary : auxiliaryTasks)
        {
            double auxiliaryTaskDemand = auxiliary.calculateTaskDemand(perception, gtu, parameters);
            auxiliary.setTaskDemand(auxiliaryTaskDemand);
            // max AR is beta of TD, actual AR approaches 0 as primary TD approaches 0
            double auxiliaryTaskAR = (b * auxiliaryTaskDemand * primaryTaskDemand);
            auxiliary.setAnticipationReliance(auxiliaryTaskAR);
            
            // add to sum of auxiliary task demand
            auxiliaryRemainingTaskDemand += auxiliaryTaskDemand;
            auxiliaryRemainingTaskDemand -= auxiliaryTaskAR;
        }
        
    	// get distraction status for each distraction
    	boolean inVehicleDistractionState = distractionTask.getInVehicleDistractionState(perception, gtu, parameters);
    	boolean roadSideDistractionState = distractionTask.getRoadSideDistractionState(perception, gtu, parameters);
        
        // set distraction demand
        double distractionDemand = 0;
        if (inVehicleDistractionState || roadSideDistractionState) {
        	// get remaining mental capacity till the critical task saturation
            double criticalTS = gtu.getParameters().getParameterOrNull(Fuller.TS_CRIT);
            double remainingTS = Math.max(0, criticalTS - (primaryRemainingTaskDemand + auxiliaryRemainingTaskDemand));
            // dynamically calculate demand of distraction (remaining non-deteriation task capacity + 10%, non-negative)
            distractionDemand = remainingTS * 1.1;
        }
        
        // set task demand and anticipation reliance for distraction
        distractionTask.setTaskDemand(distractionDemand);
        distractionTask.setAnticipationReliance(0.0);
    }
}
