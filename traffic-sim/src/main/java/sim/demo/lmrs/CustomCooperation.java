package sim.demo.lmrs;

import org.djunits.unit.AccelerationUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.gtu.perception.EgoPerception;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModel;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Desire;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsUtil;
import org.opentrafficsim.road.network.speed.SpeedLimitInfo;

import sim.demo.vehicleconfigurations.VehicleBehaviourTowardsOthers.LaneChangingBehavior;

/**
 * This is a copy from the LMRS Cooperation class, but with the addition of changes in 
 * the DCOOP parameter towards different vehicle types.
 */

/**
 * Copy from original class Cooperation.
 * This custom class is able to adjust cooperation behaviour based on the type of the leader GTU.
 * It utilises the adaptToLaneChangingVehicle() method from the TrafficInteractionAdaptations class.
 */
public interface CustomCooperation extends Cooperation
{

    /** Same as passive cooperation, except that cooperation is fully ignored if the potential lane changer brakes heavily. */
    Cooperation PASSIVE_MOVING = new Cooperation()
    {
        /** {@inheritDoc} */
        @Override
        public Acceleration cooperate(final LanePerception perception, final Parameters params, final SpeedLimitInfo sli,
                final CarFollowingModel cfm, final LateralDirectionality lat, final Desire ownDesire)
                throws ParameterException, OperationalPlanException
        {
        	if (!perception.getLaneStructure().exists(lat.isRight() ? RelativeLane.RIGHT : RelativeLane.LEFT))
            {
                return new Acceleration(Double.MAX_VALUE, AccelerationUnit.SI);
            }
            Acceleration bCrit = params.getParameter(ParameterTypes.BCRIT);
            Acceleration a = new Acceleration(Double.MAX_VALUE, AccelerationUnit.SI);
            double dCoop = params.getParameter(DCOOP);
            Speed ownSpeed = perception.getPerceptionCategory(EgoPerception.class).getSpeed();
            RelativeLane relativeLane = new RelativeLane(lat, 1);
            NeighborsPerception neighbours = perception.getPerceptionCategory(NeighborsPerception.class);
            PerceptionCollectable<HeadwayGtu, LaneBasedGtu> leaders = neighbours.getLeaders(RelativeLane.CURRENT);
            Speed thresholdSpeed = Speed.instantiateSI(6.86); // 295m / 43s
            boolean leaderInCongestion = leaders.isEmpty() ? false : leaders.first().getSpeed().lt(thresholdSpeed);
            for (HeadwayGtu leader : neighbours.getLeaders(relativeLane))
            {
            	Parameters params2 = leader.getParameters();
                double desire = lat.equals(LateralDirectionality.LEFT) ? params2.getParameter(DRIGHT)
                        : lat.equals(LateralDirectionality.RIGHT) ? params2.getParameter(DLEFT) : 0;
                // TODO: only cooperate if merger still quite fast or there's congestion downstream anyway (which we can better
                // estimate than only considering the direct leader
                
            	// add laneChangeBehaviour class to adjust vehicle interactions
            	LaneChangingBehavior laneChangeBehavior = null;
				try {
					laneChangeBehavior = new LaneChangingBehavior(perception.getGtu());
				} catch (OperationalPlanException e) {
					e.printStackTrace();
				} catch (ParameterException e) {
					e.printStackTrace();
				}
                
                // adjust DCOOP to leader type
                if (laneChangeBehavior != null) {
                	double adjustment = laneChangeBehavior.adaptToLaneChangingVehicle(leader);
                	if (desire >= (dCoop + adjustment) && (leader.getSpeed().gt0() || leader.getDistance().gt0())
                            && (leader.getSpeed().ge(thresholdSpeed) || leaderInCongestion))
                    {
                        Acceleration aSingle = LmrsUtil.singleAcceleration(leader.getDistance(), ownSpeed, leader.getSpeed(),
                                desire, params, sli, cfm);
                        a = Acceleration.min(a, aSingle);
                    }
                }
                
                // behaviour class not created? perform normal logic
                else if (desire >= dCoop && (leader.getSpeed().gt0() || leader.getDistance().gt0())
                        && (leader.getSpeed().ge(thresholdSpeed) || leaderInCongestion))
                {
                    Acceleration aSingle = LmrsUtil.singleAcceleration(leader.getDistance(), ownSpeed, leader.getSpeed(),
                            desire, params, sli, cfm);
                    a = Acceleration.min(a, aSingle);
                }
            }
            return Acceleration.max(a, bCrit.neg());
        }

        /** {@inheritDoc} */
        @Override
        public String toString()
        {
            return "PASSIVE_MOVING";
        }
    };


    /**
     * Determine acceleration for cooperation.
     * @param perception LanePerception; perception
     * @param params Parameters; parameters
     * @param sli SpeedLimitInfo; speed limit info
     * @param cfm CarFollowingModel; car-following model
     * @param lat LateralDirectionality; lateral direction for cooperation
     * @param ownDesire Desire; own lane change desire
     * @return acceleration for synchronization
     * @throws ParameterException if a parameter is not defined
     * @throws OperationalPlanException perception exception
     */
    Acceleration cooperate(LanePerception perception, Parameters params, SpeedLimitInfo sli, CarFollowingModel cfm,
            LateralDirectionality lat, Desire ownDesire) throws ParameterException, OperationalPlanException;
}

