package sim.demo.lmrs;

import org.djunits.unit.AccelerationUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.gtu.GtuException;
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
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.network.speed.SpeedLimitInfo;

import sim.demo.behavior.TrafficInteractionAdaptations.LaneChangingBehavior;

/**
 * This is a copy from the LMRS Cooperation class, but with the addition of changes in 
 * the DCOOP parameter towards different vehicle types.
 */

/**
 * Different forms of cooperation.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public interface CustomCooperation extends Cooperation
{

    /** Simple passive cooperation. */
    CustomCooperation PASSIVE = new CustomCooperation()
    {
        /** {@inheritDoc} */
        @Override
        public Acceleration cooperate(final LanePerception perception, final Parameters params, final SpeedLimitInfo sli,
                final CarFollowingModel cfm, final LateralDirectionality lat, final Desire ownDesire)
                throws ParameterException, OperationalPlanException
        {
            if ((lat.isLeft() && !perception.getLaneStructure().getExtendedCrossSection().contains(RelativeLane.LEFT))
                    || (lat.isRight() && !perception.getLaneStructure().getExtendedCrossSection().contains(RelativeLane.RIGHT)))
            {
                return new Acceleration(Double.MAX_VALUE, AccelerationUnit.SI);
            }
            Acceleration b = params.getParameter(ParameterTypes.B);
            Acceleration a = new Acceleration(Double.MAX_VALUE, AccelerationUnit.SI);
            double dCoop = params.getParameter(DCOOP);
            Speed ownSpeed = perception.getPerceptionCategory(EgoPerception.class).getSpeed();
            RelativeLane relativeLane = new RelativeLane(lat, 1);
            for (HeadwayGtu leader : perception.getPerceptionCategory(NeighborsPerception.class).getLeaders(relativeLane))
            {
            	// adjust DCOOP parameter because of leader (potentially changing lanes) vehicle type
            	LaneChangingBehavior laneChangeBehavior = null;
				try {
					laneChangeBehavior = new LaneChangingBehavior(perception.getGtu());
				} catch (OperationalPlanException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (ParameterException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (GtuException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
            	
                Parameters params2 = leader.getParameters();
                double desire = lat.equals(LateralDirectionality.LEFT) ? params2.getParameter(DRIGHT)
                        : lat.equals(LateralDirectionality.RIGHT) ? params2.getParameter(DLEFT) : 0;
                // adjust DCOOP to leader
                if (laneChangeBehavior != null) {
                	double adjustment = laneChangeBehavior.adaptToLaneChangingVehicle(leader);
                	if (desire >= (dCoop + adjustment) && (leader.getSpeed().gt0() || leader.getDistance().gt0()))
                    {
                        Acceleration aSingle = LmrsUtil.singleAcceleration(leader.getDistance(), ownSpeed, leader.getSpeed(),
                                desire, params, sli, cfm);
                        a = Acceleration.min(a, aSingle);
                    }
                }
                // behaviour not created? perform normal logic
                else if (desire >= dCoop && (leader.getSpeed().gt0() || leader.getDistance().gt0()))
                {
                    Acceleration aSingle = LmrsUtil.singleAcceleration(leader.getDistance(), ownSpeed, leader.getSpeed(),
                            desire, params, sli, cfm);
                    a = Acceleration.min(a, aSingle);
                }
            }
            return Acceleration.max(a, b.neg());
        }
        
        /** {@inheritDoc} */
        @Override
        public String toString()
        {
            return "PASSIVE";
        }
    };

    /** Same as passive cooperation, except that cooperation is fully ignored if the potential lane changer brakes heavily. */
    CustomCooperation PASSIVE_MOVING = new CustomCooperation()
    {
        /** {@inheritDoc} */
        @Override
        public Acceleration cooperate(final LanePerception perception, final Parameters params, final SpeedLimitInfo sli,
                final CarFollowingModel cfm, final LateralDirectionality lat, final Desire ownDesire)
                throws ParameterException, OperationalPlanException
        {
            if ((lat.isLeft() && !perception.getLaneStructure().getExtendedCrossSection().contains(RelativeLane.LEFT))
                    || (lat.isRight() && !perception.getLaneStructure().getExtendedCrossSection().contains(RelativeLane.RIGHT)))
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
            Speed thresholdSpeed = Speed.instantiateSI(40.0 / 3.6);
            boolean leaderInCongestion = leaders.isEmpty() ? false : leaders.first().getSpeed().lt(thresholdSpeed);
            for (HeadwayGtu leader : neighbours.getLeaders(relativeLane))
            {
            	// create behaviour with current leader (potentially changing lanes)
            	LaneChangingBehavior laneChangeBehavior = null;
				try {
					laneChangeBehavior = new LaneChangingBehavior(perception.getGtu());
				} catch (OperationalPlanException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (ParameterException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (GtuException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
                Parameters params2 = leader.getParameters();
                double desire = lat.equals(LateralDirectionality.LEFT) ? params2.getParameter(DRIGHT)
                        : lat.equals(LateralDirectionality.RIGHT) ? params2.getParameter(DLEFT) : 0;
                // TODO: only cooperate if merger still quite fast or there's congestion downstream anyway (which we can better
                // estimate than only considering the direct leader
                
                // adjust DCOOP to leader
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
                // behaviour not created? perform normal logic
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

    /** Cooperation similar to the default, with nuanced differences of when to ignore. */
    CustomCooperation ACTIVE = new CustomCooperation()
    {
        /** {@inheritDoc} */
        @Override
        public Acceleration cooperate(final LanePerception perception, final Parameters params, final SpeedLimitInfo sli,
                final CarFollowingModel cfm, final LateralDirectionality lat, final Desire ownDesire)
                throws ParameterException, OperationalPlanException
        {

            if ((lat.isLeft() && !perception.getLaneStructure().getExtendedCrossSection().contains(RelativeLane.LEFT))
                    || (lat.isRight() && !perception.getLaneStructure().getExtendedCrossSection().contains(RelativeLane.RIGHT)))
            {
                return new Acceleration(Double.MAX_VALUE, AccelerationUnit.SI);
            }
            Acceleration a = new Acceleration(Double.MAX_VALUE, AccelerationUnit.SI);
            double dCoop = params.getParameter(DCOOP);
            Speed ownSpeed = perception.getPerceptionCategory(EgoPerception.class).getSpeed();
            RelativeLane relativeLane = new RelativeLane(lat, 1);
            for (HeadwayGtu leader : perception.getPerceptionCategory(NeighborsPerception.class).getLeaders(relativeLane))
            {
            	// create behaviour with current leader (potentially changing lanes)
            	LaneChangingBehavior laneChangeBehavior = null;
				try {
					laneChangeBehavior = new LaneChangingBehavior(perception.getGtu());
				} catch (OperationalPlanException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (ParameterException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (GtuException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
                Parameters params2 = leader.getParameters();
                double desire = lat.equals(LateralDirectionality.LEFT) ? params2.getParameter(DRIGHT)
                        : lat.equals(LateralDirectionality.RIGHT) ? params2.getParameter(DLEFT) : 0;
                // adjust DCOOP to leader
                if (laneChangeBehavior != null) {
                	double adjustment = laneChangeBehavior.adaptToLaneChangingVehicle(leader);
                	if (desire >= (dCoop + adjustment) && leader.getDistance().gt0()
                            && leader.getAcceleration().gt(params.getParameter(ParameterTypes.BCRIT).neg()))
                    {
                        Acceleration aSingle = LmrsUtil.singleAcceleration(leader.getDistance(), ownSpeed, leader.getSpeed(),
                                desire, params, sli, cfm);
                        a = Acceleration.min(a, Synchronization.gentleUrgency(aSingle, desire, params));
                    }
                }
                // behaviour not created? perform normal logic
                else if (desire >= dCoop && leader.getDistance().gt0()
                        && leader.getAcceleration().gt(params.getParameter(ParameterTypes.BCRIT).neg()))
                {
                    Acceleration aSingle = LmrsUtil.singleAcceleration(leader.getDistance(), ownSpeed, leader.getSpeed(),
                            desire, params, sli, cfm);
                    a = Acceleration.min(a, Synchronization.gentleUrgency(aSingle, desire, params));
                }
            }
            return a;
        }
        
        /** {@inheritDoc} */
        @Override
        public String toString()
        {
            return "ACTIVE";
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
