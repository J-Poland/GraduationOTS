package sim.demo;

import java.awt.Dimension;
import java.net.URL;
import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import javax.naming.NamingException;

import org.djunits.unit.AccelerationUnit;
import org.djunits.unit.DurationUnit;
import org.djunits.unit.FrequencyUnit;
import org.djunits.unit.LengthUnit;
import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Frequency;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djutils.event.Event;
import org.djutils.event.EventListener;
import org.djutils.io.URLResource;
import org.opentrafficsim.animation.colorer.LmrsSwitchableColorer;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterSet;
import org.opentrafficsim.base.parameters.ParameterTypeString;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.distributions.ConstantGenerator;
import org.opentrafficsim.core.distributions.Distribution;
import org.opentrafficsim.core.distributions.Distribution.FrequencyAndObject;
import org.opentrafficsim.core.distributions.Generator;
import org.opentrafficsim.core.distributions.ProbabilityException;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.perception.DirectEgoPerception;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.idgenerator.IdGenerator;
import org.opentrafficsim.core.network.LateralDirectionality;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.network.route.ProbabilisticRouteGenerator;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.core.parameters.ParameterFactory;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.core.units.distributions.ContinuousDistDoubleScalar;
import org.opentrafficsim.draw.OtsDrawingException;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator.RoomChecker;
import org.opentrafficsim.road.gtu.generator.TtcRoomChecker;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplate;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplateDistribution;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.CategoricalLanePerception;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionFactory;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.categories.AnticipationTrafficPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.DirectInfrastructurePerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.DirectIntersectionPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Anticipation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.DirectNeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Estimation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType.PerceivedHeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSituationalAwareness;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.Task;
import org.opentrafficsim.road.gtu.lane.perception.mental.TaskManager;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller.BehavioralAdaptation;
import org.opentrafficsim.road.gtu.lane.tactical.LaneBasedTacticalPlannerFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModelFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlus;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlusFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationConflicts;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationSpeedLimitTransition;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationTrafficLights;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.DefaultLmrsPerceptionFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveKeep;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveRoute;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSocioSpeed;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSpeedWithCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.Lmrs;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.LmrsFactory;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.GapAcceptance;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.LmrsParameters;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.MandatoryIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Synchronization;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Tailgating;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.VoluntaryIncentive;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LanePosition;
import org.opentrafficsim.road.network.lane.object.SpeedSign;
import org.opentrafficsim.swing.gui.AnimationToggles;
import org.opentrafficsim.swing.gui.OtsAnimationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.simulators.SimulatorInterface;
import nl.tudelft.simulation.jstats.distributions.DistNormal;
import nl.tudelft.simulation.jstats.distributions.DistUniform;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import nl.tudelft.simulation.language.DsolException;
import sim.demo.gtu.CustomLaneBasedGtuTemplate;
import sim.demo.gtu.CustomLaneBasedGtuTemplateDistribution;
import sim.demo.mental.TaskCarFollowing;
import sim.demo.mental.CustomAdaptationHeadway;
import sim.demo.mental.CustomAdaptationSituationalAwareness;
import sim.demo.mental.TaskLaneChange;
import sim.demo.mental.TaskManagerAr;

/**
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class MyShortMerge extends OtsSimulationApplication<sim.demo.MyShortMerge.MyShortMergeModel>
{
    /** */
    private static final long serialVersionUID = 20170407L;

    /** Network. */
    static final String NETWORK = "shortMerge";

    /** Truck fraction. */
    static final double TRUCK_FRACTION = 0.15;

    /** Left traffic fraction. */
    static final double LEFT_FRACTION = 0.3;

    /** Main demand per lane. */
    static final Frequency MAIN_DEMAND = new Frequency(6000, FrequencyUnit.PER_HOUR);

    /** Ramp demand. */
    static final Frequency RAMP_DEMAND = new Frequency(2000, FrequencyUnit.PER_HOUR);

    /** Synchronization. */
    static final Synchronization SYNCHRONIZATION = Synchronization.ALIGN_GAP;

    /** Cooperation. */
    static final Cooperation COOPERATION = Cooperation.PASSIVE_MOVING;

    /** Use additional incentives. */
    static final boolean ADDITIONAL_INCENTIVES = true;

    /** Simulation time. */
    public static final Time SIMTIME = Time.instantiateSI(3600);
    
    public static double maxAcceleration = 3.0;
    public static double maxDeceleration = -8.0;
    
    public static final ParameterTypeString VEHICLE_TYPE = new ParameterTypeString("VEHICLE_TYPE", "Vehicle type.");

    /**
     * Create a ShortMerge Swing application.
     * @param title String; the title of the Frame
     * @param panel OtsAnimationPanel; the tabbed panel to display
     * @param model ShortMergeModel; the model
     * @throws OtsDrawingException on animation error
     */
    public MyShortMerge(final String title, final OtsAnimationPanel panel, final MyShortMergeModel model) throws OtsDrawingException
    {
        super(model, panel);
    }

    /** {@inheritDoc} */
    @Override
    protected void setAnimationToggles()
    {
        AnimationToggles.setTextAnimationTogglesFull(getAnimationPanel());
        getAnimationPanel().getAnimationPanel().toggleClass(Link.class);
        getAnimationPanel().getAnimationPanel().toggleClass(Node.class);
        getAnimationPanel().getAnimationPanel().showClass(SpeedSign.class);
    }

    /**
     * Main program.
     * @param args String[]; the command line arguments (not used)
     */
    public static void main(final String[] args)
    {
        demo(true);
    }

    /**
     * Start the demo.
     * @param exitOnClose boolean; when running stand-alone: true; when running as part of a demo: false
     */
    public static void demo(final boolean exitOnClose)
    {
        try
        {
            OtsAnimator simulator = new OtsAnimator("ShortMerge");
            final MyShortMergeModel otsModel = new MyShortMergeModel(simulator);
            simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(3600.0), otsModel);
            OtsAnimationPanel animationPanel = new OtsAnimationPanel(otsModel.getNetwork().getExtent(), new Dimension(800, 600),
                    simulator, otsModel, new LmrsSwitchableColorer(DefaultsNl.GTU_TYPE_COLORS.toMap()), otsModel.getNetwork());
            MyShortMerge app = new MyShortMerge("ShortMerge", animationPanel, otsModel);
            app.setExitOnClose(exitOnClose);
            animationPanel.enableSimulationControlButtons();
        }
        catch (SimRuntimeException | NamingException | RemoteException | OtsDrawingException | IndexOutOfBoundsException
                | DsolException exception)
        {
            exception.printStackTrace();
        }
    }

    /**
     * <p>
     * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.
     * <br>
     * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
     * </p>
     * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
     * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
     * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
     */
    public static class MyShortMergeModel extends AbstractOtsModel implements EventListener
    {
        /** */
        private static final long serialVersionUID = 20170407L;

        /** The network. */
        private RoadNetwork network;

        /**
         * @param simulator OtsSimulatorInterface; the simulator
         */
        public MyShortMergeModel(final OtsSimulatorInterface simulator)
        {
        	// parse simulator
    		super(simulator);
    		
    		this.simulator = simulator;
        }

        /**
         * @param network RoadNetwork; set network.
         */
        public void setNetwork(final RoadNetwork network)
        {
            this.network = network;
        }

        /** {@inheritDoc} */
        @Override
        public void constructModel() throws SimRuntimeException
        {
            try
            {
                URL xmlURL = URLResource.getResource("C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\traffic-sim\\src\\main\\"
                									 + "\\resources\\networks\\" + NETWORK + ".xml");
                this.network = new RoadNetwork("ShortMerge", getSimulator());
                new XmlParser(this.network).setUrl(xmlURL).build();
                addGenerator();
                addSubscriptions();
            }
            catch (Exception exception)
            {
                exception.printStackTrace();
            }
        }

        /** {@inheritDoc} */
        @Override
        public RoadNetwork getNetwork()
        {
            return this.network;
        }
        
        /** Method to add listeners to simulation events. 
    	 * @throws RemoteException */
    	private void addSubscriptions() throws RemoteException {
    		// simulator events
    		this.simulator.addListener(this, SimulatorInterface.START_EVENT);
    		this.simulator.addListener(this, SimulatorInterface.STOP_EVENT);
    		this.simulator.addListener(this, SimulatorInterface.TIME_CHANGED_EVENT);
    		
    		// traffic events
    		this.network.addListener(this, Network.GTU_ADD_EVENT);
    		this.network.addListener(this, Network.GTU_REMOVE_EVENT);
    	}

    	/** Method to track and process notifications from the simulation listener. */
    	@Override
    	public void notify(Event event) throws RemoteException {

    		// handle start of replication event
    		if (event.getType().equals(SimulatorInterface.START_EVENT)) {
    			System.out.println("Start simulation.");
    		}
    		
    		// handle end of replication event
    		if (event.getType().equals(SimulatorInterface.STOP_EVENT)) {
    			System.out.println("Stop simulation.");
    		}
    		
    		// handle simulation time changed event
    		if (event.getType().equals(SimulatorInterface.TIME_CHANGED_EVENT)) {
    			
    		}
    		
    		// subscribe to GTU lane events when GTUs are added to the simulation
    		if (event.getType().equals(Network.GTU_ADD_EVENT)) {
    			String gtuId = (String) event.getContent();
    			LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
    			
    			// add GTU listeners
    			gtu.addListener(this, Gtu.MOVE_EVENT);
    			gtu.addListener(this, LaneBasedGtu.LANE_CHANGE_EVENT);
    			gtu.addListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
    			
    		}
    		
    		// disable subscription to GTU lane events when GTUs leave the simulation
    		if (event.getType().equals(Network.GTU_REMOVE_EVENT)) {
    			String gtuId = (String) event.getContent();
    			LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
    			
    			// remove GTU listeners
    			gtu.removeListener(this, Gtu.MOVE_EVENT);
    			gtu.removeListener(this, LaneBasedGtu.LANE_CHANGE_EVENT);
    		}
    		
    		// handle GTU move events
    		if (event.getType().equals(Gtu.MOVE_EVENT)) {
    			String gtuId = (String) ((Object[]) event.getContent())[0];
    	        LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
    	        
//    	        if (gtu.getType().toString().contains("TRUCK")) {
//    	        	System.out.println(gtu.getType().toString() + ": savedType = " + gtu.getParameters().getParameterOrNull(VEHICLE_TYPE));
//    	        	System.out.println("Tmin=" + gtu.getParameters().getParameterOrNull(ParameterTypes.TMIN) + " - Tmax=" + gtu.getParameters().getParameterOrNull(ParameterTypes.TMAX));
//    	        }
    	    }
    		
    		if (event.getType().equals(LaneBasedGtu.LANEBASED_MOVE_EVENT)) {

    		}
    		
    		// process change event
    		if (event.getType().equals(LaneBasedGtu.LANE_CHANGE_EVENT)) {

    		}
    	}

        /**
         * Create generators.
         * @throws ParameterException on parameter exception
         * @throws GtuException on GTU exception
         * @throws NetworkException if not does not exist
         * @throws ProbabilityException negative probability
         * @throws SimRuntimeException in case of sim run time exception
         * @throws RemoteException if no simulator
         */
        private void addGenerator() throws ParameterException, GtuException, NetworkException, ProbabilityException,
                SimRuntimeException, RemoteException
        {

            Random seedGenerator = new Random(1L);
            Map<String, StreamInterface> streams = new LinkedHashMap<>();
            StreamInterface stream = new MersenneTwister(Math.abs(seedGenerator.nextLong()) + 1);
            streams.put("headwayGeneration", stream);
            streams.put("gtuClass", new MersenneTwister(Math.abs(seedGenerator.nextLong()) + 1));
            getStreamInformation().addStream("headwayGeneration", stream);
            getStreamInformation().addStream("gtuClass", streams.get("gtuClass"));

            TtcRoomChecker roomChecker = new TtcRoomChecker(new Duration(10.0, DurationUnit.SI));
            IdGenerator idGenerator = new IdGenerator("");

            CarFollowingModelFactory<IdmPlus> idmPlusFactory = new IdmPlusFactory(streams.get("gtuClass"));
            ParameterSet params = new ParameterSet();
            params.setDefaultParameter(AbstractIdm.DELTA);
            
            // driver perception
    		PerceptionFactory perceptionFactory = new PerceptionFactory()
    		{
    			public Parameters getParameters() throws ParameterException
    		    {
    		        return new ParameterSet().setDefaultParameter(ParameterTypes.LOOKAHEAD).setDefaultParameter(ParameterTypes.LOOKBACKOLD)
    		                .setDefaultParameter(ParameterTypes.PERCEPTION).setDefaultParameter(ParameterTypes.LOOKBACK);
    		    }
    			
    			@Override
    			public LanePerception generatePerception(final LaneBasedGtu gtu)
    			{
    				Set<Task> tasks = new LinkedHashSet<>();
                    tasks.add(new TaskCarFollowing());
                    tasks.add(new TaskLaneChange());
                    
                    Set<BehavioralAdaptation> behavioralAdapatations = new LinkedHashSet<>();
                    // these adaptations must be present! next to adjusting parameters based on task load,
                    // they are responsible to set the parameters required for Fuller perception
                    behavioralAdapatations.add(new AdaptationSituationalAwareness());
                    behavioralAdapatations.add(new AdaptationHeadway());
                    behavioralAdapatations.add(new AdaptationSpeed());
                    
                    TaskManager taskManager = new TaskManagerAr("car-following");
                    CategoricalLanePerception perception =
                            new CategoricalLanePerception(gtu, new Fuller(tasks, behavioralAdapatations, taskManager));
                    
                    Estimation estimation = Estimation.FACTOR_ESTIMATION;
//                    Estimation estimation = Estimation.NONE;
                    Anticipation anticipation = Anticipation.CONSTANT_ACCELERATION;
                    
                    HeadwayGtuType headwayGtuType = new PerceivedHeadwayGtuType(estimation, anticipation);
                    perception.addPerceptionCategory(new DirectEgoPerception<>(perception));
                    perception.addPerceptionCategory(new DirectInfrastructurePerception(perception));
                    perception.addPerceptionCategory(new DirectNeighborsPerception(perception, headwayGtuType));
                    perception.addPerceptionCategory(new AnticipationTrafficPerception(perception));
                    perception.addPerceptionCategory(new DirectIntersectionPerception(perception, HeadwayGtuType.WRAP));
                    return perception;
    			}
    		};

            Set<MandatoryIncentive> mandatoryIncentives = new LinkedHashSet<>();
            Set<VoluntaryIncentive> voluntaryIncentives = new LinkedHashSet<>();
            Set<AccelerationIncentive> accelerationIncentives = new LinkedHashSet<>();
            mandatoryIncentives.add(new IncentiveRoute());
            if (ADDITIONAL_INCENTIVES)
            {
                // mandatoryIncentives.add(new IncentiveGetInLane());
            }
            voluntaryIncentives.add(new IncentiveSpeedWithCourtesy());
            voluntaryIncentives.add(new IncentiveKeep());
            if (ADDITIONAL_INCENTIVES)
            {
                voluntaryIncentives.add(new IncentiveCourtesy());
                voluntaryIncentives.add(new IncentiveSocioSpeed());
            }
            accelerationIncentives.add(new AccelerationSpeedLimitTransition());
            accelerationIncentives.add(new AccelerationTrafficLights());
            accelerationIncentives.add(new AccelerationConflicts());
            LaneBasedTacticalPlannerFactory<Lmrs> tacticalFactory = new LmrsFactory(idmPlusFactory,
                    perceptionFactory, SYNCHRONIZATION, COOPERATION, GapAcceptance.INFORMED, Tailgating.NONE,
                    mandatoryIncentives, voluntaryIncentives, accelerationIncentives);

            GtuType car = DefaultsNl.CAR;
            GtuType truck = DefaultsNl.TRUCK;
            Route routeAE = this.network.getShortestRouteBetween(car, this.network.getNode("A"), this.network.getNode("E"));
            Route routeAG = !NETWORK.equals("shortWeave") ? null
                    : this.network.getShortestRouteBetween(car, this.network.getNode("A"), this.network.getNode("G"));
            Route routeFE = this.network.getShortestRouteBetween(car, this.network.getNode("F"), this.network.getNode("E"));
            Route routeFG = !NETWORK.equals("shortWeave") ? null
                    : this.network.getShortestRouteBetween(car, this.network.getNode("F"), this.network.getNode("G"));

            double leftFraction = NETWORK.equals("shortWeave") ? LEFT_FRACTION : 0.0;
            List<FrequencyAndObject<Route>> routesA = new ArrayList<>();
            routesA.add(new FrequencyAndObject<>(1.0 - leftFraction, routeAE));
            routesA.add(new FrequencyAndObject<>(leftFraction, routeAG));
            List<FrequencyAndObject<Route>> routesF = new ArrayList<>();
            routesF.add(new FrequencyAndObject<>(1.0 - leftFraction, routeFE));
            routesF.add(new FrequencyAndObject<>(leftFraction, routeFG));
            Generator<Route> routeGeneratorA = new ProbabilisticRouteGenerator(routesA, stream);
            Generator<Route> routeGeneratorF = new ProbabilisticRouteGenerator(routesF, stream);

            Speed speedA = new Speed(120.0, SpeedUnit.KM_PER_HOUR);
            Speed speedF = new Speed(20.0, SpeedUnit.KM_PER_HOUR);

            CrossSectionLink linkA = (CrossSectionLink) this.network.getLink("AB");
            CrossSectionLink linkF = (CrossSectionLink) this.network.getLink("FF2");

            ParameterFactoryByType bcFactory = new ParameterFactoryByType();
            // vehicle type
            bcFactory.addParameter(car, VEHICLE_TYPE, "CAR");
            bcFactory.addParameter(car, ParameterTypes.TMIN, Duration.instantiateSI(0.1));
			bcFactory.addParameter(car, ParameterTypes.TMAX, Duration.instantiateSI(0.2));
            bcFactory.addParameter(truck, VEHICLE_TYPE, "TRUCK");
            bcFactory.addParameter(truck, ParameterTypes.TMIN, Duration.instantiateSI(1.0));
			bcFactory.addParameter(truck, ParameterTypes.TMAX, Duration.instantiateSI(2.0));
            // car vs truck variables
            bcFactory.addParameter(car, ParameterTypes.FSPEED, new DistNormal(stream, 123.7 / 120, 12.0 / 120));
            bcFactory.addParameter(car, LmrsParameters.SOCIO, new DistNormal(stream, 0.5, 0.1));
            bcFactory.addParameter(truck, ParameterTypes.A, new Acceleration(0.8, AccelerationUnit.SI));
            bcFactory.addParameter(truck, LmrsParameters.SOCIO, new DistNormal(stream, 0.5, 0.1));
            bcFactory.addParameter(Tailgating.RHO, Tailgating.RHO.getDefaultValue());
            // perception
 			bcFactory.addParameter(Fuller.TC, Fuller.TC.getDefaultValue());
 			bcFactory.addParameter(Fuller.TS_CRIT, Fuller.TS_CRIT.getDefaultValue());
 			bcFactory.addParameter(Fuller.TS_MAX, Fuller.TS_MAX.getDefaultValue());
 			bcFactory.addParameter(AdaptationSituationalAwareness.SA_MIN, AdaptationSituationalAwareness.SA_MIN.getDefaultValue());
 			bcFactory.addParameter(AdaptationSituationalAwareness.SA, AdaptationSituationalAwareness.SA.getDefaultValue());
 			bcFactory.addParameter(AdaptationSituationalAwareness.SA_MAX, AdaptationSituationalAwareness.SA_MAX.getDefaultValue());
 			bcFactory.addParameter(AdaptationSituationalAwareness.TR_MAX, AdaptationSituationalAwareness.TR_MAX.getDefaultValue());
 			bcFactory.addParameter(AdaptationHeadway.BETA_T, AdaptationHeadway.BETA_T.getDefaultValue());
 			bcFactory.addParameter(AdaptationSpeed.BETA_V0, AdaptationSpeed.BETA_V0.getDefaultValue());
 			bcFactory.addParameter(Estimation.OVER_EST, 1.0);
			// tasks
			bcFactory.addParameter(TaskManagerAr.ALPHA, TaskManagerAr.ALPHA.getDefaultValue());
			bcFactory.addParameter(TaskManagerAr.BETA, TaskManagerAr.BETA.getDefaultValue());
			bcFactory.addParameter(TaskCarFollowing.HEXP, TaskCarFollowing.HEXP.getDefaultValue());
			
            Generator<Duration> headwaysA1 = new HeadwayGenerator(getSimulator(), MAIN_DEMAND);
            Generator<Duration> headwaysA2 = new HeadwayGenerator(getSimulator(), MAIN_DEMAND);
            Generator<Duration> headwaysA3 = new HeadwayGenerator(getSimulator(), MAIN_DEMAND);
            Generator<Duration> headwaysF = new HeadwayGenerator(getSimulator(), RAMP_DEMAND);

            // speed generators
            ContinuousDistDoubleScalar.Rel<Speed, SpeedUnit> speedCar =
                    new ContinuousDistDoubleScalar.Rel<>(new DistUniform(stream, 160, 200), SpeedUnit.KM_PER_HOUR);
            ContinuousDistDoubleScalar.Rel<Speed, SpeedUnit> speedTruck =
                    new ContinuousDistDoubleScalar.Rel<>(new DistUniform(stream, 80, 95), SpeedUnit.KM_PER_HOUR);
            // strategical planner factory
            LaneBasedStrategicalRoutePlannerFactory strategicalFactory =
                    new LaneBasedStrategicalRoutePlannerFactory(tacticalFactory, bcFactory);
            // vehicle templates, with routes
            LaneBasedGtuTemplate carA =
                    new LaneBasedGtuTemplate(car, new ConstantGenerator<>(Length.instantiateSI(4.0)),
                            new ConstantGenerator<>(Length.instantiateSI(2.0)), speedCar,
                            strategicalFactory, routeGeneratorA);
            LaneBasedGtuTemplate carF =
                    new LaneBasedGtuTemplate(car, new ConstantGenerator<>(Length.instantiateSI(4.0)),
                            new ConstantGenerator<>(Length.instantiateSI(2.0)), speedCar,
        					strategicalFactory, routeGeneratorF);
            LaneBasedGtuTemplate truckA = new LaneBasedGtuTemplate(truck,
                    new ConstantGenerator<>(Length.instantiateSI(15.0)), new ConstantGenerator<>(Length.instantiateSI(2.5)),
                    speedTruck,
					strategicalFactory, routeGeneratorA);
            LaneBasedGtuTemplate truckF = new LaneBasedGtuTemplate(truck,
                    new ConstantGenerator<>(Length.instantiateSI(15.0)), new ConstantGenerator<>(Length.instantiateSI(2.5)),
                    speedTruck, 
					strategicalFactory, routeGeneratorF);
            //
            Distribution<LaneBasedGtuTemplate> gtuTypeAllCarA = new Distribution<>(streams.get("gtuClass"));
            gtuTypeAllCarA.add(new FrequencyAndObject<>(1.0, carA));

            Distribution<LaneBasedGtuTemplate> gtuType1LaneF = new Distribution<>(streams.get("gtuClass"));
            gtuType1LaneF.add(new FrequencyAndObject<>(1.0 - 2 * TRUCK_FRACTION, carF));
            gtuType1LaneF.add(new FrequencyAndObject<>(2 * TRUCK_FRACTION, truckF));

            Distribution<LaneBasedGtuTemplate> gtuType2ndLaneA = new Distribution<>(streams.get("gtuClass"));
            gtuType2ndLaneA.add(new FrequencyAndObject<>(1.0 - 2 * TRUCK_FRACTION, carA));
            gtuType2ndLaneA.add(new FrequencyAndObject<>(2 * TRUCK_FRACTION, truckA));

            Distribution<LaneBasedGtuTemplate> gtuType3rdLaneA = new Distribution<>(streams.get("gtuClass"));
            gtuType3rdLaneA.add(new FrequencyAndObject<>(1.0 - 3 * TRUCK_FRACTION, carA));
            gtuType3rdLaneA.add(new FrequencyAndObject<>(3 * TRUCK_FRACTION, truckA));

            GtuColorer colorer = new LmrsSwitchableColorer(DefaultsNl.GTU_TYPE_COLORS.toMap());
            makeGenerator(getLane(linkA, "FORWARD1"), speedA, "gen1", idGenerator, gtuTypeAllCarA, headwaysA1, colorer,
                    roomChecker, bcFactory, tacticalFactory, SIMTIME, streams.get("gtuClass"));
            if (NETWORK.equals("shortWeave"))
            {
                makeGenerator(getLane(linkA, "FORWARD2"), speedA, "gen2", idGenerator, gtuTypeAllCarA, headwaysA2, colorer,
                        roomChecker, bcFactory, tacticalFactory, SIMTIME, streams.get("gtuClass"));
                makeGenerator(getLane(linkA, "FORWARD3"), speedA, "gen3", idGenerator, gtuType3rdLaneA, headwaysA3, colorer,
                        roomChecker, bcFactory, tacticalFactory, SIMTIME, streams.get("gtuClass"));
            }
            else
            {
                makeGenerator(getLane(linkA, "FORWARD2"), speedA, "gen2", idGenerator, gtuType2ndLaneA, headwaysA2, colorer,
                        roomChecker, bcFactory, tacticalFactory, SIMTIME, streams.get("gtuClass"));
            }
            makeGenerator(getLane(linkF, "FORWARD1"), speedF, "gen4", idGenerator, gtuType1LaneF, headwaysF, colorer,
                    roomChecker, bcFactory, tacticalFactory, SIMTIME, streams.get("gtuClass"));

            new SpeedSign("sign1", getLane(linkA, "FORWARD1"), Length.instantiateSI(10), this.getSimulator(),
                    new Speed(130.0, SpeedUnit.KM_PER_HOUR), DefaultsNl.VEHICLE, Duration.ZERO,
                    new Duration(24, DurationUnit.HOUR));

        }

        /**
         * Get lane from link by id.
         * @param link CrossSectionLink; link
         * @param id String; id
         * @return lane
         */
        private Lane getLane(final CrossSectionLink link, final String id)
        {
            return (Lane) link.getCrossSectionElement(id);
        }

        /**
         * @param lane Lane; the reference lane for this generator
         * @param generationSpeed Speed; the speed of the GTU
         * @param id String; the id of the generator itself
         * @param idGenerator IdGenerator; the generator for the ID
         * @param distribution Distribution&lt;LaneBasedTemplateGTUType&gt;; the type generator for the GTU
         * @param headwayGenerator Generator&lt;Duration&gt;; the headway generator for the GTU
         * @param gtuColorer GtuColorer; the GTU colorer for animation
         * @param roomChecker RoomChecker; the checker to see if there is room for the GTU
         * @param bcFactory ParameterFactory; the factory to generate parameters for the GTU
         * @param tacticalFactory LaneBasedTacticalPlannerFactory&lt;?&gt;; the generator for the tactical planner
         * @param simulationTime Time; simulation time
         * @param stream StreamInterface; random numbers stream
         * @throws SimRuntimeException in case of scheduling problems
         * @throws ProbabilityException in case of an illegal probability distribution
         * @throws GtuException in case the GTU is inconsistent
         * @throws ParameterException in case a parameter for the perception is missing
         * @throws NetworkException if the object could not be added to the network
         */
        private void makeGenerator(final Lane lane, final Speed generationSpeed, final String id, final IdGenerator idGenerator,
                final Distribution<LaneBasedGtuTemplate> distribution, final Generator<Duration> headwayGenerator,
                final GtuColorer gtuColorer, final RoomChecker roomChecker, final ParameterFactory bcFactory,
                final LaneBasedTacticalPlannerFactory<?> tacticalFactory, final Time simulationTime,
                final StreamInterface stream) throws SimRuntimeException, ProbabilityException, GtuException, ParameterException, NetworkException
        {

            Set<LanePosition> initialLongitudinalPositions = new LinkedHashSet<>();
            // TODO DIR_MINUS
            initialLongitudinalPositions.add(new LanePosition(lane, new Length(5.0, LengthUnit.SI)));
            LaneBasedGtuTemplateDistribution characteristicsGenerator =
                    new LaneBasedGtuTemplateDistribution(distribution);
            new LaneBasedGtuGenerator(id, headwayGenerator, characteristicsGenerator,
                    GeneratorPositions.create(initialLongitudinalPositions, stream), this.network, getSimulator(), roomChecker,
                    idGenerator);
        }

    }

    /**
     * <p>
     * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.
     * <br>
     * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
     * </p>
     * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
     * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
     * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
     */
    private static class HeadwayGenerator implements Generator<Duration>
    {
        /** the simulator. */
        private final OtsSimulatorInterface simulator;

        /** Demand level. */
        private final Frequency demand;

        /**
         * @param simulator OtsSimulatorInterface; the simulator
         * @param demand Frequency; demand
         */
        HeadwayGenerator(final OtsSimulatorInterface simulator, final Frequency demand)
        {
            this.simulator = simulator;
            this.demand = demand;
        }

        /** {@inheritDoc} */
        @Override
        public Duration draw() throws ProbabilityException, ParameterException
        {
            return new Duration(
                    -Math.log(this.simulator.getModel().getStream("headwayGeneration").nextDouble()) / this.demand.si,
                    DurationUnit.SI);
        }

    }

}
