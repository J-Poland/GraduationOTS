package sim.demo;

import java.net.URL;
import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.djunits.unit.DurationUnit;
import org.djunits.unit.FrequencyUnit;
import org.djunits.unit.LengthUnit;
import org.djunits.unit.SpeedUnit;
import org.djunits.unit.TimeUnit;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Frequency;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djutils.event.Event;
import org.djutils.event.EventListener;
import org.djutils.immutablecollections.ImmutableCollection;
import org.djutils.immutablecollections.ImmutableList;
import org.djutils.io.URLResource;
import org.opentrafficsim.animation.GraphLaneUtil;
import org.opentrafficsim.animation.colorer.LmrsSwitchableColorer;
import org.opentrafficsim.animation.gtu.colorer.GtuColorer;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterSet;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.distributions.ConstantGenerator;
import org.opentrafficsim.core.distributions.Distribution;
import org.opentrafficsim.core.distributions.Distribution.FrequencyAndObject;
import org.opentrafficsim.core.distributions.Generator;
import org.opentrafficsim.core.distributions.ProbabilityException;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.perception.DirectEgoPerception;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.idgenerator.IdGenerator;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.route.ProbabilisticRouteGenerator;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.core.parameters.ParameterFactory;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.core.units.distributions.ContinuousDistDoubleScalar;
import org.opentrafficsim.draw.graphs.FundamentalDiagram;
import org.opentrafficsim.draw.graphs.GraphCrossSection;
import org.opentrafficsim.draw.graphs.GraphPath;
import org.opentrafficsim.draw.graphs.GraphPath.Section;
import org.opentrafficsim.draw.graphs.FundamentalDiagram.FdSource;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator.RoomChecker;
import org.opentrafficsim.road.gtu.generator.TtcRoomChecker;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplate;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplateDistribution;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.TaskManager;
import org.opentrafficsim.road.gtu.lane.tactical.LaneBasedTacticalPlannerFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModelFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlus;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlusFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationConflicts;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationIncentive;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationSpeedLimitTransition;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.AccelerationTrafficLights;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveKeep;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveRoute;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSocioSpeed;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSpeedWithCourtesy;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.Lmrs;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.LmrsFactory;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.Cooperation;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.GapAcceptance;
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
import org.opentrafficsim.road.network.sampling.LaneDataRoad;
import org.opentrafficsim.road.network.sampling.RoadSampler;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.simulators.SimulatorInterface;
import nl.tudelft.simulation.jstats.distributions.DistUniform;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import sim.demo.behavior.TrafficInteractionAdaptations.CarFollowingBehavior;
import sim.demo.lmrs.CustomCooperation;
import sim.demo.mental.CarFollowingTask;
import sim.demo.mental.CustomAdaptationHeadway;
import sim.demo.mental.CustomAdaptationSituationalAwareness;
import sim.demo.mental.LaneChangeTask;
import sim.demo.mental.TaskManagerAr;

import org.opentrafficsim.road.gtu.lane.perception.AbstractLanePerception;
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
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationHeadway;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSituationalAwareness;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSpeed;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller.BehavioralAdaptation;
import org.opentrafficsim.road.gtu.lane.perception.mental.Task;


/**
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA,
 * Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See
 * <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim
 * License</a>.
 * </p>
 * 
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class VehicleAutomationModel extends AbstractOtsModel implements EventListener
{
	
	/** */
	private static final long serialVersionUID = 1L;
	
	/** Output data manager */
	private final OutputDataManager outputDataManager = new OutputDataManager();
	
	/** The network. */
	private OtsSimulatorInterface simulator;
	
	/** The network. */
	private RoadNetwork network;

	/** Network. */
	// Rijkswaterstaat ROA Chapter 6 contains standards on turbulence regions
	// also the merge lane lengths are defined
	static final String networkString = "twoLaneFreewayWithOnRamp2";
	
	/** Speed limit and speed variation. */
	static final double speedLimit = 130.0;
	static final double speedMin = 120.0;
	static final double speedMax = 140.0;
	
	/** HDV length and width. */
	static final double hdvLength = 4.0;
	static final double hdvWidth = 2.0;
	
	/** AV length and width. */
	static final double avLength = 4.0;
	static final double avWidth = 2.0;
	
	/** Synchronization. */
	static final Synchronization synchronizationMethod = Synchronization.ALIGN_GAP;

	/** Cooperation. */
	static final Cooperation cooperationMethod = Cooperation.PASSIVE_MOVING;
    
    /** Sampler. */
    private RoadSampler sampler;
    
    /** Cross section for data recording. */
    GraphCrossSection<LaneDataRoad> crossSection;
    
    private List<LaneDataRoad> allLanes;
    
    /** Source. */
    private FdSource source;
    
    /** Map for GTU add times */
    private LinkedHashMap<String, Double> gtuAddTimesMap;
    
    /** Map for GTU remove times */
    private LinkedHashMap<String, Double> gtuRemoveTimesMap;
    
    /** Last recording of output trajectory data*/
    private double trajectoryLastTime;
	
	
	// input parameters
	
	/** Vehicle Automation Types and Parameters. */
	static VehicleConfigurationsBundle vehicleVars;
	
	/** Simulation configuration object. */
	public VehicleAutomationModelParameters simConfig;
	
	/** Generator seed. */
	static Random seedGenerator;
	
	/** Total run time of simulation. */
	static Time simTime;
	
	/** Gtu type fractions in traffic. */
	static ArrayList<Double> gtuFractions;

	/** Left traffic fraction. */
	static double leftFraction;

	/** Main demand per lane. Default = 1000 */
	static Frequency mainDemand;

	/** Ramp demand. Default = 500 */
	static Frequency rampDemand;
	
	/** Use additional incentives. */
	static boolean additionalIncentives;

	/** Left traffic fraction. */
	static Duration tMin;
	
	/** Left traffic fraction. */
	static Duration tMax;
	
	/** File path for output data. */
	static String singleOutputFilePath;
	static String intermediateMeanValuesFilePath;
	static String sequenceOutputFilePath;
	static String trajectoryOutputFilePath;
	

	/**
	 * Constructor for VehicleAutomationModel class.
	 * @param simulator OtsSimulatorInterface; the simulator
	 */
	public VehicleAutomationModel(final OtsSimulatorInterface simulator, final VehicleAutomationModelParameters simConfig) {
		// parse simulator
		super(simulator);
		
		this.simulator = simulator;
		
		// set simulation configuration
		this.simConfig = simConfig;
		setSimParameters();
	}
	
	/**
	 * Set all required simulation parameters by the simConfig object.
	 */
	public void setSimParameters() {
		// initialise class variables
		gtuAddTimesMap = new LinkedHashMap<String, Double>();
		gtuRemoveTimesMap = new LinkedHashMap<String, Double>();
		gtuFractions = new ArrayList<Double>();
		trajectoryLastTime = -100.0;
		
		// initialise input variables
		simTime = new Time(simConfig.getSimTime(), TimeUnit.BASE_SECOND);
		seedGenerator = new Random(simConfig.getSeed());
		gtuFractions.add(simConfig.getLevel0Fraction());
		gtuFractions.add(simConfig.getLevel1Fraction());
		gtuFractions.add(simConfig.getLevel2Fraction());
		gtuFractions.add(simConfig.getLevel3Fraction());
		leftFraction = simConfig.getLeftFraction();
		mainDemand = new Frequency(simConfig.getMainDemand(), FrequencyUnit.PER_HOUR);
		rampDemand = new Frequency(simConfig.getRampDemand(), FrequencyUnit.PER_HOUR);
		additionalIncentives = simConfig.getAdditionalIncentives();
		tMin = Duration.instantiateSI(simConfig.getTMin());
		tMax = Duration.instantiateSI(simConfig.getTMax());
		singleOutputFilePath = simConfig.getSingleOutputFilePath();
		intermediateMeanValuesFilePath = simConfig.getIntermediateMeanValuesFilePath();
		sequenceOutputFilePath = simConfig.getSequenceOutputFilePath();
		trajectoryOutputFilePath = simConfig.getTrajectoryOutputFilePath();
	}

	/**
	 * @param network RoadNetwork; set network.
	 */
	public void setNetwork(final RoadNetwork network) {
		this.network = network;
	}

	/** {@inheritDoc} */
	@Override
	public void constructModel() throws SimRuntimeException {
		try {
			// load network from xml file
			URL xmlURL = URLResource.getResource("C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\"
													+ "traffic-sim\\src\\main\\resources\\networks\\" + networkString + ".xml");
			this.network = new RoadNetwork("VehicleAutomationSimulation", getSimulator());
			new XmlParser(this.network).setUrl(xmlURL).build();
			
			// create Generators
			addGenerator();
			
			// enable event notifications by adding listeners
			addSubscriptions();
			
			// create sampler to collect data
			createSampler();
			
			// setup FD data
			Duration updateInterval = Duration.instantiateSI(2.0);
			createNetworkFdValues(updateInterval);
	        
	        // recalculate FD data at a fixed interval
	        this.simulator.scheduleEventRel(updateInterval, this::recalculateFdValues);
	        
		} catch (Exception exception) {
			exception.printStackTrace();
		}
	}
	
	private void createSampler() {
		this.sampler = new RoadSampler(this.network);
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
			
			System.out.println("Total simulation time: " + simTime.si);
			
			// initialise count output variables
			outputDataManager.setSingleValue("criticalTtc", 0);
			outputDataManager.setSingleValue("laneChangesToRightBeforeRamp", 0);
			outputDataManager.setSingleValue("laneChangesToRightOnRamp", 0);
			outputDataManager.setSingleValue("laneChangesToRightAfterRamp", 0);
			outputDataManager.setSingleValue("laneChangesToLeftBeforeRamp", 0);
			outputDataManager.setSingleValue("laneChangesToLeftOnRamp", 0);
			outputDataManager.setSingleValue("laneChangesToLeftAfterRamp", 0);
		}
		
		// handle end of replication event
		if (event.getType().equals(SimulatorInterface.STOP_EVENT)) {
			System.out.println("Simulation stop time: " + this.simulator.getSimulatorAbsTime().si);
			
			System.out.println("Stop simulation.");
			
			saveFdValues();
			
			outputDataManager.exportToCsv(singleOutputFilePath, sequenceOutputFilePath, trajectoryOutputFilePath);
		}
		
		// handle simulation time changed event
		if (event.getType().equals(SimulatorInterface.TIME_CHANGED_EVENT)) {
			// track trajectory data
	        TrackTrajectories(2.0);
		}
		
		// subscribe to GTU lane events when GTUs are added to the simulation
		if (event.getType().equals(Network.GTU_ADD_EVENT)) {
			String gtuId = (String) event.getContent();
			LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
			
			// add GTU listeners
			gtu.addListener(this, Gtu.MOVE_EVENT);
			gtu.addListener(this, LaneBasedGtu.LANE_CHANGE_EVENT);
			
			// add GTU to gtuAddTimesMap to track GTU travel time
			gtuAddTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().getSI());
		}
		
		// disable subscription to GTU lane events when GTUs leave the simulation
		if (event.getType().equals(Network.GTU_REMOVE_EVENT)) {
			String gtuId = (String) event.getContent();
			LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
			
			// remove GTU listeners
			gtu.removeListener(this, Gtu.MOVE_EVENT);
			gtu.removeListener(this, LaneBasedGtu.LANE_CHANGE_EVENT);
			
			// add GTU to gtuRemoveTimesMap and calculate travelled time
			gtuRemoveTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().getSI());
			calculateRemovedGtuTravelTime(gtu, gtuId);
		}
		
		// handle GTU move events
		if (event.getType().equals(Gtu.MOVE_EVENT)) {
			String gtuId = (String) ((Object[]) event.getContent())[0];
	        LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
	        
	        // track GTU data
	        TrackGtuData(gtu);
	        
	        // change driving behaviour because of surrounding traffic
//	        try {
	        	// CF
//	        	CarFollowingBehavior carFollowingBehavior = new CarFollowingBehavior(gtu);
//	        	carFollowingBehavior.adaptToVehicleInFront();
	        	// LC
	        	// lane changing behaviour is changed when lmrs cooperation decision is made
	        	// CustomCooperation contains this logic
//			} catch (OperationalPlanException e) {
//				e.printStackTrace();
//			} catch (ParameterException e) {
//				e.printStackTrace();
//			}
		}
		
		// process change event
		if (event.getType().equals(LaneBasedGtu.LANE_CHANGE_EVENT)) {

			String gtuId = (String) ((Object[]) event.getContent())[0];
			// LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
			String gtuDirection = (String) ((Object[]) event.getContent())[1];
			String gtuLink = (String) ((Object[]) event.getContent())[2];
			String gtuLane = (String) ((Object[]) event.getContent())[3];
			
			TrackLaneChanges(gtuId, gtuDirection, gtuLink, gtuLane);
		}
	}
	
	/**
	 * Convert a Lane to LaneDataRoad.
	 * 
	 * @param lane Lane; the lane to be converted
	 * @return LaneDataRoad; the converted LaneDataRoad
	 */
	private LaneDataRoad convertToLaneDataRoad(Lane lane) {
	    return new LaneDataRoad(lane);
	}

	/** {@inheritDoc} */
	@Override
	public RoadNetwork getNetwork() {
		return this.network;
	}

	/**
	 * Create generators.
	 * 
	 * @throws ParameterException   on parameter exception
	 * @throws GtuException         on GTU exception
	 * @throws NetworkException     if not does not exist
	 * @throws ProbabilityException negative probability
	 * @throws SimRuntimeException  in case of sim run time exception
	 * @throws RemoteException      if no simulator
	 */
	private void addGenerator() throws ParameterException, GtuException, NetworkException, ProbabilityException,
			SimRuntimeException, RemoteException {
		
		// source settings
		Map<String, StreamInterface> streams = new LinkedHashMap<>();
		StreamInterface stream = new MersenneTwister(Math.abs(seedGenerator.nextLong()) + 1);
		streams.put("headwayGeneration", stream);
		streams.put("gtuClass", new MersenneTwister(Math.abs(seedGenerator.nextLong()) + 1));
		getStreamInformation().addStream("headwayGeneration", stream);
		getStreamInformation().addStream("gtuClass", streams.get("gtuClass"));
		TtcRoomChecker roomChecker = new TtcRoomChecker(new Duration(10.0, DurationUnit.SI));
		IdGenerator idGenerator = new IdGenerator("");
		
		// car following model
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
                tasks.add(new CarFollowingTask());
                tasks.add(new LaneChangeTask());
                
                Set<BehavioralAdaptation> behavioralAdapatations = new LinkedHashSet<>();
                // these adaptations must be present! next to adjusting parameters based on task load,
                // they are responsible to set the parameters required for Fuller perception
                behavioralAdapatations.add(new CustomAdaptationSituationalAwareness());
                behavioralAdapatations.add(new CustomAdaptationHeadway());
                behavioralAdapatations.add(new AdaptationSpeed());
                
                TaskManager taskManager = new TaskManagerAr("car-following");
                CategoricalLanePerception perception =
                        new CategoricalLanePerception(gtu, new Fuller(tasks, behavioralAdapatations, taskManager));
                
                Estimation estimation = Estimation.NONE;
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
		
		// drive incentives
		Set<MandatoryIncentive> mandatoryIncentives = new LinkedHashSet<>();
		Set<VoluntaryIncentive> voluntaryIncentives = new LinkedHashSet<>();
		Set<AccelerationIncentive> accelerationIncentives = new LinkedHashSet<>();
		mandatoryIncentives.add(new IncentiveRoute());
		if (additionalIncentives) {
			// mandatoryIncentives.add(new IncentiveGetInLane());
		}
		voluntaryIncentives.add(new IncentiveSpeedWithCourtesy());
		voluntaryIncentives.add(new IncentiveKeep());
		if (additionalIncentives) {
			voluntaryIncentives.add(new IncentiveCourtesy());
			voluntaryIncentives.add(new IncentiveSocioSpeed());
		}
		accelerationIncentives.add(new AccelerationSpeedLimitTransition());
		accelerationIncentives.add(new AccelerationTrafficLights());
		accelerationIncentives.add(new AccelerationConflicts());
//		LaneBasedTacticalPlannerFactory<Lmrs> tacticalFactory = new LmrsFactory(idmPlusFactory,
//				new DefaultLmrsPerceptionFactory(), synchronizationMethod, cooperationMethod, GapAcceptance.INFORMED,
//				Tailgating.NONE, mandatoryIncentives, voluntaryIncentives, accelerationIncentives);
		LaneBasedTacticalPlannerFactory<Lmrs> tacticalFactory = new LmrsFactory(idmPlusFactory,
				perceptionFactory, synchronizationMethod, cooperationMethod, GapAcceptance.INFORMED,
				Tailgating.NONE, mandatoryIncentives, voluntaryIncentives, accelerationIncentives);
		
		// retrieve automation vehicle types and parameters
		VehicleConfigurations vehicleConfig = new VehicleConfigurations();
		vehicleVars = vehicleConfig.setVehicleTypes(stream);
		ArrayList<GtuType> availableGtuTypes = vehicleVars.getAutomationGtuTypes();
		ParameterFactoryByType paramFactory = vehicleVars.getParameterFactory();
		
		// create routes on RoadNetwork (routes are the same for all gtu types)
		Route routeAD = this.network.getShortestRouteBetween(availableGtuTypes.get(0), this.network.getNode("A"), this.network.getNode("D"));
		Route routeED = this.network.getShortestRouteBetween(availableGtuTypes.get(0), this.network.getNode("E"), this.network.getNode("D"));
		
		// create route generators
		List<FrequencyAndObject<Route>> routesA = new ArrayList<>();
		routesA.add(new FrequencyAndObject<>(1.0 - leftFraction, routeAD));
//		routesA.add(new FrequencyAndObject<>(leftFraction, routeAG));
		List<FrequencyAndObject<Route>> routesE = new ArrayList<>();
		routesE.add(new FrequencyAndObject<>(1.0 - leftFraction, routeED));
//		routesF.add(new FrequencyAndObject<>(leftFraction, routeFG));
		Generator<Route> routeGeneratorA = new ProbabilisticRouteGenerator(routesA, stream);
		Generator<Route> routeGeneratorE = new ProbabilisticRouteGenerator(routesE, stream);

		// set initial speed for generators
		double rampSpeed = 50.0;	// Rijkswaterstaat ROA 6.1.1 (table 6.1)
		Speed speedA = new Speed(speedLimit, SpeedUnit.KM_PER_HOUR);
		Speed speedE = new Speed(rampSpeed, SpeedUnit.KM_PER_HOUR);
		
		// define cross sections for vehicle generation
		CrossSectionLink linkA = (CrossSectionLink) this.network.getLink("AB");
		CrossSectionLink linkF = (CrossSectionLink) this.network.getLink("EE2");
		
		// create headway generators
		Generator<Duration> headwaysA1 = new HeadwayGenerator(getSimulator(), mainDemand);
		Generator<Duration> headwaysA2 = new HeadwayGenerator(getSimulator(), mainDemand);
		Generator<Duration> headwaysE = new HeadwayGenerator(getSimulator(), rampDemand);

		// speed generators (the same for all gtu types)
		ContinuousDistDoubleScalar.Rel<Speed, SpeedUnit> gtuSpeed = new ContinuousDistDoubleScalar.Rel<>(
				new DistUniform(stream, speedMin, speedMax), SpeedUnit.KM_PER_HOUR);
		
		// strategical planner factory
		LaneBasedStrategicalRoutePlannerFactory strategicalFactory = new LaneBasedStrategicalRoutePlannerFactory(
				tacticalFactory, paramFactory);
		
		// create vehicle templates, with routes from main road and ramp
		ArrayList<LaneBasedGtuTemplate> gtuMainTemplates = new ArrayList<LaneBasedGtuTemplate>();
		ArrayList<LaneBasedGtuTemplate> gtuRampTemplates = new ArrayList<LaneBasedGtuTemplate>();
		for (GtuType gtuType : availableGtuTypes) {
			// main road
			LaneBasedGtuTemplate templateA = 
					new LaneBasedGtuTemplate(gtuType, 
					new ConstantGenerator<>(Length.instantiateSI(hdvLength)),
					new ConstantGenerator<>(Length.instantiateSI(hdvWidth)), 
					gtuSpeed, strategicalFactory, routeGeneratorA);
			gtuMainTemplates.add(templateA);
			// ramp
			LaneBasedGtuTemplate templateE = 
					new LaneBasedGtuTemplate(gtuType, 
					new ConstantGenerator<>(Length.instantiateSI(hdvLength)),
					new ConstantGenerator<>(Length.instantiateSI(hdvWidth)), 
					gtuSpeed, strategicalFactory, routeGeneratorE);
			gtuRampTemplates.add(templateE);
		}
		
		// set vehicle distributions for
		// left lane
		Distribution<LaneBasedGtuTemplate> gtuTypeLaneA1 = new Distribution<>(streams.get("gtuClass"));
		for (int i = 0; i < gtuMainTemplates.size(); i++) {
			gtuTypeLaneA1.add(new FrequencyAndObject<>(gtuFractions.get(i), gtuMainTemplates.get(i)));
		}
		// right lane
		Distribution<LaneBasedGtuTemplate> gtuTypeLaneA2 = new Distribution<>(streams.get("gtuClass"));
		for (int i = 0; i < gtuMainTemplates.size(); i++) {
			gtuTypeLaneA2.add(new FrequencyAndObject<>(gtuFractions.get(i), gtuMainTemplates.get(i)));
		}
		// ramp lane
		Distribution<LaneBasedGtuTemplate> gtuTypeLaneE = new Distribution<>(streams.get("gtuClass"));
		for (int i = 0; i < gtuMainTemplates.size(); i++) {
			gtuTypeLaneE.add(new FrequencyAndObject<>(gtuFractions.get(i), gtuRampTemplates.get(i)));
		}

		// set vehicle colours
		GtuColorer colorer = new LmrsSwitchableColorer(DefaultsNl.GTU_TYPE_COLORS.toMap());
		
		// create generators for all lanes with corresponding objects
		// left lane
		makeGenerator(getLane(linkA, "FORWARD1"), speedA, "gen1", idGenerator, gtuTypeLaneA1, headwaysA1, colorer,
				roomChecker, paramFactory, tacticalFactory, simTime, streams.get("gtuClass"));
		// right lane
		makeGenerator(getLane(linkA, "FORWARD2"), speedA, "gen2", idGenerator, gtuTypeLaneA2, headwaysA2, colorer,
				roomChecker, paramFactory, tacticalFactory, simTime, streams.get("gtuClass"));
		// ramp lane
		makeGenerator(getLane(linkF, "FORWARD1"), speedE, "gen3", idGenerator, gtuTypeLaneE, headwaysE, colorer,
				roomChecker, paramFactory, tacticalFactory, simTime, streams.get("gtuClass"));

		new SpeedSign("sign1", getLane(linkA, "FORWARD1"), Length.instantiateSI(10), this.getSimulator(),
				new Speed(speedLimit, SpeedUnit.KM_PER_HOUR), DefaultsNl.VEHICLE, Duration.ZERO,
				new Duration(24, DurationUnit.HOUR));

	}

	/**
	 * Get lane from link by id.
	 * 
	 * @param link CrossSectionLink; link
	 * @param id   String; id
	 * @return lane
	 */
	private Lane getLane(final CrossSectionLink link, final String id) {
		return (Lane) link.getCrossSectionElement(id);
	}

	/**
	 * @param lane             Lane; the reference lane for this generator
	 * @param generationSpeed  Speed; the speed of the GTU
	 * @param id               String; the id of the generator itself
	 * @param idGenerator      IdGenerator; the generator for the ID
	 * @param distribution     Distribution&lt;LaneBasedTemplateGTUType&gt;; the
	 *                         type generator for the GTU
	 * @param headwayGenerator Generator&lt;Duration&gt;; the headway generator for
	 *                         the GTU
	 * @param gtuColorer       GtuColorer; the GTU colorer for animation
	 * @param roomChecker      RoomChecker; the checker to see if there is room for
	 *                         the GTU
	 * @param bcFactory        ParameterFactory; the factory to generate parameters
	 *                         for the GTU
	 * @param tacticalFactory  LaneBasedTacticalPlannerFactory&lt;?&gt;; the
	 *                         generator for the tactical planner
	 * @param simulationTime   Time; simulation time
	 * @param stream           StreamInterface; random numbers stream
	 * @throws SimRuntimeException  in case of scheduling problems
	 * @throws ProbabilityException in case of an illegal probability distribution
	 * @throws GtuException         in case the GTU is inconsistent
	 * @throws ParameterException   in case a parameter for the perception is
	 *                              missing
	 * @throws NetworkException     if the object could not be added to the network
	 */
	private void makeGenerator(final Lane lane, final Speed generationSpeed, final String id,
			final IdGenerator idGenerator, final Distribution<LaneBasedGtuTemplate> distribution,
			final Generator<Duration> headwayGenerator, final GtuColorer gtuColorer, final RoomChecker roomChecker,
			final ParameterFactory bcFactory, final LaneBasedTacticalPlannerFactory<?> tacticalFactory,
			final Time simulationTime, final StreamInterface stream)
			throws SimRuntimeException, ProbabilityException, GtuException, ParameterException, NetworkException {

		Set<LanePosition> initialLongitudinalPositions = new LinkedHashSet<>();
		// TODO DIR_MINUS
		initialLongitudinalPositions.add(new LanePosition(lane, new Length(5.0, LengthUnit.SI)));
		LaneBasedGtuTemplateDistribution characteristicsGenerator = new LaneBasedGtuTemplateDistribution(distribution);
		new LaneBasedGtuGenerator(id, headwayGenerator, characteristicsGenerator,
				GeneratorPositions.create(initialLongitudinalPositions, stream), this.network, getSimulator(),
				roomChecker, idGenerator);
	}

	/**
	 * <p>
	 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA,
	 * Delft, the Netherlands. All rights reserved. <br>
	 * BSD-style license. See
	 * <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim
	 * License</a>.
	 * </p>
	 * 
	 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
	 * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
	 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
	 */
	private static class HeadwayGenerator implements Generator<Duration> {
		/** the simulator. */
		private final OtsSimulatorInterface simulator;

		/** Demand level. */
		private final Frequency demand;

		/**
		 * @param simulator OtsSimulatorInterface; the simulator
		 * @param demand    Frequency; demand
		 */
		HeadwayGenerator(final OtsSimulatorInterface simulator, final Frequency demand) {
			this.simulator = simulator;
			this.demand = demand;
		}

		/** {@inheritDoc} */
		@Override
		public Duration draw() throws ProbabilityException, ParameterException {
			return new Duration(
					-Math.log(this.simulator.getModel().getStream("headwayGeneration").nextDouble()) / this.demand.si,
					DurationUnit.SI);
		}
	}
	
	/**
	 * Function to track GTU trajectories.
	 * @param gtu
	 */
	public void TrackTrajectories(double frequency)
	{
		// check whether a recording was already performed for this time interval
		// skip this for first time, initially trajectoryLastTime will be < 0
		double now = this.simulator.getSimulatorAbsTime().si;
		if (trajectoryLastTime > 0) {
			// it is not the next time interval?
			if (now < trajectoryLastTime + frequency) {
				// do not record trajectories
				return;
			}
		}
		
		// record trajectories
		// go through links to track
		String[] linksToTrack = {"AB", "BC", "CD"};
		for (String link : linksToTrack) {
			// go through lanes of this link
			List<Lane> lanesInLink = ((CrossSectionLink) this.network.getLink(link)).getLanes();
			int numOfLanes = lanesInLink.size();
			for (int i = 0; i < numOfLanes; i++) {
				Lane lane = lanesInLink.get(i);
				ImmutableList<LaneBasedGtu> gtus = lane.getGtuList();
				// track trajectory of GTUs in this lane
				for (Gtu gtu : gtus) {
					String gtuId = gtu.getId();
					double time = gtu.getSimulator().getSimulatorAbsTime().si;
					double distance = gtu.getLocation().getX();
					double speed = gtu.getSpeed().si;
					
					// store values
					outputDataManager.saveTrajectory(gtuId, link, lane.getId(), time, distance, speed);
				}
			}
			
			// set last recording time
			trajectoryLastTime = now;
		}
	}
	
	/**
	 * Function to track GTU info within the simulation.
	 * @param gtu
	 */
	public void TrackGtuData(LaneBasedGtu gtu)
	{
		// main info
		String gtuId = gtu.getId();
    	double time = this.simulator.getSimulatorAbsTime().si;
    	
		// get headway info
		HeadwayInfo headwayInfo = null;
		try {
			headwayInfo = calculateHeadwayInfo(gtu);
		} catch (OperationalPlanException | ParameterException e) {
			e.printStackTrace();
		}
        
        if (headwayInfo != null) {
        	// store single headway info for mean calculations
        	double headwayTime = headwayInfo.getTtc();
        	double headwayDistance = headwayInfo.getHeadwayDistance();
        	boolean headwayCriticalTtc = headwayInfo.isCriticalTtc();
        	if (headwayTime != Double.POSITIVE_INFINITY) {
        		outputDataManager.addToMeanList("meanHeadwayTime", headwayTime);
        	}
        	if (headwayDistance != Double.POSITIVE_INFINITY) {
        		outputDataManager.addToMeanList("meanHeadwayDistance", headwayDistance);
        	}
        	if (headwayCriticalTtc) {
        		outputDataManager.increaseSingleCount("criticalTtc");
        	}
        	
        	// store sequence headway info for detailed data
        	outputDataManager.addSequenceData(gtuId, time, "headwayTime", headwayTime);
        	outputDataManager.addSequenceData(gtuId, time, "headwayDistance", headwayDistance);
        	outputDataManager.addSequenceData(gtuId, time, "criticalTtc", headwayCriticalTtc);
        	double desiredHeadwayTime = gtu.getParameters().getParameterOrNull(ParameterTypes.T).si;
        	outputDataManager.addSequenceData(gtuId, time, "desiredHeadwayTimeError", (headwayTime - desiredHeadwayTime));
        }
        
        // store other detailed data
        outputDataManager.addSequenceData(gtuId, time, "type", gtu.getType().toString());
        outputDataManager.addSequenceData(gtuId, time, "acceleration", gtu.getAcceleration().toString());
        outputDataManager.addSequenceData(gtuId, time, "speed", gtu.getSpeed().toString());
        outputDataManager.addSequenceData(gtuId, time, "taskSaturation", gtu.getParameters().getParameterOrNull(Fuller.TS).toString());
	}
	
	/**
	 * Function to calculate headway info. It calculates the headway distance, headway time, and time-to-collision.
	 * @param gtu
	 * @return HeadwayInfo object; An class object containing the headway info values
	 * @throws OperationalPlanException
	 * @throws ParameterException
	 */
	public HeadwayInfo calculateHeadwayInfo(Gtu gtu) throws OperationalPlanException, ParameterException {
	    if (gtu instanceof LaneBasedGtu) {
	        LaneBasedGtu laneBasedGtu = (LaneBasedGtu) gtu;
	        AbstractLanePerception perception = (AbstractLanePerception) laneBasedGtu.getTacticalPlanner().getPerception();

	        DirectNeighborsPerception neighborPerception = perception.getPerceptionCategory(DirectNeighborsPerception.class);
	        
	        double responseTime = gtu.getParameters().getParameter(ParameterTypes.TR).si;
	        
	        if (neighborPerception != null) {
	        	PerceptionCollectable<HeadwayGtu,LaneBasedGtu> perceptionCollectable = neighborPerception.getLeaders(RelativeLane.CURRENT);
	        	HeadwayGtu headway = perceptionCollectable.first();
	        	if (headway == null) {
	        		return new HeadwayInfo(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, false);
	        	}
	            double distance = headway.getDistance().si;
	            double speed = laneBasedGtu.getSpeed().si;
	            double ttc = distance / speed;

	            if (speed > 0) {
	            	// return headway distance and time-to-collision
	            	boolean criticalTtc = ttc > responseTime;
                    return new HeadwayInfo(distance, ttc, criticalTtc);
                }
	            
	            // return only headway distance
	            return new HeadwayInfo(distance, Double.POSITIVE_INFINITY, false);
	        }
	    }
	    
	    // default to no collision if GTU is not a LaneBasedGtu or headwayGtu is null
	    return new HeadwayInfo(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, false);
	}
	
	/**
	 * Function to track GTU lane changes in the simulation.
	 * @param: gtuId; ID of lane changing GTU
	 * @param: gtuDirection; Direction of lane change
	 * @param: gtuLink; Network link where lane change was started from
	 * @param: gtuLane; Lane of network link where lane change was started from
	 */
	public void TrackLaneChanges(String gtuId, String gtuDirection, String gtuLink, String gtuLane) {
		if (gtuDirection == "RIGHT" && gtuLane != "FORWARD3") {
			if (gtuLink.equals("AB")) {
				outputDataManager.increaseSingleCount("laneChangesToRightBeforeRamp");
			}
			else if (gtuLink.equals("BC")) {
				outputDataManager.increaseSingleCount("laneChangesToRightOnRamp");
			}
			else if (gtuLink.equals("CD")) {
				outputDataManager.increaseSingleCount("laneChangesToRightAfterRamp");
			}
		}
		else if (gtuDirection == "LEFT" && gtuLane != "FORWARD3") {
			if (gtuLink.equals("AB")) {
				outputDataManager.increaseSingleCount("laneChangesToLeftBeforeRamp");
			}
			else if (gtuLink.equals("BC")) {
				outputDataManager.increaseSingleCount("laneChangesToLeftOnRamp");
			}
			else if (gtuLink.equals("CD")) {
				outputDataManager.increaseSingleCount("laneChangesToLeftAfterRamp");
			}
		}
	}
	
	/**
     * Method to calculate travel time of GTUs removed from the simulation.
     * @param gtuId String;
     */
	private void calculateRemovedGtuTravelTime(Gtu gtu, String gtuId) {
		
		// calculate travel time
		double addTime = gtuAddTimesMap.get(gtuId);
		double removeTime = gtuRemoveTimesMap.get(gtuId);
		double travelTime = removeTime - addTime;
		
		// get travelled distance
		// travel time per distance (traveltime/distance) gives insight into the mean time
		// the vehicle did on a specific distance, but this is just the inverse of the mean speed
		// double distance = gtu.getOdometer().getSI();
		
		// save value
		outputDataManager.addToMeanList("meanTravelTime", travelTime);
		
		// this GTU is removed from the simulation, so can also be removed from the maps
		gtuAddTimesMap.remove(gtuId);
		gtuRemoveTimesMap.remove(gtuId);
		
	}
	
	/**
     * Method to calculate Fundamental Diagram values for specific lanes.
     * @param updateInterval Duration;
     */
    @SuppressWarnings("unused")
    private void createLaneSpecifiFdValues(Duration updateInterval)
    {	
    	// create two data lanes
    	GraphPath<LaneDataRoad> path0;
    	GraphPath<LaneDataRoad> path1;
		try {
			// get left lane after merge ramp
			Lane start0 = ((CrossSectionLink) this.network.getLink("CD")).getLanes().get(0);
			path0 = GraphLaneUtil.createPath("Left lane", start0);
			//get right lane after merge ramp
			Lane start1 = ((CrossSectionLink) this.network.getLink("CD")).getLanes().get(1);
			path1 = GraphLaneUtil.createPath("Right lane", start1);
			
			// list all lanes (required for stopLaneRecording() method)
			this.allLanes = new ArrayList<>();
		} catch (NetworkException exception) {
			throw new RuntimeException("Could not create a path as a lane has no set speed limit.", exception);
		}
		
		// initialise data recording on lanes
		GraphPath.initRecording(this.sampler, path0);
		GraphPath.initRecording(this.sampler, path1);
        
        // create FD sources
		FdSource pathSource0 = FundamentalDiagram.sourceFromSampler(sampler, path0, true, updateInterval);
		FdSource pathSource1 = FundamentalDiagram.sourceFromSampler(sampler, path1, true, updateInterval);
		
		// combine sources of separate lanes into one FD source
		Map<String, FdSource> sourceMap = new LinkedHashMap<>();
		sourceMap.put("Source0", pathSource0);
		sourceMap.put("Source1", pathSource1);
		this.source = FundamentalDiagram.combinedSource(sourceMap);
		
    }
    		
	/**
     * Method to calculate Fundamental Diagram values for whole network.
     * @param updateInterval Duration;
     */
    private void createNetworkFdValues(Duration updateInterval)
    {	
    	// collect all network links
        List<CrossSectionLink> allLinks = new ArrayList<>();
        ImmutableCollection<Link> links = this.network.getLinkMap().values();
        for (Link link : links) {
            allLinks.add((CrossSectionLink) link);
        }
        
        // collect all lanes in the network
        this.allLanes = new ArrayList<>();
        for (CrossSectionLink link : allLinks) {
            for (Lane lane : link.getLanes()) {
                allLanes.add(new LaneDataRoad(lane));
            }
        }

        // create sections list containing all network lanes with corresponding data
        // also initialise data recording for all lanes
        List<Section<LaneDataRoad>> sections = new ArrayList<>();
        for (LaneDataRoad laneData : allLanes) {
            this.sampler.initRecording(laneData);

            Length laneLength = laneData.getLength(); // get lane length
            Speed laneSpeedLimit = new Speed(speedLimit, SpeedUnit.KM_PER_HOUR); // set speed limit for this lane
            
            List<LaneDataRoad> laneList = new ArrayList<>(); // create list of this lane because Section<>() expects a list, not a single lane
            laneList.add(laneData);

            sections.add(new Section<LaneDataRoad>(laneLength, laneSpeedLimit, laneList));
        }

        // create a GraphPath for all lanes
        GraphPath<LaneDataRoad> networkPath = new GraphPath<LaneDataRoad>("Network", sections);

        // create an FD source for the entire network
        this.source = FundamentalDiagram.sourceFromSampler(sampler, networkPath, true, updateInterval);
    }
    
    /*
     * Method to recalculate FD values at fixed frequency.
     */
    private void recalculateFdValues() {
    	
    	// recalculate values
        this.source.recalculate(this.simulator.getSimulatorAbsTime());
        
        // only schedule new calculation when it happens within simulation time
        // stop sampler lane data recording is necessary to make the last calculation reproducible for network recording
        Duration updateInterval = this.source.getUpdateInterval();
        double currentTime = this.simulator.getSimulatorTime().getSI();
        double nextCalculationTime = currentTime + updateInterval.getSI();
        if (nextCalculationTime > simTime.getSI()) {
        	stopLaneRecording();
        }
        else {
        	// schedule recalculation for every data update interval
            this.simulator.scheduleEventRel(updateInterval, this::recalculateFdValues);
        }
    	
    }
    
    /*
     * Method to stop the sampler's lane recording for all listed network lanes.
     */
    private void stopLaneRecording() {
    	// stop recording on the sampler for the entire network, only if lanes are specified
    	if (!this.allLanes.isEmpty()) {
        	System.out.println("Stopped sampler lane recording");
    		for (LaneDataRoad laneData : this.allLanes) {
            	this.sampler.stopRecording(laneData);
            }
    	}
    }
    
    /*
     * Method to save calculated FD values.
     */
    private void saveFdValues()
    {
        // get last calculated index
        int latestIndex = this.source.getItemCount(0);
        if (latestIndex > 0) {
        	latestIndex -= 1;
        }
        
        // process all calculated values
        for (int i = 0; i < latestIndex; i++) {
        	outputDataManager.addToMeanList("meanSpeed", this.source.getSpeed(0, i));
        	outputDataManager.addToMeanList("meanDensity", this.source.getDensity(0, i));
        	outputDataManager.addToMeanList("meanFlow", this.source.getFlow(0, i));
        }
    }
    
	
	/**
     * Class for output data.
     */
    public class OutputDataManager
    {
    	/** Map of values for each sequence output parameter. */
    	private Map<String, GtuSequenceData> sequenceOutputMap = new LinkedHashMap<>();
        
        /** Map of values for each constant output parameter. */
        private Map<String, Object> singleOutputMap = new LinkedHashMap<>();
        
        /** Map to temporarily store values to calculate the mean. */
        private Map<String, ArrayList<Double>> meanMap = new LinkedHashMap<>();
        
        
        /** Function to retrieve Map of sequence output data. */
        public Map<String, GtuSequenceData> getSequenceOutputData() {
        	return sequenceOutputMap;
        }
        
        /** Function to retrieve Map of single output data. */
        public Map<String, Object> getSingleOutputData() {
        	return singleOutputMap;
        }
        
        /** Function to retrieve Map of values used for mean calculations. */
        public Map<String, ArrayList<Double>> getValuesForMeanCalculationData() {
        	return meanMap;
        }
        
        /** Lists to store trajectory data. */
        public ArrayList<String> trajectoryIds = new ArrayList<String>();
        public ArrayList<String> trajectoryLink = new ArrayList<String>();
        public ArrayList<String> trajectoryLane = new ArrayList<String>();
        public ArrayList<String> trajectoryTime = new ArrayList<String>();
        public ArrayList<String> trajectoryDistance = new ArrayList<String>();
        public ArrayList<String> trajectorySpeed = new ArrayList<String>();
        
        /** Export output data. */
        public void exportToCsv(String singleFilePath, String sequenceFilePath, String trajectoryFilePath) {
        	
        	// calculate mean values for the variables stored in the meanMap and save them as single output (not sequence)
        	calculateAndSaveMeans();
        	
        	// create export object and perform export functions
        	ExportSimulationOutput exporter = new ExportSimulationOutput();
        	exporter.exportSingleToCsv(singleOutputMap, singleFilePath);
        	exporter.exportIntermediateMeanValuesToCsv(meanMap , intermediateMeanValuesFilePath);
        	exporter.exportSequenceToCsv(sequenceOutputMap, sequenceFilePath);
        	exporter.exportTrajectoriesToCsv(trajectoryIds, trajectoryLink, trajectoryLane, trajectoryTime,
        									 trajectoryDistance, trajectorySpeed, trajectoryFilePath);
        	
        	// temp printing values
        	for (String key : singleOutputMap.keySet()) {
        		System.out.println(key + ": " + singleOutputMap.get(key));
        	}
        }
        
        /** Function to add a value to a list that is meant for post simulation calculation of the mean value */
        public void addToMeanList(String paramString, Double paramValue) {
        	
        	// check whether this parameter string already exists
        	boolean exists = meanMap.containsKey(paramString);
        	// exists? then, add value to the existing list
        	if (exists) {
        		this.meanMap.get(paramString).add(paramValue);
        	}
        	// does not exist yet? then, create new list for this first value and add it to the map
        	else {
        		ArrayList<Double> firstValueList = new ArrayList<Double>();
        		firstValueList.add(paramValue);
        		this.meanMap.put(paramString, firstValueList);
        	}
        }
        
        /** Function to add data to sequence map. Meant to store detailed data at simulation time with gtuID*/
        public void addSequenceData(String gtuId, double time, String variable, Object value) {
            GtuSequenceData gtuData = sequenceOutputMap.getOrDefault(gtuId, new GtuSequenceData());
            gtuData.addDataPoint(time, variable, value);
            sequenceOutputMap.put(gtuId, gtuData);
        }
        
        /** Function to set a constant output value. */
        public void setSingleValue(String paramString, Object paramValue) {
        	singleOutputMap.put(paramString, paramValue);
        }
        
        /** Function increase a constant count. */
        public void increaseSingleCount(String paramString) {
        	
        	// check whether this parameter string already exists
        	boolean exists = singleOutputMap.containsKey(paramString);
        	// exists? then, increase existing value
        	if (exists) {
        		try {
        			int newValue = (int) singleOutputMap.get(paramString) + 1;
    	        	singleOutputMap.put(paramString, newValue);      		
    	        } 
        		catch (Exception exception) {
        			exception.printStackTrace();
        		}
        	}
        	// does not exist? then, create new constant and start counting
        	else {
        		int value = 1;
        		singleOutputMap.put(paramString, value);
        	}
        }
        
        /** Function increase a constant count. */
        public void decreaseSingleCount(String paramString) {
        	
        	// check whether this parameter string already exists
        	boolean exists = singleOutputMap.containsKey(paramString);
        	// exists? then, increase existing value
        	if (exists) {
        		try {
        			int newValue = (int) singleOutputMap.get(paramString) - 1;
    	        	singleOutputMap.put(paramString, newValue);
        		} 
        		catch (Exception exception) {
        			exception.printStackTrace();
        		}
        	}
        	// does not exist? then, create new constant and start counting
        	else {
        		int value = 1;
        		singleOutputMap.put(paramString, value);
        	}
        }
        
        /** Function to calculate the means for variables stored in the mean map. */
        private void calculateAndSaveMeans() {
        	
        	// loop through stored variables
        	for (String varName : this.meanMap.keySet()) {
        		// calculate mean value
        		double mean = calculateMeanOfList((ArrayList<Double>) meanMap.get(varName));
        		// save as constant output parameter
        		this.setSingleValue(varName, mean);
        	}
        	
        }
        
        /** Function to calculate the mean of a list of values.
         * @param valueList ArrayList<Double>;
         * */
        private double calculateMeanOfList(ArrayList<Double> valueList) {
        	
        	// get sum of values
        	double sum = 0.0;
        	for (double value : valueList) {
        		sum += value;
        	}
        	
        	// calculate mean
        	double mean = sum / valueList.size();
        	
        	// return mean value
        	return mean;
        }
        
        /** Function to add trajectory data to the corresponding lists. */
        public void saveTrajectory(String gtuId, String link, String lane, double time, double distance, double speed)
        {
        	trajectoryIds.add(gtuId);
        	trajectoryLink.add(link);
        	trajectoryLane.add(lane);
        	trajectoryTime.add(Double.toString(time));
        	trajectoryDistance.add(Double.toString(distance));
        	trajectorySpeed.add(Double.toString(speed));
        }
        
        /** classes to create a storage structure to save sequence data with gtuId and simulation time */
        public class GtuSequenceData {
        	// Map with simulation time as key and {variable, values} as value
            private Map<Double, DataPoint> dataPoints;

            public GtuSequenceData() {
                this.dataPoints = new LinkedHashMap<>();
            }

            public void addDataPoint(double time, String variable, Object value) {
                DataPoint dataPoint = dataPoints.getOrDefault(time, new DataPoint(time));
                dataPoint.addValue(variable, value);
                dataPoints.put(time, dataPoint);
            }

            public Collection<DataPoint> getDataPoints() {
                return dataPoints.values();
            }
        }
        
        public class DataPoint {
            private double time;
            private Map<String, Object> values;

            public DataPoint(double time) {
                this.time = time;
                this.values = new LinkedHashMap<>();
            }

            public double getSimulationTime() {
                return time;
            }

            public Map<String, Object> getValues() {
                return values;
            }

            public void addValue(String key, Object value) {
                this.values.put(key, value);
            }
        }
        
    }
    
    private class HeadwayInfo {
        private double headwayDistance;
        private double ttc;
        private boolean criticalTtc;

        public HeadwayInfo(double headwayDistance, double ttc, boolean criticalTtc) {
            this.headwayDistance = headwayDistance;
            this.ttc = ttc;
            this.criticalTtc = criticalTtc;
        }

        public double getHeadwayDistance() {
            return headwayDistance;
        }

        public double getTtc() {
            return ttc;
        }
        
        public boolean isCriticalTtc() {
            return criticalTtc;
        }

        @Override
        public String toString() {
            return "HeadwayInfo{" +
                   "headwayDistance=" + headwayDistance +
                   ", ttc=" + ttc +
                   '}';
        }
    }

}
