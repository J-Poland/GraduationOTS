package sim.demo;

import java.net.URL;
import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.djunits.unit.DurationUnit;
import org.djunits.unit.FrequencyUnit;
import org.djunits.unit.SpeedUnit;
import org.djunits.unit.TimeUnit;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
import org.djunits.value.vdouble.scalar.Time;
import org.djunits.value.vdouble.vector.FrequencyVector;
import org.djunits.value.vdouble.vector.TimeVector;
import org.djutils.event.Event;
import org.djutils.event.EventListener;
import org.djutils.immutablecollections.ImmutableCollection;
import org.djutils.io.URLResource;
import org.djutils.logger.CategoryLogger;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterSet;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.perception.DirectEgoPerception;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.Network;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.draw.graphs.FundamentalDiagram;
import org.opentrafficsim.draw.graphs.FundamentalDiagram.FdSource;
import org.opentrafficsim.draw.graphs.GraphPath;
import org.opentrafficsim.draw.graphs.GraphPath.Section;
import org.opentrafficsim.road.definitions.DefaultsRoadNl;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions.LaneBias;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions.LaneBiases;
import org.opentrafficsim.road.gtu.generator.characteristics.DefaultLaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuCharacteristicsGeneratorOd;
import org.opentrafficsim.road.gtu.lane.AbstractLaneBasedMoveChecker;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.perception.CategoricalLanePerception;
import org.opentrafficsim.road.gtu.lane.perception.LanePerception;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionFactory;
import org.opentrafficsim.road.gtu.lane.perception.RelativeLane;
import org.opentrafficsim.road.gtu.lane.perception.PerceptionCollectable.UnderlyingDistance;
import org.opentrafficsim.road.gtu.lane.perception.categories.AnticipationTrafficPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.DirectInfrastructurePerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Anticipation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.DirectNeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.Estimation;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.NeighborsPerception;
import org.opentrafficsim.road.gtu.lane.perception.categories.neighbors.HeadwayGtuType.PerceivedHeadwayGtuType;
import org.opentrafficsim.road.gtu.lane.perception.headway.HeadwayGtu;
import org.opentrafficsim.road.gtu.lane.perception.mental.AdaptationSituationalAwareness;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller;
import org.opentrafficsim.road.gtu.lane.perception.mental.Fuller.BehavioralAdaptation;
import org.opentrafficsim.road.gtu.lane.perception.mental.Task;
import org.opentrafficsim.road.gtu.lane.perception.mental.TaskManager;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdmFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModelFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlus;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveKeep;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveRoute;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSocioSpeed;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.IncentiveSpeedWithCourtesy;
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
import org.opentrafficsim.road.network.sampling.LaneDataRoad;
import org.opentrafficsim.road.network.sampling.RoadSampler;
import org.opentrafficsim.road.od.Categorization;
import org.opentrafficsim.road.od.Category;
import org.opentrafficsim.road.od.Interpolation;
import org.opentrafficsim.road.od.OdApplier;
import org.opentrafficsim.road.od.OdMatrix;
import org.opentrafficsim.road.od.OdOptions;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.simulators.SimulatorInterface;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import sim.demo.lmrs.CustomCooperation;
import sim.demo.lmrs.CustomTailgating;
import sim.demo.lmrs.SocioDesiredSpeed;
import sim.demo.mental.TaskDriverDistraction;
import sim.demo.mental.CustomAdaptationHeadway;
import sim.demo.mental.CustomAdaptationSituationalAwareness;
import sim.demo.mental.CustomAdaptationSpeed;
import sim.demo.mental.TaskCarFollowing;
import sim.demo.mental.TaskLaneChange;
import sim.demo.mental.TaskManagerAr;
import sim.demo.vehicleconfigurations.VehicleAutomationConfigurations;


/**
 * This demo exists to show how the human factor models can be used in code. In particular see the
 * {@code HumanFactorsModel.constructModel()} method. The included human factors are 1) social interactions regarding lane
 * changes, tailgating and changes in speed, and 2) Anticipation Reliance in a mental task load framework of imperfect
 * perception. The scenario includes a distraction halfway on the network.
 * <p>
 * Copyright (c) 2024-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 * @see HumanFactorsModel#buildHumanFactorsModel()
 * @see <a href="https://www.preprints.org/manuscript/202305.0193/v1">Schakel et al. (2023) Social Interactions on Multi-Lane
 *      Motorways: Towards a Theory of Impacts</a>
 * @see <a href="https://www.sciencedirect.com/science/article/pii/S0191261520303714">Calvert et al. (2020) A generic
 *      multi-scale framework for microscopic traffic simulation part II – Anticipation Reliance as compensation mechanism for
 *      potential task overload</a>
 */
public class VehicleAutomationModel extends AbstractOtsModel implements EventListener
{

    /** */
    private static final long serialVersionUID = 20241012L;

    /** The network. */
    private RoadNetwork network;
	
	/** Sampler. */
    private RoadSampler sampler;

    /** Characteristics generator. */
    private LaneBasedGtuCharacteristicsGeneratorOd characteristics;
    
    /** Sources. */
    private FdSource source;
    private LinkedHashMap<String, FdSource> individualSources;
    
    /** All data lanes within the network. */
    private List<LaneDataRoad> allLanes;
	
	/** Output data manager */
	private OutputDataManager outputDataManager;
    
    /** Maps for GTU add and remove times */
    private LinkedHashMap<String, Double> gtuAddTimesMap;
    private LinkedHashMap<String, Double> gtuRemoveTimesMap;
    
    /** Maps for GTU add and remove times specifically for main and on-ramp lanes. */
    private LinkedHashMap<String, Double> mainGtuAddTimesMap;
    private LinkedHashMap<String, Double> mainGtuRemoveTimesMap;
    private LinkedHashMap<String, Double> rampGtuAddTimesMap;
    private LinkedHashMap<String, Double> rampGtuRemoveTimesMap;
    
    /** Number of FD value calculations. */
    private int calculations = 0;
    
    
    // static simulation settings
	
 	/** Network. */
 	// Rijkswaterstaat ROA Chapter 6 contains standards on turbulence regions (figure 6.6)
 	// also the merge lane lengths are defined (table 6.9)
 	static final String networkString = "twoLaneFreewayWithOnRamp";
 	
 	/** Speed limit and speed variation. */
 	static final double speedLimit = 130.0;
 	static final double speedMin = 120.0;
 	static final double speedMax = 140.0;
 	
 	/** Vehicle max acceleration and max deceleration. */
 	static final double maxAcceleration = 3.0;
 	//TODO: values!
 	static final double minAcceleration = -8.0;
 	
 	/** Synchronisation. */
 	static final Synchronization synchronizationMethod = Synchronization.PASSIVE;

 	/** Cooperation. */
 	static final Cooperation cooperationMethod = CustomCooperation.PASSIVE;
 	
 	/** GapAcceptance. */
 	static final GapAcceptance gapAcceptanceMethod= GapAcceptance.INFORMED;
 	
 	/** Tailgating. */
 	static final Tailgating tailgatingMethod = CustomTailgating.PRESSURE_HUMAN;
 	
 	/** Estimation. */
 	static final Estimation estimationMethod = Estimation.FACTOR_ESTIMATION;
 	
 	/** Anticipation. */
 	static final Anticipation anticipationMethod = Anticipation.CONSTANT_SPEED;
 	
    
 	// input parameters
 	
 	/** Vehicle configuration object. */
 	public VehicleAutomationConfigurations vehicleConfig;
 	
 	/** Simulation configuration object. */
 	public VehicleAutomationModelParameters simConfig;
 	
 	/** Generator seed. */
 	static long seed;
 	
 	/** StreamInterface to control randomness. */
 	public StreamInterface stream;
 	
 	/** Time variables for simulation runtime. */
 	static Time warmUpTime;
 	static Time sampleTime;
 	static Time simTime;
 	
 	/** Gtu type fractions in traffic. */
 	static Map<String, Double> gtuFractions;

 	/** Main demand per lane. Default = 1000 */
 	static double mainDemand;

 	/** Ramp demand. Default = 500 */
 	static double rampDemand;
 	
 	/** Distraction settings */
 	static boolean inVehicleDistractionEnabled;
 	static boolean roadSideDistractionEnabled;
 	
 	/** Variable for sensitivity analysis. */
 	static double sensitivityAnalysisValue;
 	
 	/** File path for output data. */
 	static String outputFolderPath;
 	static String inputValuesFileName;
 	static String singleOutputFileName;
 	static String intermediateMeanValuesFileName;
 	static String sequenceOutputFileName;
 	static String laneChangeOutputFileName;
 	static String collisionOutputFileName;

    /**
     * Constructor.
     * @param simulator simulator
     */
    public VehicleAutomationModel(final OtsSimulatorInterface simulator, VehicleAutomationModelParameters simConfig)
    {
        super(simulator);
        
        this.simConfig = simConfig;
    }
    
    /** {@inheritDoc} */
    @Override
    public void constructModel() throws SimRuntimeException
    {
        try
        {
 			// set simulation configuration
 			setSimParameters();
 			
        	// create output manager and save input parameters
 			outputDataManager = new OutputDataManager(outputFolderPath, sequenceOutputFileName);
 			saveInputParameters();
 			
 			// create road network
            loadNetwork();
            
            // load vehicle automation level GTUs with acceleration limits
            vehicleConfig = new VehicleAutomationConfigurations(maxAcceleration, minAcceleration);
            
            // select and configure driving models
            buildVehicleAutomationModel();
            
            // set traffic demand
            setDemand();
            
            // enable event notifications by adding listeners
         	addSubscriptions();
         	
	        // create sampler to collect FD data
			createSampler();
			
			// setup FD source
			Duration aggregationPeriod = Duration.instantiateSI(30.0, DurationUnit.SECOND);
			individualSources = new LinkedHashMap<String, FdSource>();
			createNetworkFdSources(aggregationPeriod);
	        
	        // sample simulation data throughout the simulation
			Duration sampleInterval = Duration.instantiateSI(0.5, DurationUnit.SECOND);
	        sampleData(sampleInterval, false);
	        
	        // detect collisions in network
	        new CollisionDetector(this.network);
	        
	        // initialise count variables
	        outputDataManager.setSingleValue("collisions", 0);
        }
        catch (NetworkException | ParameterException | RemoteException exception)
        {
            throw new SimRuntimeException(exception);
        }
    }

    /** {@inheritDoc} */
    @Override
    public Network getNetwork()
    {
        return this.network;
    }
    
    /** Create a sampler for simulation data. */
	private void createSampler() {
		this.sampler = new RoadSampler(this.network);
	}
    
    /**
	 * Set all required simulation parameters by the simConfig object.
	 */
	public void setSimParameters() {
		// initialise class variables
		gtuAddTimesMap = new LinkedHashMap<String, Double>();
		gtuRemoveTimesMap = new LinkedHashMap<String, Double>();
		mainGtuAddTimesMap = new LinkedHashMap<String, Double>();
		mainGtuRemoveTimesMap = new LinkedHashMap<String, Double>();
		rampGtuAddTimesMap = new LinkedHashMap<String, Double>();
		rampGtuRemoveTimesMap = new LinkedHashMap<String, Double>();
		gtuFractions = new LinkedHashMap<>();
		
		// initialise input variables
		warmUpTime = new Time(simConfig.getWarmUpTime(), TimeUnit.BASE_SECOND);
		sampleTime = new Time(simConfig.getSampleTime(), TimeUnit.BASE_SECOND);
		simTime = new Time(simConfig.getSimTime(), TimeUnit.BASE_SECOND);
		seed = simConfig.getSeed();
		gtuFractions.put("LEVEL0", simConfig.getLevel0Fraction());
		gtuFractions.put("LEVEL1", simConfig.getLevel1Fraction());
		gtuFractions.put("LEVEL2", simConfig.getLevel2Fraction());
		gtuFractions.put("LEVEL3", simConfig.getLevel3Fraction());
		mainDemand = simConfig.getMainDemand();
		rampDemand = simConfig.getRampDemand();
		inVehicleDistractionEnabled = simConfig.getInVehicleDistraction();
		roadSideDistractionEnabled = simConfig.getRoadSideDistraction();
		sensitivityAnalysisValue = simConfig.getSensitivityAnalysisValue();
		
		// file save locations
		outputFolderPath = simConfig.getOutputFolderPath();
		inputValuesFileName = simConfig.getInputValuesFileName();
		singleOutputFileName = simConfig.getSingleOutputFileName();
		intermediateMeanValuesFileName = simConfig.getIntermediateMeanValuesFileName();
		sequenceOutputFileName = simConfig.getSequenceOutputFileName();
		laneChangeOutputFileName = simConfig.getLaneChangeOutputFileName();
		collisionOutputFileName = simConfig.getCollisionOutputFileName();
		
		// use seed
		final long seedAbove0 = seed + 1;
		this.stream = new MersenneTwister(seedAbove0);
	}
	
	/**
	 * Save input parameter values.
	 */
	public void saveInputParameters() {
		// save input values
		outputDataManager.saveInputValue("warm_up_time", Double.toString(simConfig.getWarmUpTime()));
		outputDataManager.saveInputValue("sample_time", Double.toString(simConfig.getSampleTime()));
		outputDataManager.saveInputValue("seed", Long.toString(simConfig.getSeed()));
		outputDataManager.saveInputValue("level0_fraction", Double.toString(simConfig.getLevel0Fraction()));
		outputDataManager.saveInputValue("level1_fraction", Double.toString(simConfig.getLevel1Fraction()));
		outputDataManager.saveInputValue("level2_fraction", Double.toString(simConfig.getLevel2Fraction()));
		outputDataManager.saveInputValue("level3_fraction", Double.toString(simConfig.getLevel3Fraction()));
		outputDataManager.saveInputValue("main_demand", Double.toString(simConfig.getMainDemand()));
		outputDataManager.saveInputValue("ramp_demand", Double.toString(simConfig.getRampDemand()));
		outputDataManager.saveInputValue("in_vehicle_distraction", Boolean.toString(simConfig.getInVehicleDistraction()));
		outputDataManager.saveInputValue("road_side_distraction", Boolean.toString(simConfig.getRoadSideDistraction()));
		outputDataManager.saveInputValue("sensitivity_analysis_value", Double.toString(simConfig.getSensitivityAnalysisValue()));
		
		// export these parameters to a CSV
		outputDataManager.exportInputValues(inputValuesFileName);
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

	/** Method to process notifications from the simulation listener. */
	@Override
	public void notify(Event event) throws RemoteException {

		// handle start of replication event
		if (event.getType().equals(SimulatorInterface.START_EVENT)) {
			System.out.println("\nStart simulation.");
			System.out.println("For a runtime of: " + simTime.si);
		}
		
		// handle end of replication event
		if (event.getType().equals(SimulatorInterface.STOP_EVENT)) {
			System.out.println("Simulation stopped at time: " + this.simulator.getSimulatorAbsTime().si);
			System.out.println("Stop simulation.");
			System.out.println("Calculations: " + calculations);
			System.out.println("Collisions: " + outputDataManager.getSingleCount("collisions"));
			
			// store calculated FD values
			storeFdValues();
			
			// export all collected data (sequence GTU data is automatically exported throughout the simulation)
			outputDataManager.finalExportToCsv();
		}
		
		// handle simulation time changed event
		if (event.getType().equals(SimulatorInterface.TIME_CHANGED_EVENT)) {
			
		}
		
		// handle adding GTUs to the simulation
		if (event.getType().equals(Network.GTU_ADD_EVENT)) {
			String gtuId = (String) event.getContent();
			LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
			
			// add GTU listeners
			gtu.addListener(this, Gtu.MOVE_EVENT);
			gtu.addListener(this, LaneBasedGtu.LANE_CHANGE_EVENT);
			gtu.addListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
			
			// ensure warm-up time is over
			if (this.simulator.getSimulatorAbsTime().si >= warmUpTime.si) {
				// add GTU to gtuAddTimesMap to track GTU travel time
				gtuAddTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().getSI());
			}
		}
		
		// disable subscription to GTU lane events when GTUs leave the simulation
		if (event.getType().equals(Network.GTU_REMOVE_EVENT)) {
			String gtuId = (String) event.getContent();
			LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
			
			// remove GTU listeners
			gtu.removeListener(this, Gtu.MOVE_EVENT);
			gtu.removeListener(this, LaneBasedGtu.LANE_CHANGE_EVENT);
			
			// ensure warm-up time is over
			if (this.simulator.getSimulatorAbsTime().si >= warmUpTime.si) {
				// add GTU to gtuRemoveTimesMap and calculate travelled time
				gtuRemoveTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().si);
				calculateRemovedGtuTravelTime(gtu, gtuId);
				
				// also process main lane travel time if origin node of GTU is "A"
				if (gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.ORIGIN).equals("A")) {
					mainGtuRemoveTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().si);
					calculateMainRemovedGtuTravelTime(gtu, gtuId);
				}
			}
		}
		
		// handle GTU move events
		if (event.getType().equals(Gtu.MOVE_EVENT)) {
			String gtuId = (String) ((Object[]) event.getContent())[0];
	        LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
	        
	        // at GTU's first move event, set distribution parameters according to automation level and driver profile
	        // and set the GTU origin
			if (gtu.getOdometer().equals(Length.instantiateSI(0.0))) {
				try {
					String gtuType = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
					Parameters parametersForThisType = vehicleConfig.drawDistributionParameters(this.stream, gtuType, gtu.getParameters());
					gtu.setParameters(parametersForThisType);
				} catch (ParameterException e) {
					CategoryLogger.always().error("GTU distribution parameters could not be set.");
					e.printStackTrace();
				}
				
				try {
					String origin = gtu.getReferencePosition().lane().getLink().getStartNode().getId().toString();
					gtu.getParameters().setParameter(VehicleAutomationConfigurations.ORIGIN, origin);
					
					if (origin.equals("A")) {
						mainGtuAddTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().si);
					}
					else if (origin.equals("E")) {
						rampGtuAddTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().si);
					}
				} catch (GtuException | ParameterException e) {
					CategoryLogger.always().error("GTU origin parameter could not be set.");
					e.printStackTrace();
				}
			};
		}
		
		if (event.getType().equals(LaneBasedGtu.LANEBASED_MOVE_EVENT)) {
			
		}
		
		// process lane change event
		if (event.getType().equals(LaneBasedGtu.LANE_CHANGE_EVENT)) {
			// get lane change info
			String gtuId = (String) ((Object[]) event.getContent())[0];
			String gtuDirection = (String) ((Object[]) event.getContent())[1];
			String gtuLink = (String) ((Object[]) event.getContent())[2];
			// lane info contains the lane where the GTU departed from
			String gtuFromLane = (String) ((Object[]) event.getContent())[3];
			
			// get gtu type
	        LaneBasedGtu gtu = (LaneBasedGtu) this.network.getGTU(gtuId);
	        String gtuType = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
			
	        // ensure warm-up time is over
	     	if (this.simulator.getSimulatorAbsTime().si >= warmUpTime.si) {
				// save this info
				TrackLaneChanges(gtuId, gtuType, gtuDirection, gtuLink, gtuFromLane);
				
				// calculate travel time for on-ramp generated GTUs
				if (gtuFromLane.equals("ONRAMP") && gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.ORIGIN).equals("E")) {
					rampGtuRemoveTimesMap.put(gtuId, this.simulator.getSimulatorAbsTime().si);
					calculateRampRemovedGtuTravelTime(gtu, gtuId);
				}
	     	}
		}
	}
	
	/**
     * Builds the network, a 3km 2-lane highway section.
     * @throws NetworkException when the network is ill defined
     */
    private void loadNetwork() throws NetworkException
    {
    	try {
	    	// load network from xml file
			URL xmlURL = URLResource.getResource("C:\\Users\\jesse\\Documents\\Java\\TrafficSimulation-workspace\\"
													+ "traffic-sim\\src\\main\\resources\\networks\\" + networkString + ".xml");
			this.network = new RoadNetwork("VehicleAutomationSimulation", getSimulator());
			new XmlParser(this.network).setUrl(xmlURL).build();
    	} catch (Exception e) {
    		e.printStackTrace();
    	}
    }

    /**
     * Builds the human factors model.
     * @throws ParameterException if parameter has no default value
     */
    private void buildVehicleAutomationModel() throws ParameterException
    {
    	// get simulation stream
        StreamInterface stream = this.stream;

        // get parameter factory for vehicle automation level GTUs
        ParameterFactoryByType parameterFactory = vehicleConfig.getVehicleAutomationParameterFactory();

        // car-following model: the normal desired headway model adjusts as a response to social pressure from the following vehicle
        CarFollowingModelFactory<IdmPlus> cfModelFactory = new AbstractIdmFactory<>(new IdmPlus(AbstractIdm.HEADWAY, new SocioDesiredSpeed(AbstractIdm.DESIRED_SPEED)), stream);
        
        // perception factory for mental processes
        PerceptionFactory perceptionFactory = new PerceptionFactory()
        {
            /** {@inheritDoc} */
            @Override
            public Parameters getParameters() throws ParameterException
            {
                /*
                 * We need to include (default) values for 1) the Fuller task demand model of task demand, capacity and
                 * saturation, 2) the anticipation reliance (AR) task manager based on a notion of a primary task and
                 * auxiliary task, 3) situational awareness as it depends on task saturation, 4) car-following task
                 * parameter of exponential relationship with task demand, 5) over or underestimation, and 6) sensitivity of
                 * adapting the headway and desired speed based on high task saturation.
                 */
                ParameterSet perceptionParams = new ParameterSet();
                // these parameters are managed in the VehicleAutomationConfigurations class
                return perceptionParams;
            }

            /** {@inheritDoc} */
            @Override
            public LanePerception generatePerception(final LaneBasedGtu gtu)
            {
                // Tasks that determine task demand
                Set<Task> tasks = new LinkedHashSet<>();
                tasks.add(new TaskCarFollowing());
                tasks.add(new TaskLaneChange());
                // also add driver distraction demand
            	double roadSideDistractionPos = network.getNode("B").getLocation().getX();
            	tasks.add(new TaskDriverDistraction(stream, inVehicleDistractionEnabled,
            			                        roadSideDistractionEnabled, roadSideDistractionPos));
                
                // Behavioral adaptations (AdaptationSituationalAwareness only sets situational awareness and reaction time)
                Set<BehavioralAdaptation> behavioralAdapatations = new LinkedHashSet<>();
                behavioralAdapatations.add(new CustomAdaptationSituationalAwareness());
                behavioralAdapatations.add(new CustomAdaptationHeadway());
                behavioralAdapatations.add(new CustomAdaptationSpeed());
                // AR task manager, assuming lane changing to be primary
                TaskManager taskManager = new TaskManagerAr("lane-changing");
                // Fuller framework based on components
                CategoricalLanePerception perception =
                        new CategoricalLanePerception(gtu, new Fuller(tasks, behavioralAdapatations, taskManager));
                // Imperfect estimation of distance and speed difference, with reaction time, and compensatory anticipation
                HeadwayGtuType headwayGtuType =
                        new PerceivedHeadwayGtuType(estimationMethod, anticipationMethod);
                // Standard perception categories, using imperfect perception regarding neighbors with the HeadwayGtuType
                perception.addPerceptionCategory(new DirectEgoPerception<>(perception));
                perception.addPerceptionCategory(new DirectInfrastructurePerception(perception));
                perception.addPerceptionCategory(new DirectNeighborsPerception(perception, headwayGtuType));
                perception.addPerceptionCategory(new AnticipationTrafficPerception(perception));
                return perception;
            }
        };

        // After 1.7.5, the lmrsFactory works with suppliers of sets of incentives. In 1.7.5 this was just the sets
        // (wrongfully shared among all GTUs).
        Set<MandatoryIncentive> mandatoryIncentives = new LinkedHashSet<>();
        mandatoryIncentives.add(new IncentiveRoute());
        Set<VoluntaryIncentive> voluntaryIncentives = new LinkedHashSet<>();
        voluntaryIncentives.add(new IncentiveSpeedWithCourtesy());
        voluntaryIncentives.add(new IncentiveKeep());
        /*
         * Next to increasing speed, social interactions include a change in lane change desire to get or stay out of the
         * way of faster (potential) followers.
         */
        voluntaryIncentives.add(new IncentiveSocioSpeed());
        
        // register template suppliers for GTUs
        Map<String, GtuType> vehicleAutomationLevelsMap = vehicleConfig.getVehicleAutomationLevelsMap();
        for (GtuType gtuType : vehicleAutomationLevelsMap.values()) {
			GtuType.registerTemplateSupplier(gtuType, vehicleConfig);
		}

        // Layered factories (tactical, strategical, strategical in an OD context)
        LmrsFactory lmrsFactory = new LmrsFactory(cfModelFactory, perceptionFactory,
        		synchronizationMethod, cooperationMethod, gapAcceptanceMethod, tailgatingMethod, 
        		mandatoryIncentives, voluntaryIncentives, new LinkedHashSet<>());
        LaneBasedStrategicalRoutePlannerFactory strategicalPlannerFactory =
                new LaneBasedStrategicalRoutePlannerFactory(lmrsFactory, parameterFactory);
        this.characteristics =
                new DefaultLaneBasedGtuCharacteristicsGeneratorOd.Factory(strategicalPlannerFactory).create();
    }

    /**
     * Set demand in network.
     * @throws SimRuntimeException sim exception
     * @throws ParameterException parameter exception
     */
    private void setDemand() throws SimRuntimeException, ParameterException
    {
    	// get start of the main lanes
        Node nodeA = this.network.getNode("A");
        // get start of the on-ramp
        Node nodeE = this.network.getNode("E");
        // get end of the main lanes
        Node nodeD = this.network.getNode("D");
        
        // list route start and end points
        List<Node> origins = new ArrayList<>();
        origins.add(nodeA);
        origins.add(nodeE);
        List<Node> destinations = new ArrayList<>();
        destinations.add(nodeD);
        
        // setup categorisation for demand by vehicle type
        Categorization categorization = new Categorization("GTU type", GtuType.class);
        
        // create origin-destination (OD) matrix with vehicle demand details
        // a time vector and frequency vector can create a dynamic traffic demand over time, the simulation checks the 
        // demand for every time time point. however, here we want a steady stream throughout the whole simulation.
        TimeVector demandTime = new TimeVector(new double[] {0.0, simTime.si});
        FrequencyVector mainDemandVector = new FrequencyVector(new double[] {mainDemand, mainDemand}, FrequencyUnit.PER_HOUR);
        FrequencyVector rampDemandVector = new FrequencyVector(new double[] {rampDemand, rampDemand}, FrequencyUnit.PER_HOUR);
        OdMatrix od = new OdMatrix("OD", origins, destinations, categorization, demandTime, Interpolation.LINEAR);
        
        // get vehicle types
        Map<String, GtuType> vehicleAutomationLevelsMap = vehicleConfig.getVehicleAutomationLevelsMap();
        
        // set demand for each GTU type
        for (Map.Entry<String, GtuType> entry : vehicleAutomationLevelsMap.entrySet()) {
		    String automationLevel = entry.getKey();
		    GtuType gtuType = entry.getValue();
        	
        	// select fraction of this GTU type
        	double gtuTypeFraction = gtuFractions.get(automationLevel);
        	
        	// set demand for both main lanes and on-ramp
	        od.putDemandVector(nodeA, nodeD, new Category(categorization, gtuType), mainDemandVector, gtuTypeFraction);
	        od.putDemandVector(nodeE, nodeD, new Category(categorization, gtuType), rampDemandVector, gtuTypeFraction);
        }
        
        // OD matrix settings
        OdOptions odOptions = new OdOptions();
        odOptions.set(OdOptions.NO_LC_DIST, Length.instantiateSI(150.0));
        odOptions.set(OdOptions.GTU_TYPE, this.characteristics);
        
        
        // set general lane biases for the network.
        // regarding lane biases, no difference is made between CAR automation levels.
        
        // DefaultsRoadNl.LANE_BIAS_CAR_TRUCK not available in 1.7.5
        // so create biases here
        LaneBiases laneBiases = new LaneBiases();
        laneBiases.addBias(DefaultsNl.CAR, LaneBias.WEAK_LEFT);
        laneBiases.addBias(DefaultsNl.TRUCK, LaneBias.TRUCK_RIGHT);
        
        // add lane biases to OD matrix
        odOptions.set(OdOptions.LANE_BIAS, laneBiases);
        OdApplier.applyOd(this.network, od, odOptions, DefaultsRoadNl.VEHICLES);
    }
    
    /**
     * Method to create sources for network Fundamental Diagram calculations.
     * @param updateInterval Duration;
     */
    private void createNetworkFdSources(Duration aggregationPeriod)
    {	
    	// collect all network links
        List<CrossSectionLink> allLinks = new ArrayList<>();
        ImmutableCollection<Link> links = this.network.getLinkMap().values();
        for (Link link : links) {
            allLinks.add((CrossSectionLink) link);
        }
        
        // collect all lanes within these links
        this.allLanes = new ArrayList<>();
        for (CrossSectionLink link : allLinks) {
            for (Lane lane : link.getLanes()) {
                allLanes.add(new LaneDataRoad(lane));
            }
        }
        
        // create a sections list containing all found lanes and add required length and speed data
        // also initialise data recording for all lanes
        List<Section<LaneDataRoad>> sections = new ArrayList<>();
        for (LaneDataRoad laneData : allLanes) {
            this.sampler.initRecording(laneData);

            Length laneLength = laneData.getLength(); // get lane length
            Speed laneSpeedLimit = new Speed(speedLimit, SpeedUnit.KM_PER_HOUR); // set speed limit for this lane
            
            List<LaneDataRoad> laneList = new ArrayList<>(); // create list of this lane because Section<>() expects a list, not a single lane
            laneList.add(laneData);

			// add this lane to the list of sections
            Section<LaneDataRoad> section = new Section<LaneDataRoad>(laneLength, laneSpeedLimit, laneList);
            sections.add(section);
            
            // also create a FD source specifically for this lane
            List<Section<LaneDataRoad>> individualSectionList = new ArrayList<>();
            individualSectionList.add(section);
            GraphPath<LaneDataRoad> individualGraphPath = new GraphPath<LaneDataRoad>(laneData.getLane().getFullId(), individualSectionList);
            FdSource individualSource = FundamentalDiagram.sourceFromSampler(sampler, individualGraphPath, true, aggregationPeriod);
            this.individualSources.put(laneData.getLane().getFullId(), individualSource);
        }

        this.source = FundamentalDiagram.combinedSource(individualSources);
    }
    
    /** Method to sample simulation data */
	private void sampleData(Duration sampleInterval, boolean headersAlreadyCreated) {
		// do not sample data during warm-up time
		if (this.simulator.getSimulatorAbsTime().si < warmUpTime.si) {
			// reschedule this method for next time step
			this.simulator.scheduleEventRel(sampleInterval, this, "sampleData", new Object[] {sampleInterval, false} );
			return;
		}
		
		// create initial map
		Map<String, String> gtuData = new LinkedHashMap<>();

		// set headers of sample data if not done already
    	if (!headersAlreadyCreated) {
    		gtuData.put("time", "");
        	gtuData.put("gtu_id", "");
        	gtuData.put("gtu_type", "");
        	gtuData.put("acceleration", "");
        	gtuData.put("speed", "");
        	gtuData.put("desired_speed", "");
        	gtuData.put("x_position", "");
        	gtuData.put("turn_indicator", "");
        	gtuData.put("link", "");
        	gtuData.put("lane", "");
        	gtuData.put("leader_gtu_id", "");
        	gtuData.put("leader_gtu_type", "");
        	gtuData.put("headway_distance", "");
        	gtuData.put("headway_time", "");
        	gtuData.put("desired_headway_time", "");
        	gtuData.put("ttc", "");
        	gtuData.put("reaction_time", "");
        	gtuData.put("situational_awareness", "");
        	gtuData.put("task_saturation", "");
        	gtuData.put("cf_task_demand", "");
        	gtuData.put("lc_task_demand", "");
        	gtuData.put("secondary_task_demand", "");
        	gtuData.put("road_side_distraction_demand", "");
    		outputDataManager.createSequenceHeaders(gtuData);
    	}
		
		// get GTU data from all vehicles in the network
		for (Gtu gtu : this.network.getGTUs()) {
			LaneBasedGtu laneBasedGtu = (LaneBasedGtu) gtu;
			Map<String, String> observedGtuData = TrackGtuData(gtuData, laneBasedGtu);
			outputDataManager.addSequenceData(observedGtuData);
		}
		
		// recalculate FD values and stop sampler lane data recording if simulation is finished
		// this is necessary to make the last FD calculation reproducible
        double currentTime = this.simulator.getSimulatorAbsTime().si;
        double nextCalculationTime = currentTime + sampleInterval.si;
        if (nextCalculationTime > simTime.si) {
        	recalculateFdValues();
        	stopLaneRecording();
        }
		
		// reschedule this method for next GTU sample
		this.simulator.scheduleEventRel(sampleInterval, this, "sampleData", new Object[] {sampleInterval, true} );
	}
	
	/**
	 * Function to track GTU info within the simulation.
	 * @param gtu
	 */
	public Map<String, String> TrackGtuData(Map<String, String> gtuData, LaneBasedGtu gtu)
	{
		// main GTU info
    	double time = this.simulator.getSimulatorAbsTime().si;
		String gtuId = gtu.getId();
    	String gtuType = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
        
        // get detailed data
        double acceleration = gtu.getAcceleration().si;
        double speed = gtu.getSpeed().si;
        double desiredSpeed = gtu.getDesiredSpeed().si;
        double xPosition = gtu.getLocation().getX();
        double reactionTime = gtu.getParameters().getParameterOrNull(ParameterTypes.TR).si;
        double situationalAwareness = gtu.getParameters().getParameterOrNull(AdaptationSituationalAwareness.SA);
        double taskSaturation = gtu.getParameters().getParameterOrNull(Fuller.TS);
        double carFollowingTaskDemand = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.CF_TASK_DEMAND);
        double laneChangingTaskDemand = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.LC_TASK_DEMAND);
        boolean secondaryTaskDemand = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.SECONDARY_TASK_DISTRACTED);
        boolean roadSideDistractionDemand = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.ROAD_SIDE_DISTRACTED);
        
        // get lane and link
        String lane = null;
        String link = null;
		try {
			lane = gtu.getReferencePosition().lane().getId().toString();
			link = gtu.getReferencePosition().lane().getLink().getId().toString();
		} catch (GtuException e) {
			e.printStackTrace();
		}
		
		// get GTU turn state
		String turnIndicator = gtu.getTurnIndicatorStatus().toString();;
		
		// get headway info
		HeadwayInfo headwayInfo = new HeadwayInfo(gtu);
    	double desiredHeadwayTime = gtu.getParameters().getParameterOrNull(ParameterTypes.T).si;
    	
    	// collect observed and calculated data
    	gtuData.put("time", Double.toString(time));
    	gtuData.put("gtu_id", gtuId);
    	gtuData.put("gtu_type", gtuType);
    	gtuData.put("acceleration", Double.toString(acceleration));
    	gtuData.put("speed", Double.toString(speed));
    	gtuData.put("desired_speed", Double.toString(desiredSpeed));
    	gtuData.put("x_position", Double.toString(xPosition));
    	gtuData.put("turn_indicator", turnIndicator);
    	gtuData.put("link", link);
    	gtuData.put("lane", lane);
    	gtuData.put("leader_gtu_id", headwayInfo.getLeaderGtuId());
    	gtuData.put("leader_gtu_type", headwayInfo.getLeaderGtuType());
    	gtuData.put("headway_distance", Double.toString(headwayInfo.getHeadwayDistance()));
    	gtuData.put("headway_time", Double.toString(headwayInfo.getHeadwayTime()));
    	gtuData.put("desired_headway_time", Double.toString(desiredHeadwayTime));
    	gtuData.put("ttc", Double.toString(headwayInfo.getTtc()));
    	gtuData.put("reaction_time", Double.toString(reactionTime));
    	gtuData.put("situational_awareness", Double.toString(situationalAwareness));
    	gtuData.put("task_saturation", Double.toString(taskSaturation));
    	gtuData.put("cf_task_demand", Double.toString(carFollowingTaskDemand));
    	gtuData.put("lc_task_demand", Double.toString(laneChangingTaskDemand));
    	gtuData.put("secondary_task_demand", Boolean.toString(secondaryTaskDemand));
    	gtuData.put("road_side_distraction_demand", Boolean.toString(roadSideDistractionDemand));
    	
    	// return data from this GTU
    	return gtuData;
	}
    
    /**
	 * Function to track GTU lane changes in the simulation.
	 * @param: gtuId; ID of lane changing GTU
	 * @param: gtuDirection; Direction of lane change
	 * @param: gtuLink; Network link where lane change was started from
	 * @param: gtuLane; Lane of network link where lane change was started from
	 */
	public void TrackLaneChanges(String gtuId, String gtuType, String gtuDirection, String gtuLink, String gtuFromLane) {
		double time = this.simulator.getSimulatorAbsTime().si;
		outputDataManager.saveLaneChange(time, gtuId, gtuType, gtuDirection, gtuLink, gtuFromLane);
	}
	
	/**
	 * Function to track GTU collisions in the simulation.
	 * @param: gtuId; ID of lane changing GTU
	 * @param: gtuDirection; Direction of lane change
	 * @param: gtuLink; Network link where lane change was started from
	 * @param: gtuLane; Lane of network link where lane change was started from
	 */
	public void TrackCollisions(String gtuId, String gtuType, String gtuLeaderId, String gtuLeaderType) {
		double time = this.simulator.getSimulatorAbsTime().si;
		outputDataManager.saveCollision(time, gtuId, gtuType, gtuLeaderId, gtuLeaderType);
	}
	
	/**
     * Method to calculate travel time of GTUs removed from the simulation.
     * @param gtuId String;
     */
	private void calculateRemovedGtuTravelTime(Gtu gtu, String gtuId) {
		
		// because of the warm-up time, a GTU can be registered as removed from the network
		// but not be present in the gtuAddTimesMap. so, first check whether this GTU is registered in the add map.
		if (!gtuAddTimesMap.containsKey(gtuId)) {
			gtuRemoveTimesMap.remove(gtuId);
			return;
		}
		
		// calculate travel time
		double addTime = gtuAddTimesMap.get(gtuId);
		double removeTime = gtuRemoveTimesMap.get(gtuId);
		double travelTime = removeTime - addTime;
		
		// save value
		outputDataManager.addToMeanList("travel_time", travelTime);
		
		// this GTU is removed from the simulation, so can also be removed from the maps
		gtuAddTimesMap.remove(gtuId);
		gtuRemoveTimesMap.remove(gtuId);
	}
	
	/**
     * Method to calculate travel time of GTUs removed from the simulation specifically for main lanes.
     * @param gtuId String;
     */
	private void calculateMainRemovedGtuTravelTime(Gtu gtu, String gtuId) {
		
		// because of the warm-up time, a GTU can be registered as removed from the network
		// but not be present in the gtuAddTimesMap. so, first check whether this GTU is registered in the add map.
		if (!mainGtuAddTimesMap.containsKey(gtuId)) {
			mainGtuRemoveTimesMap.remove(gtuId);
			return;
		}
		
		// calculate travel time
		double addTime = mainGtuAddTimesMap.get(gtuId);
		double removeTime = mainGtuRemoveTimesMap.get(gtuId);
		double travelTime = removeTime - addTime;
		
		// save value
		outputDataManager.addToMeanList("main_travel_time", travelTime);
		
		// this GTU is removed from the simulation, so can also be removed from the maps
		mainGtuAddTimesMap.remove(gtuId);
		mainGtuRemoveTimesMap.remove(gtuId);
	}
	
	/**
     * Method to calculate travel time of GTUs removed from the simulation specifically for the on-ramp.
     * @param gtuId String;
     */
	private void calculateRampRemovedGtuTravelTime(Gtu gtu, String gtuId) {
		
		// because of the warm-up time, a GTU can be registered as removed from the network
		// but not be present in the gtuAddTimesMap. so, first check whether this GTU is registered in the add map.
		// in this case, a GTU can be registered as removed from the ramp when it changes lanes from the on-ramp for the
		// second time. because of this check the second lane change will not contribute to the travel time.
		if (!rampGtuAddTimesMap.containsKey(gtuId)) {
			rampGtuRemoveTimesMap.remove(gtuId);
			return;
		}
		
		// calculate travel time
		double addTime = rampGtuAddTimesMap.get(gtuId);
		double removeTime = rampGtuRemoveTimesMap.get(gtuId);
		double travelTime = removeTime - addTime;
		
		// save value
		outputDataManager.addToMeanList("ramp_travel_time", travelTime);
		
		// this GTU is removed from the simulation, so can also be removed from the maps
		rampGtuAddTimesMap.remove(gtuId);
		rampGtuRemoveTimesMap.remove(gtuId);
	}
	
	/*
     * Method to recalculate FD values at fixed frequency.
     */
    private void recalculateFdValues() {
    	// recalculate FD values
        this.source.recalculate(this.simulator.getSimulatorAbsTime());
        this.calculations += 1;
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
    private void storeFdValues()
    {
    	// get individual sources from the combined FD source
    	// each individual source has a Series of data in the combined source
    	int numberOfSources = this.source.getNumberOfSeries();
    	
    	// loop through them and save their calculated values for speed, density and flow
    	for (int sourceNumber = 0; sourceNumber < numberOfSources; sourceNumber++) {
    		// get number of values calculated in a source
	        int latestIndex = this.source.getItemCount(sourceNumber);
	        if (latestIndex > 0) {
	        	latestIndex -= 1;
	        }
	        // process all calculated values after the warm-up time
	        for (int i = 0; i < latestIndex; i++) {
	        	double aggregationPeriod = this.source.getAggregationPeriod().si;
	        	double timeOfCalculation = aggregationPeriod * i;
	        	if (timeOfCalculation < warmUpTime.si) {
	        		continue; // go to next calculated value
	        	}
	        	
	        	outputDataManager.addToMeanList((this.source.getName(sourceNumber) + "_speed"), this.source.getSpeed(sourceNumber, i));
	        	outputDataManager.addToMeanList((this.source.getName(sourceNumber) + "_density"), this.source.getDensity(sourceNumber, i));
	        	outputDataManager.addToMeanList((this.source.getName(sourceNumber) + "_flow"), this.source.getFlow(sourceNumber, i));
	        }
    	}
    }
    
    
    /**
     * Class to manage simulation output data.
     */
    public class OutputDataManager
    {
    	/** Exporter. */
    	ExportSimulationOutput exporter = null;
    	
    	/** Map of input parameter values. */
    	private Map<String, String> inputValuesMap = new LinkedHashMap<>();
        
        /** Map of values for each constant output parameter. */
        private Map<String, Object> singleOutputMap = new LinkedHashMap<>();
        
        /** Map to temporarily store values to calculate the mean. */
        private Map<String, ArrayList<Double>> meanMap = new LinkedHashMap<>();
        
        /** Lists to store lane change data. */
        public ArrayList<String> laneChangeTime = new ArrayList<String>();
        public ArrayList<String> laneChangeIds = new ArrayList<String>();
        public ArrayList<String> laneChangeTypes = new ArrayList<String>();
        public ArrayList<String> laneChangeDirections = new ArrayList<String>();
        public ArrayList<String> laneChangeLinks = new ArrayList<String>();
        public ArrayList<String> laneChangeFromLanes = new ArrayList<String>();
        
        /** Lists to store collision data. */
        public ArrayList<String> collisionTime = new ArrayList<String>();
        public ArrayList<String> collisionIds = new ArrayList<String>();
        public ArrayList<String> collisionTypes = new ArrayList<String>();
        public ArrayList<String> collisionLeaderIds = new ArrayList<String>();
        public ArrayList<String> collisionLeaderTypes = new ArrayList<String>();
        
        
        /** Constructor */
        public OutputDataManager(String outputFolderPath, String sequenceFileName) {
        	// create export object
        	exporter = new ExportSimulationOutput(outputFolderPath, inputValuesFileName, sequenceFileName, singleOutputFileName,
        										  intermediateMeanValuesFileName, laneChangeOutputFileName, collisionOutputFileName);
        }
        
        
        /** Export additional output data at end of the simulation. */
        public void finalExportToCsv() {
        	
        	// calculate mean values for the variables stored in the meanMap and save them as single output (not sequence)
        	calculateAndSaveMeans();
        	
        	// perform export of mean, intermediate mean, and lane change data
        	try {
	        	exporter.exportSingleToCsv(singleOutputMap, singleOutputFileName);
	        	exporter.exportIntermediateMeanValuesToCsv(meanMap , intermediateMeanValuesFileName);
	        	exporter.exportLaneChangesToCsv(laneChangeTime, laneChangeIds, laneChangeTypes, laneChangeDirections,
	        									laneChangeLinks, laneChangeFromLanes, laneChangeOutputFileName);
	        	exporter.exportCollisionsToCsv(collisionTime, collisionIds, collisionTypes, 
	        			                        collisionLeaderIds, collisionLeaderTypes, collisionOutputFileName);
        	} catch (NullPointerException e) {
        		e.printStackTrace();
        	}
        	
        	// sequence data is already saved during the simulation
        	// output streams are closed when program shuts down, see ExportSimulationOutput class
        }
        
        /** Function to retrieve Map of single output data. */
        public Map<String, Object> getSingleOutputData() {
        	return singleOutputMap;
        }
        
        /** Function to retrieve Map of values used for mean calculations. */
        public Map<String, ArrayList<Double>> getValuesForMeanCalculationData() {
        	return meanMap;
        }
        
        /** Function to write headers for the sequence data file. */
        public void createSequenceHeaders(Map<String, String> dataRow) {
        	ArrayList<String> headers = new ArrayList<String>();
        	
        	// get all variable names
        	for (String variable : dataRow.keySet()) {
        		headers.add(variable);
        	}
        	
        	// write headers for sequence data file
        	exporter.writeSequenceDataHeaders(headers);
        }
        
        /** Function to save a sequence data row. */
        public void addSequenceData(Map<String, String> dataRow) {
        	// write new dataRow to file
        	exporter.writeSequenceDataRow(dataRow);
        }
        
        /** Save input parameter values */
        public void saveInputValue(String paramString, String paramValue) {
        	inputValuesMap.put(paramString, paramValue);
        }
        
        /**
         * Export saved input parameter values.
         * @param inputFilePath
         */
        public void exportInputValues(String inputFilePath) {
        	exporter.exportInputToCsv(inputValuesMap, inputFilePath);
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
        
        /** Function to set a constant output value. */
        public void setSingleValue(String paramString, Object paramValue) {
        	singleOutputMap.put(paramString, paramValue);
        }
        
        /** Function to retrieve count value. */
        public int getSingleCount(String paramString) {
        	return (int) singleOutputMap.get(paramString);
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
        	// does not exist? then, create new constant and start counting from 1
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
        		// get values
        		ArrayList<Double> varValues = meanMap.get(varName);
        		// calculate mean value if values are available
        		if (varValues.size() > 0) {
        			double mean = calculateMeanOfList(varValues);
        			// save as constant output parameter
            		this.setSingleValue(varName, mean);
        		}
        		else {
        			String mean = "No values";
        			// save as constant output parameter
            		this.setSingleValue(varName, mean);
        		}
        	}
        	
        }
        
        /** Function to calculate the mean of a list of values.
         * @param valueList ArrayList<Double>;
         * */
        private double calculateMeanOfList(ArrayList<Double> valueList) {
        	
        	// get sum of values
        	double sum = 0.0;
        	for (double value : valueList) {
        		// do not take NaN values into account, this will result in a NaN sum
        		if (Double.isNaN(value)) {
                	continue;
        		}
        		sum += value;
        	}
        	
        	// calculate mean
        	double mean = sum / valueList.size();
        	
        	// return mean value
        	return mean;
        }
        
        /** Function to add lane change data to the corresponding lists */
        public void saveLaneChange(double time, String gtuId, String gtuType, String gtuDirection, String gtuLink, String gtuFromLane)
        {
        	laneChangeTime.add(Double.toString(time));
        	laneChangeIds.add(gtuId);
        	laneChangeTypes.add(gtuType);
        	laneChangeDirections.add(gtuDirection);
        	laneChangeLinks.add(gtuLink);
        	laneChangeFromLanes.add(gtuFromLane);
        }
        
        /** Function to add collision data to the corresponding lists */
        public void saveCollision(double time, String gtuId, String gtuType, String gtuLeaderId, String gtuLeaderType)
        {
        	collisionTime.add(Double.toString(time));
        	collisionIds.add(gtuId);
        	collisionTypes.add(gtuType);
        	collisionLeaderIds.add(gtuLeaderId);
        	collisionLeaderTypes.add(gtuLeaderType);
        }
       
    }
    
    
    /** Class for headway info */
    private class HeadwayInfo {
    	
    	// headway variables
    	private String gtuType = null;
    	private String leaderGtuId = null;
    	private String leaderGtuType = null;
        private double headwayDistance = Double.NaN;
        private double headwayTime = Double.NaN;
        private double timeToCollision = Double.NaN;
        
        // headway info constructor
        public HeadwayInfo(LaneBasedGtu gtu) {
        	
        	// get GTU perception
	        LaneBasedGtu laneBasedGtu = (LaneBasedGtu) gtu;
	        this.gtuType = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
	        CategoricalLanePerception perception = (CategoricalLanePerception) laneBasedGtu.getTacticalPlanner().getPerception();
	        DirectNeighborsPerception neighborPerception = perception.getPerceptionCategoryOrNull(DirectNeighborsPerception.class);
	        
	        if (neighborPerception != null) {
	        	// get headway to leader
	        	PerceptionCollectable<HeadwayGtu,LaneBasedGtu> perceptionCollectable = neighborPerception.getLeaders(RelativeLane.CURRENT);
	        	HeadwayGtu headway = perceptionCollectable.first();
	        	if (headway != null) {
	        		this.leaderGtuId = headway.getId();
	        		this.leaderGtuType = headway.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
		            double distance = headway.getDistance().si;
		            this.headwayDistance = distance;
		            double speed = laneBasedGtu.getSpeed().si;
		            double leaderSpeed = headway.getSpeed().si;
		            
		            // check NaN values, 
		            if (!Double.isNaN(distance) && !Double.isNaN(speed) && !Double.isNaN(leaderSpeed)) {
			            // calculate headway time (brick wall principle)
			            double hwt = distance / speed;
			            this.headwayTime = hwt;
			            // calculate time-to-collision (relative)
			            if (speed > leaderSpeed) {
			            	double ttc = distance / (speed - leaderSpeed);
			            	this.timeToCollision = ttc;
			            }
		            }
	        	}
	        }
        }
        
        @SuppressWarnings("unused")
		public String getGtuType() {
            return gtuType;
        }
        
        public String getLeaderGtuId() {
            return leaderGtuId;
        }
        
        public String getLeaderGtuType() {
            return leaderGtuType;
        }

        public double getHeadwayDistance() {
            return headwayDistance;
        }

        public double getHeadwayTime() {
            return headwayTime;
        }

        public double getTtc() {
            return timeToCollision;
        }

        @Override
        public String toString() {
            return "HeadwayInfo{" +
            	   "gtuType=" + gtuType +
            	   ", leaderGtuId=" + leaderGtuId +
            	   ", leaderGtuType=" + leaderGtuType +
                   ", headwayDistance=" + headwayDistance +
                   ", headwayTime=" + headwayTime +
                   ", ttc=" + timeToCollision +
                   "}";
        }
    }
    
    
    /**
     * Checks for collisions.
     * <p>
     * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
     * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
     * </p>
     * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
     * @author <a href="https://tudelft.nl/staff/p.knoppers-1">Peter Knoppers</a>
     * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
     */
    public class CollisionDetector extends AbstractLaneBasedMoveChecker
    {

        /**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		/**
         * Constructor.
         * @param network Network; network
         */
        public CollisionDetector(final Network network)
        {
            super(network);
        }

        /** {@inheritDoc} */
        @Override
        public void checkMove(final LaneBasedGtu gtu) throws Exception
        {
            try
            {
                NeighborsPerception neighbors =
                        gtu.getTacticalPlanner().getPerception().getPerceptionCategory(NeighborsPerception.class);
                PerceptionCollectable<HeadwayGtu, LaneBasedGtu> leaders = neighbors.getLeaders(RelativeLane.CURRENT);
                Iterator<UnderlyingDistance<LaneBasedGtu>> gtus = leaders.underlyingWithDistance();
                if (!gtus.hasNext())
                {
                    return;
                }
                UnderlyingDistance<LaneBasedGtu> leader = gtus.next();
                if (leader.getDistance().lt0())
                {
                	// track collision for output data
                	String gtuType = gtu.getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
                	String gtuLeaderType = leader.getObject().getParameters().getParameterOrNull(VehicleAutomationConfigurations.AUTOMATION_LEVEL);
                	TrackCollisions(gtu.getId(), gtuType, leader.getObject().getId(), gtuLeaderType);
                	// count collision
                    outputDataManager.increaseSingleCount("collisions");
                    // show error
                    System.err.println("GTU " + gtu.getId() + " collided with GTU " + leader.getObject().getId());
                    gtu.destroy();
                }
            }
            catch (OperationalPlanException exception)
            {
                throw new GtuException(exception);
            }
        }

    }
	
}

