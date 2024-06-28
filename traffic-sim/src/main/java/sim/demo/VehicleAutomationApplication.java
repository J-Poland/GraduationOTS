package sim.demo;

import java.awt.Dimension;
import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.List;

import javax.naming.NamingException;

import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Time;
import org.opentrafficsim.animation.GraphLaneUtil;
import org.opentrafficsim.animation.colorer.LmrsSwitchableColorer;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.LinkPosition;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.draw.OtsDrawingException;
import org.opentrafficsim.draw.graphs.FundamentalDiagram;
import org.opentrafficsim.draw.graphs.GraphCrossSection;
import org.opentrafficsim.draw.graphs.FundamentalDiagram.FdSource;
import org.opentrafficsim.draw.graphs.GraphPath;
import org.opentrafficsim.draw.graphs.PlotScheduler;
import org.opentrafficsim.draw.graphs.TrajectoryPlot;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.object.SpeedSign;
import org.opentrafficsim.road.network.sampling.LaneDataRoad;
import org.opentrafficsim.road.network.sampling.RoadSampler;
import org.opentrafficsim.swing.graphs.OtsPlotScheduler;
import org.opentrafficsim.swing.graphs.SwingPlot;
import org.opentrafficsim.swing.graphs.SwingTrajectoryPlot;
import org.opentrafficsim.swing.gui.AnimationToggles;
import org.opentrafficsim.swing.gui.OtsAnimationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.language.DsolException;


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
public class VehicleAutomationApplication extends OtsSimulationApplication<VehicleAutomationModel> {
	
	/** */
	private static final long serialVersionUID = 20170407L;

	/**
	 * Create a ShortMerge Swing application.
	 * 
	 * @param title String; the title of the Frame
	 * @param panel OtsAnimationPanel; the tabbed panel to display
	 * @param model ShortMergeModel; the model
	 * @throws OtsDrawingException on animation error
	 */
	public VehicleAutomationApplication(final String title, final OtsAnimationPanel panel, final VehicleAutomationModel model)
			throws OtsDrawingException {
		super(model, panel);
	}

	/** {@inheritDoc} */
	@Override
	protected void setAnimationToggles() {
		AnimationToggles.setTextAnimationTogglesFull(getAnimationPanel());
		getAnimationPanel().getAnimationPanel().toggleClass(Link.class);
		getAnimationPanel().getAnimationPanel().toggleClass(Node.class);
		getAnimationPanel().getAnimationPanel().showClass(SpeedSign.class);
	}

	/** {@inheritDoc} */
	@Override
	protected void addTabs() {
		GraphPath<LaneDataRoad> path;
		try {
			Lane start = ((CrossSectionLink) getModel().getNetwork().getLink("AB")).getLanes().get(1);
			path = GraphLaneUtil.createPath("Right lane", start);
		} catch (NetworkException exception) {
			throw new RuntimeException("Could not create a path as a lane has no set speed limit.", exception);
		}
		RoadSampler sampler = new RoadSampler(getModel().getNetwork());
		GraphPath.initRecording(sampler, path);
		PlotScheduler scheduler = new OtsPlotScheduler(getModel().getSimulator());
		Duration updateInterval = Duration.instantiateSI(10.0);
		SwingPlot plot = new SwingTrajectoryPlot(
				new TrajectoryPlot("Trajectory right lane", updateInterval, scheduler, sampler.getSamplerData(), path));
		getAnimationPanel().getTabbedPane().addTab(getAnimationPanel().getTabbedPane().getTabCount(), "Trajectories",
				plot.getContentPane());
		
		// define cross section to base FD values on
        List<String> names = new ArrayList<>();
        names.add("FORWARD1");
        names.add("FORWARD2");
        Length lanePosition = Length.instantiateSI(895.0);	// link DE has a length of 1895 (2105 - 4000)
        LinkPosition linkPosition = new LinkPosition(getModel().getNetwork().getLink("DE"), lanePosition);
        GraphCrossSection<LaneDataRoad> crossSection;
        try
        {
        	crossSection = GraphLaneUtil.createCrossSection(names, linkPosition);
        }
        catch (NetworkException exception)
        {
            throw new RuntimeException("Unable to create cross section.", exception);
        }
        
		FdSource source = FundamentalDiagram.sourceFromSampler(sampler, crossSection, true, updateInterval, false);
	}

	/**
	 * Start method to invoke the simulation.
	 * 
	 * @param simConfig
	 */
	public static void start(VehicleAutomationModelParameters simConfig) {
		demo(true, simConfig);
	}

	/**
	 * Start the demo.
	 * 
	 * @param exitOnClose boolean; when running stand-alone: true; when running as
	 *                    part of a demo: false
	 */
	public static void demo(final boolean exitOnClose, final VehicleAutomationModelParameters simConfig) {
		try {
			OtsAnimator simulator = new OtsAnimator("ShortMerge");
			final VehicleAutomationModel otsModel = new VehicleAutomationModel(simulator, simConfig);
			simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(simConfig.getSimTime()), otsModel);
			OtsAnimationPanel animationPanel = new OtsAnimationPanel(otsModel.getNetwork().getExtent(),
					new Dimension(800, 600), simulator, otsModel,
					new LmrsSwitchableColorer(VehicleConfigurations.GTU_TYPE_COLORS.toMap()), otsModel.getNetwork());
			VehicleAutomationApplication app = new VehicleAutomationApplication("ShortMerge", animationPanel, otsModel);
			app.setExitOnClose(exitOnClose);
			animationPanel.enableSimulationControlButtons();
		} catch (SimRuntimeException | NamingException | RemoteException | OtsDrawingException
				| IndexOutOfBoundsException | DsolException exception) {
			exception.printStackTrace();
		}
	}
}

