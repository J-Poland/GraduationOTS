package sim.demo;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;
import de.siegmar.fastcsv.writer.CsvWriter;


/**
 * Class to handle data export.
 * 
 * Data is stored over different CSVs. To lower the file size, all these CSVs are stored within a zip file.
 * Because a zip file is used, data has to be written to a ZipOutputStream. This ZipOutputStream can add a new
 * entry to insert a file. When this entry is open, data can be written to the file. Once the entry is closed
 */
public class ExportSimulationOutput
{
	/** Path for output folder. */
	String outputFolder;
	
	/** Writers per output file. */
	CsvWriter inputWriter = null;
	CsvWriter sequenceWriter = null;
	CsvWriter singleWriter = null;
	CsvWriter intermediateWriter = null;
	CsvWriter laneChangeWriter = null;
	
	/**
	 * Constructor for ExportSimulationOutput class.
	 */
	public ExportSimulationOutput(String outputFolderPath, String inputFileName, String sequenceFileName, String singleFileName,
								  String intermediateFileName, String laneChangeFileName)
	{	
		// set output folder path
		outputFolder = outputFolderPath;
		
		// check whether this directory exists
		checkDirectory(outputFolder);
		
		// create zip file CSV writers for all required output files
		inputWriter = createWriter(inputFileName);
		sequenceWriter = createWriter(sequenceFileName);
		singleWriter = createWriter(singleFileName);
		intermediateWriter = createWriter(intermediateFileName);
		laneChangeWriter = createWriter(laneChangeFileName);
		
		// close all writers when program is finished or Python terminates the Java program
		// otherwise the generated zip files may be corrupted
		// unfortunately, terminating the program from Eclipse will perform a hard kill
		// in that case, no streams are closed and thus zip files will be corrupted
		Runtime.getRuntime().addShutdownHook(new Thread(() -> {
	        System.out.println("\nShutdown hook triggered. Closing zip files...");
	        closeAllWriterStreams();
	        System.out.println("Done!");
	    }));
	}
	
	
	/**
	 * Function to check whether a file path exists
	 * @param filePath
	 */
	private void checkDirectory(String filePath)
	{
		File file = new File(filePath);
        File parentDir = file.getParentFile();
        if (parentDir != null && !parentDir.exists()) {
            if (!parentDir.mkdirs()) {
                System.err.println("Failed to create directories: " + parentDir.getAbsolutePath());
                return;
            }
        }
	}
	
	/**
	 * Function to create a writer that writes a CSV file into a zip.
	 * @param fileName
	 * @return writer
	 */
	private CsvWriter createWriter(String fileName)
	{
		CsvWriter writer = null;
		
		checkDirectory(outputFolder + "\\" + fileName);
		
		try {
			// create a FileOutputStream for the zip file
	    	FileOutputStream fileOutputStream = new FileOutputStream(outputFolder + "\\" + 
	    															 fileName.substring(0, fileName.length() - 4) + ".zip");
			// wrap FileOutputStream with a ZipOutputStream
	    	ZipOutputStream zipOutputStream = new ZipOutputStream(fileOutputStream);
			// add the CSV to the zip file
	        ZipEntry zipEntry = new ZipEntry(fileName);
	        zipOutputStream.putNextEntry(zipEntry);
	        // create a stream that writes to this CSV
	        OutputStreamWriter outputStreamWriter = new OutputStreamWriter(zipOutputStream);
	        // create CSV writer that writes to the stream
	        writer = CsvWriter.builder().build(outputStreamWriter);
	        		
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// return writer
        return writer;
	}
	
	/**
	 * Function to close all writer streams.
	 */
	public void closeAllWriterStreams() {
		try {
			inputWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {
			sequenceWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {
			singleWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {
			intermediateWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {
			laneChangeWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Function to add headers to the sequence data CSV
	 * @param headers
	 */
	public void writeSequenceDataHeaders(ArrayList<String> headers) {
        // write headers
        sequenceWriter.writeRecord(headers);
	}
	
	/**
	 * Function to add new data row to the sequence data CSV
	 * @param dataRow
	 */
	public void writeSequenceDataRow(Map<String, String> dataRow) {
		// list for all new values
		List<String> row = new ArrayList<>();
		
		// loop through all variables
		for (String variable : dataRow.keySet()) {
			// get value
			String value = dataRow.get(variable);
			// add to row values
			row.add(value);
		}
		
        // write headers
        sequenceWriter.writeRecord(row);
	}
	
	/**
	 * Function to export single (scalar) simulation data to CSV file.
	 * 
	 * @param inputValuesMap
	 * @param filePath
	 */
	public void exportInputToCsv(Map<String, String> inputValuesMap, String inputFileName) {
		// only continue when input data is available
		if (!inputValuesMap.isEmpty()) {
			
			// user feedback
			System.out.println("\nInput parameters exporting to CSV file at \n" + "'" + outputFolder + "\\" + inputFileName + "'");
			
			// get headers and values
            ArrayList<String> headerList = new ArrayList<String>();
            List<String> valueList = new ArrayList<String>();
            for (String key : inputValuesMap.keySet()) {
                headerList.add(key);
                valueList.add(inputValuesMap.get(key));
            }
			
            // write headers in first row
            inputWriter.writeRecord(headerList);
            
            // write values in second row
            inputWriter.writeRecord(valueList);
            
            // user feedback
            System.out.println("Input parameters CSV file created successfully.\n");
	            
		}
	}
	
	/**
	 * Function to export single (scalar) simulation data to CSV file.
	 * 
	 * @param filePath
	 */
	public void exportSingleToCsv(Map<String, Object> singleOutputMap, String singleFileName) {
		
		// only continue when single output data is available
		if (!singleOutputMap.isEmpty()) {
			
			// user feedback
			System.out.println("\nSingle output data exporting to CSV file at \n" + "'" + outputFolder + "\\" + singleFileName + "'");
					
            // get headers and values for singletOutputMap
            ArrayList<String> headerList = new ArrayList<String>();
            List<String> valueList = new ArrayList<String>();
            for (String key : singleOutputMap.keySet()) {
                headerList.add(key);
                valueList.add(singleOutputMap.get(key).toString());
            }
            
            // write headers in first row
            singleWriter.writeRecord(headerList);
            
            // write values in second row
            singleWriter.writeRecord(valueList);
		}
	}
	
	
	public void exportIntermediateMeanValuesToCsv(Map<String, ArrayList<Double>> meanMap, String intermediateMeanValuesFileName) {
		// only continue when output data is available
		if (!meanMap.isEmpty()) {
			
			// user feedback
			System.out.println("\nIntermediate output data exporting to CSV file at \n" + "'" + outputFolder + "\\" + intermediateMeanValuesFileName + "'"); 
			
            // determine max value count
            int maxCount = 0;
            for (ArrayList<Double> values : meanMap.values()) {
            	if (values.size() > maxCount) {
            		maxCount = values.size();
            	}
            }
            
            // set headers
            ArrayList<String> headers = new ArrayList<String>();
            for (String variable : meanMap.keySet()) {
            	headers.add(variable);
            }
            // write headers to CSV
            intermediateWriter.writeRecord(headers);
	            
            // iterate through rows (max count)
            for (int i = 0; i < maxCount; i++) {
            	// create value list for this row
            	ArrayList<String> row = new ArrayList<String>();
		        // loop through stored variable values
		    	for (ArrayList<Double> values : meanMap.values()) {
		    		// check whether current index is valid for this variable
		    		// not valid? then save an empty value
		    		if (i >= values.size()) {
		    			row.add("");
		    			continue;
		    		}
		    		// get value for this variable for this row
		    		row.add(values.get(i).toString());
		    	}
		    	
		    	// write the row to CSV
                intermediateWriter.writeRecord(row);
            }
            
            // user feedback
            System.out.println("Intermediate CSV file created successfully.");
		}
    	
    }
	
	public void exportLaneChangesToCsv(ArrayList<String> laneChangeTime, ArrayList<String> laneChangeIds, ArrayList<String> laneChangeDirections,
			ArrayList<String> laneChangeLinks, ArrayList<String> laneChangeFromLanes, String laneChangeFileName) {
		// only continue when output data is available
		if (!laneChangeIds.isEmpty()) {
			
			// user feedback
			System.out.println("\nLane change output data exporting to CSV file at \n" + "'" + outputFolder + "\\" + laneChangeFileName + "'"); 
			
			// go through all required row indexes
			int maxLength = laneChangeIds.size();
            for (int i = -1; i < maxLength; i++) {
            	// create list of values for this row index
            	List<String> rowValues = new ArrayList<String>();
            	// headers
            	if (i == -1) {
	    			rowValues.add("time");
            		rowValues.add("id");
					rowValues.add("direction");
	            	rowValues.add("link");
	            	rowValues.add("from_lane");
            	}
            	// values
            	else {
	    			rowValues.add(laneChangeTime.get(i));
	            	rowValues.add(laneChangeIds.get(i));
					rowValues.add(laneChangeDirections.get(i));
	            	rowValues.add(laneChangeLinks.get(i));
	            	rowValues.add(laneChangeFromLanes.get(i));
            	}
				
            	// write this row into CSV file
	            laneChangeWriter.writeRecord(rowValues);
            }
	            
            // user feedback
            System.out.println("Lane change CSV file created successfully.");
		}
	}
	
}

