package sim.demo;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import de.siegmar.fastcsv.writer.CsvWriter;


/**
 * Class to handle data export.
 */
public class ExportSimulationOutput
{
	private Map<String, ArrayList<Object>> sequenceOutputMap = new LinkedHashMap<>();
	private Map<String, Object> singleOutputMap = new LinkedHashMap<>();
	
	/**
	 * Constructor for ExportSimulationOutput class.
	 * 
	 * @param sequenceOutputMap
	 * @param constantOutputMap
	 */
	public ExportSimulationOutput(Map<String, ArrayList<Object>> sequenceOutputMap, Map<String, Object> singleOutputMap) {
		this.sequenceOutputMap = sequenceOutputMap;
		this.singleOutputMap = singleOutputMap;
	}
	
	/**
	 * Function to export single (scalar) simulation data to CSV file.
	 * 
	 * @param filePath
	 */
	public void exportSingleToCsv(String singleFilePath) {
		
		// only continue when single output data is available
		if (!singleOutputMap.isEmpty()) {
			
			// user feedback
			System.out.println("\nSingle output data exported to CSV file at \n" + "'" +singleFilePath + "'");
			
			// write output data to given CSV file
	        try (FileWriter fileWriter = new FileWriter(singleFilePath)) {
	        	
	        	// create CSV writer
	            CsvWriter csvWriter = CsvWriter.builder().build(fileWriter);
	            		
	            // get headers and values for singletOutputMap
	            ArrayList<String> headerList = new ArrayList<String>();
	            List<String> valueList = new ArrayList<String>();
	            for (String key : singleOutputMap.keySet()) {
	                headerList.add(key);
	                valueList.add(singleOutputMap.get(key).toString());
	            }
	            
	            // write headers in first row
	            csvWriter.writeRecord(headerList);
	            
	            // write values in second row
	            csvWriter.writeRecord(valueList);
	            
	            // user feedback
	            System.out.println("CSV file created successfully.");
	        
	        } catch (IOException exception) {
	            exception.printStackTrace();
	        }
		}
	}
	
	/**
	 * Function to export sequence (time series) simulation data to CSV file.
	 * 
	 * @param filePath
	 */
	public void exportSequenceToCsv(String sequenceFilePath) {
		
		// only continue when single output data is available
		if (!sequenceOutputMap.isEmpty()) {
			
			// user feedback
			System.out.println("\nSequence output data exported to CSV file at \n" + "'" +sequenceFilePath + "'");
			
			// write output data to given CSV file
	        try (FileWriter fileWriter = new FileWriter(sequenceFilePath)) {
	        	
	        	// create CSV writer
	            CsvWriter csvWriter = CsvWriter.builder().build(fileWriter);
	            		
	            // get headers and values for singletOutputMap
	            ArrayList<String> headerList = new ArrayList<String>();
	            List<ArrayList<Object>> valueList = new ArrayList<ArrayList<Object>>();
	            for (String key : sequenceOutputMap.keySet()) {
	                headerList.add(key);
	                valueList.add(sequenceOutputMap.get(key));
	            }
	            
	            // write headers in first row
	            csvWriter.writeRecord(headerList);
	            
	            // get max length of value lists
	            int maxLength = 0;
	            for (ArrayList<Object> array : valueList) {
	            	int currentSize = array.size();
	            	if (currentSize > maxLength) {
	            		maxLength = currentSize;
	            	}
	            }
	            
	            // go through all required row indexes
	            for (int i = 0; i < maxLength; i++) {
	            	// create list of values for this row index
	            	List<String> rowValues = new ArrayList<String>();
	            	// go through all variables
	            	for (ArrayList<Object> values : valueList) {
	            		if (i < values.size()) {
	            			rowValues.add(values.get(i).toString());
	            		}
	            		else {
	            			rowValues.add("");
	            		}
	            	}
	            	// write this row into CSV file
		            csvWriter.writeRecord(rowValues);
	            }
	            
	            // user feedback
	            System.out.println("CSV file created successfully.");
	        
	        } catch (IOException exception) {
	            exception.printStackTrace();
	        }
		}
	}
	
}

