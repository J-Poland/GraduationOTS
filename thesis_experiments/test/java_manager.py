# imports
import os
import subprocess


# Class to manage compilation and running of Java file
class JavaManager:

    # initialize this object
    def __init__(self, java_file, compilation_folder, java_parameters):
        # input
        self.java_file = java_file
        self.compilation_folder = compilation_folder
        self.java_parameters = java_parameters

        # class variables
        self.compiled = False
        self.classpath = None
        self.java_class_name = None
        self.output = None

    def compile(self, classpath_file_path):
        # set Java strings
        java_compiler = 'javac'

        # get classpath from Maven project
        # run project as Maven project
        # set Goals field: dependency:build-classpath -DincludeScope=runtime
        # then run and paste resulting string into classpath.txt
        classpath_file = open(classpath_file_path, 'r')
        self.classpath = classpath_file.read().strip()

        # directory for java classes (in same folder, see Eclipse project)
        source_dir = self.compilation_folder
        output_dir = self.compilation_folder
        # compile all Java files in the src/main/java directory
        compilation_result = subprocess.run([java_compiler, "-cp", self.classpath, "-d",
                                             output_dir, source_dir + "/*.java"],
                                            capture_output=True, text=True, shell=True)

        # process compiler errors
        if compilation_result.returncode != 0:
            print("Compilation failed:")
            print(compilation_result.stdout)
            print(compilation_result.stderr)
        else:
            print("Compilation successful")

            # extracting the directory and the class name
            java_file_path = os.path.abspath(self.java_file)
            java_file_dir = os.path.dirname(java_file_path)
            self.java_class_name = os.path.basename(java_file_path)

            # change directory to location of .class file
            os.chdir(java_file_dir)

            # set compiled boolean
            self.compiled = True

    # function to run java file
    def run_java_file(self):
        if self.compiled:
            # set Java runtime
            java_runtime = 'java'

            # run the compiled Java class
            print("Running Java program...")
            run_result = subprocess.run([java_runtime, "-cp", self.classpath + ";.",
                                         self.java_class_name] + self.java_parameters,
                                        capture_output=True, text=True)

            if run_result.returncode != 0:
                print('Execution failed:')
                print(run_result.stdout)
                print(run_result.stderr)
            else:
                print('Execution successful.')
                print('\nConsole output:')
                print(run_result.stdout)
                print(run_result.stderr)
        else:
            print('Java project has to be compiled first.')


# run this script
if __name__ == '__main__':

    # set Java file path
    file_path = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace' \
                r'\traffic-sim\src\main\java\sim\demo\RunMyShortMerge.java'

    project_folder = r'C:\Users\jesse\Documents\Java\TrafficSimulation-workspace\traffic-sim\src\main\java\sim\demo'

    # run simulation headless. With animation is still subject to errors, why?
    run_headless = 'true'

    # set input parameters
    input_parameters = [
        f'-headless={run_headless}',
        '-simTime=120.0',
        '-avFraction=0.20'
    ]

    # compile and run Java file
    JavaManager(file_path, project_folder, input_parameters).run_java_file()
