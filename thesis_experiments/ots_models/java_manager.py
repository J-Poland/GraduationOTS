# imports
import os
import glob
import subprocess


# Class to manage compilation and running of Java file
class JavaManager:

    # initialize this object
    def __init__(self, java_file, compilation_folder, java_parameters, user_feedback=True):
        # input
        self.java_file = java_file
        self.compilation_folder = compilation_folder
        self.java_parameters = java_parameters
        self.user_feedback = user_feedback

        # class variables
        self.compiled = False
        self.classpath = None
        self.java_class_name = None
        self.output = None
        self.original_os_dir = None

    # function to print or suppress user feedback
    def print_feedback(self, message, always_show=False):
        # check user_feedback settings, or whether this message should always be shown (important for errors)
        if self.user_feedback or always_show:
            print(message)

    # function to compile Java project
    def compile(self, classpath_file_path):
        # set Java strings
        java_compiler = 'javac'

        # save current directory
        # this will be changed due to the location of the Java classpath, so store this value to be able to restore it
        self.original_os_dir = os.getcwd()

        # get classpath from Maven project
        # run project as Maven project
        # set Goals field: dependency:build-classpath -DincludeScope=runtime
        # then run and paste resulting string into classpath.txt
        classpath_file = open(classpath_file_path, 'r')
        self.classpath = classpath_file.read().strip()

        # directories for java classes (see Eclipse project)
        # find all .java files in the source directory and subdirectories
        java_files = glob.glob(os.path.join(self.compilation_folder, '**', '*.java'), recursive=True)

        # ensure that the output directory exists
        output_dir = self.compilation_folder
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # compile all Java files in the src/main/java directory
        compilation_result = subprocess.run([java_compiler, "-cp", self.classpath, "-d",
                                             output_dir] + java_files,
                                            capture_output=True, text=True)

        # process compiler errors
        if compilation_result.returncode != 0:
            self.print_feedback("Compilation failed:", True)
            self.print_feedback(compilation_result.stdout)
            self.print_feedback(compilation_result.stderr, True)
        else:
            self.print_feedback("Compilation successful")

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
            self.print_feedback("Running Java program...")
            run_result = subprocess.run([java_runtime, "-cp", self.classpath + ";.",
                                         self.java_class_name] + self.java_parameters,
                                        capture_output=True, text=True)

            if run_result.returncode != 0:
                self.print_feedback('Execution failed:', True)
                self.print_feedback(run_result.stdout)
                self.print_feedback(run_result.stderr, True)
            else:
                self.print_feedback('Execution successful.')
                self.print_feedback('\nConsole output:')
                self.print_feedback(run_result.stdout)
                self.print_feedback(run_result.stderr)
        else:
            self.print_feedback('Java project has to be compiled first.', True)
