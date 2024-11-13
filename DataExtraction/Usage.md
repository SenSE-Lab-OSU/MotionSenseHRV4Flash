Welcome to MotionSense Data Extraction Script! This tool is meant to function as an
data extraction parser which will convert MSense4 binary data files to high level csv 
which you can analyse.

To use, you must have a python installed on your computer, and a working MSense device with 
data files stored on the drive. 

If you have both of these, you should be able to navigate to the directory in which this 
script is stored, and enter 'python data_extraction.py 'csv_tile' 'MSense drive'', where
'csv_title' and 'MSense drive' refer to the title and drive that the MSense drive is stored on.

for example, if the MSense Drive is given storage drive F:, and the csv title is participant1,
entering 'python data_extraction.py participant1 F:/' will result in csv files under the prefx 'participant1_xx.csv'
in the directory with which the script was executed, provided that the data in the MSense files is present and non corrupt.
There will be a seperate set of csv files for every id that the device has stored (the id is located at the beginning of every
file.)

