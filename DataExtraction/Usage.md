# ECG block format v2

`ecg_blocks.py` decodes fixed-size ECF2 files and validates ECB2 blocks using only
the Python standard library. Supply consecutive chunks in chunk-index order:

```powershell
python DataExtraction/ecg_blocks.py ecg123_0000.bin ecg123_0001.bin --csv ecg123.csv
python -m unittest discover -s DataExtraction -p test_ecg_blocks.py
```

Omit `--csv` to validate without exporting samples. Output includes raw signed
ECG counts, RTC ticks, sample indices, ETAG/PTAG and a usable-sample indicator.
No voltage calibration or UTC mapping is inferred. An isolated later chunk can
be decoded as a segment; input files are never automatically sorted or joined
across missing chunks. Existing CSV output is replaced when explicitly selected.

Decoding stops at the first invalid page and returns a nonzero exit code. Samples
from earlier validated blocks remain in the CSV. Erased unused pages end decoding
normally; the format does not certify clean closure. The legacy extractor below
does not decode ECF2/ECB2 or ECG stream protocol v0.

# Legacy extraction

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
