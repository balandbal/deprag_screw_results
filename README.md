# deprag_downloader
## Application
This ROS node allows to automatically download curves saved by a deprag screwdriver.
To do so simply call the provided ROS service with the corresponding number.

## Dependencies
This Package was developed using ROS Melodic other versions might work but are not tested.
Selenium for Python with a Firefox driver installed.

## Setup
After installing all dependencies change the "downloadDirectory" variable in deprag_downloader_node.py to 
the file path where the csv files should be saved.
Now change the folder name of the folder you cloned from git to "deprag_downloader"
Furthermore, change the "depragIP" variable to the address matching your deprag screwdriver.

## Usage
After making sure roscore is running start the deprag_downloader_node using
$ rosrun deprag_downloader deprag_downloader_node.py 

Now a curve can be saved by calling the "/deprag_download" service.

The number with which the service is called determines what curve is downloaded.
Notice that 0 will download all files available, therefore passing 1 will download the latest curve.

## Acknowledgement
Sponsored by the Ministry of the Environment Baden-Württemberg, in the context of the Strategic Dialogue Automotive Industry, and supervised by the Project Management Agency Karlsruhe (PTKA). Funding number: L7520101

https://www.ipa.fraunhofer.de/de/referenzprojekte/DeMoBat.html
