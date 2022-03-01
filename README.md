<div align="center" id="top"> 
  <img src="./img/rss.jpg" alt="wifi_localization" />

  &#xa0;

  <!-- <a href="https://wifi_localization.netlify.app">Demo</a> -->
</div>

<h1 align="center">wifi_localization</h1>


<!-- Status -->

<!-- <h4 align="center"> 
	🚧  wifi_localization 🚀 Under construction...  🚧
</h4> 

<hr> -->

<p align="center">
  <a href="#dart-overview">Overview</a> &#xa0; | &#xa0; 
  <a href="#sparkles-structures">Structures</a> &#xa0; | &#xa0;
  <a href="#rocket-Features">Features</a> &#xa0; | &#xa0;
  <a href="#white_check_mark-notice">Notice</a> &#xa0; | &#xa0;
  <a href="#checkered_flag-installation">Installation</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
</p>

<br>

# :dart: Overview #
This package contains the implementation of wifi localization system, including data collection, data processing and localization system. 

The localization is based on MCL(Monte Carlo Localization). 

The ackermann model is picked as motion model to propagate particles. 

Sensor model is based on path loss and gaussian process.
  
# :sparkles: Structures #
The project consists of various folders:

    bin: containing various ROS nodes
    config: containing parameter files (.yaml) for ROS nodes
    data: wifi data will be placed under this folder based on dates
    img: images for documentation
    launch: ROS launch files
    result: containing selected bssid and trained sensor model parameters
    msg: customized ROS messages
    src: source code of MCL and data collection/processing, MCL components are put under folder "particle"
    srv: ROS service files
    test: scripts for checking implementation

Various ROS nodes:

    simulator: publish dummy tf data to test pipeline
    receiver: to associate wifi data with latest robot pose and publish it ROS message
    service: to provide tf services as some nodes need to work with modules that are only available in Python 3
    collector: subscribe to assembled rss/pose data and save them in different CSV files under data folder
    process: pick data from data folder and apply processing, then save the results in result folder
    localizer: subscribe to receiver output topic, then use results from offline training to estimate pose and determine if kidnapped situation happens or not
    detector: compare the particles from laser based method and wifi based method to detect if robot is kidnapped

# :rocket: Features #
## Data collection: ##
<div align="center" id="collection"> 
<img src="./img/data collection phase.jpg" alt="data collection" />
</div> 
Three collection methods are considered and implemented. They are:

    1. iwlist: call iwlist command from Python and do script processing to get wifi signal readings. However, this command bring significant delay
    2. wpa_cli: this is used as the main method for collecting data
    3. monitor mode: configure network interface to monitor mode and run sniffer to capture 802.11 radiotap headers, for more details, refer to https://wiki.wireshark.org/CaptureSetup/WLAN 

For more details, refer to "scan.py" under "src/general/"

Run roslaunch with "collect.launch" to collect data. Unfortunately, monitor mode is currently not integrated with ROS.

## Data Processing: ##
<div align="center" id="offline"> 
<img src="./img/offline training phase.jpg" alt="data processing" />
</div> 
In data processing part:

    1. stable bssids are selected based on effective counts (rss values greater than a certain threshold to be counted more than a certain percentage)
    2. filter original data and normalize then to scale between 0 to 1 (-95 corresponding to 0 and 0 to 1)
    3. train sensor model and dump parameters to result folder

Run roslaunch with "process.launch" to process collected data.

## Localization system: ##
<div align="center" id="online"> 
<img src="./img/online localization phase.jpg" alt="localization system" />
</div> 
The implementation of MCL has been put under folder "\src\particle\", where "motion.py" contains the implementation of motion model and "sensor.py" contains the implementation of sensor model. Motion model propagates particles based on odometry data, which is obtained by calling tf services.

# :white_check_mark: Notice #
Before stating, you need have ROS (refer to link above) and related Python packages installed
- [ROS melodic](http://wiki.ros.org/melodic)

Some nodes have to be run in Python2.7:

    service: due to default installation of tf module in ROS melodic 
    receiver: due to stability of subprocess module

# :checkered_flag: Installation #
```bash
```bash
# Clone this project
$ cd ~/catkin/src && git clone https://github.com/ZhenzhongLei/thesis.git

# Compile
$ cd ~/catkin && catkin_make

# Install dependencies
$ pip install -r requirement.txt
$ ...

# Run the project
$ roslaunch wifi_localization collect.launch 

```

# :memo: License #

To-do

Developed by <a href="leizhenzhong15@gmail.com" target="_blank">{zhenzhong lei}</a>
&#xa0;

<a href="#top">Back to top</a>