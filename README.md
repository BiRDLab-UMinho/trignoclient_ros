# trignoclient_ros

Simple ROS wrapper package around <a href="https://github.com/BiRDLab-UMinho/trignoclient">trignoclient</a> library. 
Provides high-level management of Delsys Trigno Research+ system through its proprietary SDK. Streams EMG and Auxiliary data as ROS messages, and allows custom queries to the Trigno server.

## Note

Under development. At this stage, only minimal functionality - small subset of configuration & sensor handling - is exposed as ROS topics/services.