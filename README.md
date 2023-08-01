# ROS-Neuro decoder package
The package provides a generic interface for decoding [NeuroFrame](https://github.com/rosneuro/rosneuro_msgs) processed. The interface accepts plugins that can be independently developed and dynamically loaded.
For now, it worked only as classes with dynamic plugin.

## Usage
The package acquires required the extracted features and apply the decoder. The following command can be used to run the node:
```
rosrun rosneuro_decoder decoder _plugin:=[INTEGRATORPLUGIN] [OPTIONAL PLUGIN-RELATED PARAMETERS]
```
**Example with Gaussian decoder plugin**
```
rosrun rosneuro_decoder decoder _plugin:=rosneuro::decoder::Gaussian
```

## Usage example
In the cpp code:
```
#include <rosneuro_decoder/Decoder.h>

rosneuro::decoder::Decoder* decoder = new rosneuro::decoder::Decoder();

// configure it using ros parameters passed in the launch file
decoder->configure();

// apply the decoder
Eigen::VectorXf probabilities = decoder->apply(features);
```
while in the launch file, you need to specify the plugin. For instance, we use a Qda classifier in the test_qda.cpp file:
```
<arg name="plugin" default='rosneuro::decoder::Qda'/>
	<arg name="cfg_name" default='QdaCfg'/>
	
    <rosparam command="load" file="$(find rosneuro_decoder_qda)/test/qdaCfg.yaml"/>
	<node name="test_qda" pkg="rosneuro_decoder_qda" type="test_qda" output="screen" >
		<param name="~plugin" 	  value="$(arg plugin)"/>
        <param name="cfg_name" 	  value="$(arg cfg_name)"/>
        
	</node>
```