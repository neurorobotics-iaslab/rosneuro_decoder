# ROS-Neuro decoder package
The package provides a generic interface for decoding [NeuroFrame](https://github.com/rosneuro/rosneuro_msgs) processed. The interface accepts plugins that can be independently developed and dynamically loaded.
For now, it worked only as classes with dynamic plugin.

## Usage
The package acquires required the extracted features and apply the decoder. The following command can be used to run the node:
```
rosrun rosneuro_decoder decoder _plugin:=[INTEGRATORPLUGIN] [OPTIONAL PLUGIN-RELATED PARAMETERS]
```
**Example with ExponentialSmoothing plugin**
```
rosrun rosneuro_decoder decoder _plugin:=rosneuro::decoder::Gaussian
```

