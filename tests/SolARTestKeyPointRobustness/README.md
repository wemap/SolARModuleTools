To run the test, download the fbow vocabularies from https://github.com/SolarFramework/binaries/releases/download/fbow%2F0.0.1%2Fwin/fbow_voc.zip unzip this file and put the akaze.fbow in your working directory.
Predefined configuration files are proposed in cfg directory.
For b<>com users : 
- translation of euroc, tum and kitti dataset can be found here : \\filer.b-com.local\share\project\ARTWIN\MULTI_DATASET_SLAM_EVALUATION
- copy these data to dara directory
- QtCreator Run configuration: 
	* specify working directory as : __\SolAR\modules\SolARModuleTools\tests\SolARTestKeyPointRobustness
	* as command line argument specify one config file path: {cfg\conf_euroc_01.xml, cfg\conf_euroc_02.xml, cfg\conf_euroc_03.xml, ...}
