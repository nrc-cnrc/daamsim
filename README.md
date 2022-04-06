# DAAMSIM

DAAMSim is a publically available modeling and simulation framework, developed by the National Research Council of Canada, to support the determination of DAA system requirements, and evaluation of DAA system performance. The framework incorporates the functional components including various sensor, tracker, and avoid models, data replay, visualization tools, and offline metrics. 

Contact iryna.borshchova@nrc-cnrc.gc.ca or kris.ellis@nrc-cnrc.gc.ca for further details.

## Requirements

### Operating systems
This codebase provides support for both **MacOS** and **Windows** operating systems.

### Necessary Add-Ons
Since the project is written in [MATLAB](https://www.mathworks.com/products/matlab.html), you'll need a to install it and have a valid license of it. On installation, you can add the following add-ons to fully support the features of this codebase or [add them later once MATLAB is installed](https://www.mathworks.com/help/matlab/matlab_env/get-add-ons.html).

To calculate requirements for example DAA system with the MetricsExample.m script, you'll need to have installed the **[Mapping Toolbox Add-On](https://www.mathworks.com/products/mapping.html)** for MATLAB.

To run the Simulink simluation, you'll need to have the **[Simulink Add-On](https://www.mathworks.com/products/simulink.html)** and **[Statistics and Machine Learning Add-On](https://www.mathworks.com/products/statistics.html)** installed. 

## How to run

1. To run Simulink simulation, run start.m then run M1_NRC.slx.
2. To calculate requirements for sample DAA system, run MetricsExample.m.
3. To test camera model, run CameraModel.m


