![image](https://user-images.githubusercontent.com/58992009/164030483-1e7d1cec-f807-4b95-b92f-a1a5ae8da1b0.png)

# DAAMSIM

DAAMSim is a publically available modeling and simulation framework, developed by the National Research Council of Canada (NRC), to support the determination of DAA system requirements, and evaluation of DAA system performance. The framework incorporates the functional components including various sensor, tracker, and avoid models, data replay, visualization tools, and offline metrics. The framework incorporates NRC-modified version of **[NASA DAIDALUS]( https://github.com/nasa/daidalus)** (license is presented in the "DAIDALUS" folder of the repository).  

The framework is documented in https://cdnsciencepub.com/doi/abs/10.1139/dsa-2021-0044 and includes the intercept data collected by the NRC after conducting actual collision intercepts between a surrogate RPAS (Bell 205) and multiple manned intruder targets, documented in https://cdnsciencepub.com/doi/full/10.1139/juvs-2021-0005. This data is presented in Matlab structure format, as well as in .daa format which could be replayed using NASA-developed **[UASChorus](https://nasa.github.io/daidalus/)** visualization tool.

Currently, only part of the framework is presented for public consumption. Authors are exploring licensing options to re-distribute NRC-modified versions of Matlab sensor models and trackers. Researchers and developers are welcome to contribute and improve the DAAMSIM framework. Contact iryna.borshchova@nrc-cnrc.gc.ca or kris.ellis@nrc-cnrc.gc.ca for further details.

## Requirements

### Operating systems
This codebase provides support for both **MacOS** and **Windows** operating systems.

### Necessary Add-Ons
Since the project is written in [MATLAB](https://www.mathworks.com/products/matlab.html), you'll need a to install it and have a valid license of it. On installation, you can add the following add-ons to fully support the features of this codebase or [add them later once MATLAB is installed](https://www.mathworks.com/help/matlab/matlab_env/get-add-ons.html).

To calculate requirements for example DAA system with the MetricsExample.m script, you'll need to have installed the **[Mapping Toolbox Add-On](https://www.mathworks.com/products/mapping.html)** for MATLAB.

To run the Simulink simluation, you'll need to have the **[Simulink Add-On](https://www.mathworks.com/products/simulink.html)** and **[Statistics and Machine Learning Add-On](https://www.mathworks.com/products/statistics.html)** installed. 

## How to run

1. To run Simulink simulation, execute "clear all"; run start.m then run M1_NRC.slx.
2. To run Simulink simulation with NASA DAIDALUS alerting, execute "clear all"; run start.m, then run M2_DAIDALUS.slx. Check DAIDALUS output in the form of bands and alerts in Debug->Diagnostics window of the Simulink simulation.
3. To calculate requirements for a sample DAA system, run MetricsExample.m.
4. To test camera model, run CameraModel.m


