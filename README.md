# PowerModelsTopologicalActions.jl

PowerModelsTopologicalActions.jl is a Julia/JuMP package to model AC and DC topological actions (Optimal Transmission Switching (OTS) and Busbar Splitting (BS)) for Steady-State Power Network Optimization. It is based on PowerModels.jl and PowerModelsACDC.jl. This consists of the first package being able to perform both OTS and BS on either part of AC/DC grids. 
While the OTS is a well-established problem in the literature, the BS is still not explored widely, especially for the DC part of AC/DC grids.


**Core Problem Spefications**
* AC Optimal Transmission Switching
* DC Optimal Transmission Switching
* AC/DC Optimal Transmission Switching
* AC Busbar Splitting
* DC Busbar Splitting
* AC/DC Busbar Splitting

**Core network formulations**
* AC (polar coordinates)
* SOC Relaxation (W-space) 
* QC Relaxation (W+L-space)
* LPAC Approximation (Cold Start)


# Acknowledgements

This code has been developed as part of WP1 of the ETF DIRECTIONS project from the FOD Economie of the Belgian Government. The primary developer is Giacomo Bastianel (@GiacomoBastianel) with support from the following contributors:

Marta Vanin (@MartaVanin)


# Citing PowerModelsTopologicalActions.jl



