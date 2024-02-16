using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII ; const _PMTP = PowerModelsTopologicalActionsII  
using InfrastructureModels; const _IM = InfrastructureModels
using JSON
using Mosek, MosekTools

#######################################################################################
## Define solver ##
#######################################################################################

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)
mosek = JuMP.optimizer_with_attributes(Mosek.Optimizer)

#######################################################################################
## Input data ##
#######################################################################################

test_case_5_acdc = "case5_acdc.m"
test_case_ac_only = "case24.m"

#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_original_5_acdc = _PM.parse_file(data_file_5_acdc)

data_5_acdc = deepcopy(data_original_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

data_file_ac = joinpath(@__DIR__,"data_sources",test_case_ac_only)
data_original_ac = _PM.parse_file(data_file_ac)

#######################################################################################
## Fixed busbar splitting models ##
#######################################################################################
data_ac = deepcopy(data_5_acdc)
data_dc = deepcopy(data_5_acdc)
data_ac_dc = deepcopy(data_5_acdc)

###############################
############# AC ##############
###############################
# Selecting which busbars are split
splitted_bus_ac = 2
data_ac, extremes_ZIL = _PMTP.AC_busbar_split_more_buses_fixed(data_ac,splitted_bus_ac)
split_elements = _PMTP.elements_AC_busbar_split(data_ac)

result_ac = _PMTP.run_acdcsw_AC_fixed(data_ac,ACPPowerModel,juniper)

# Now each element needs to be linked to either part of the busbar being split.
# Test with the AC busbar split
data_ac_fixed = deepcopy(data_ac)
data_ac_fixed["branch"]["4"]["f_bus"] = extremes_ZIL["2"][2]
data_ac_fixed["branch"]["5"]["f_bus"] = extremes_ZIL["2"][2]

result_ac_fixed = _PMTP.run_acdcsw_AC_fixed(data_ac_fixed,ACPPowerModel,juniper)

result_ac_fixed_diff = _PMTP.run_acdcsw_AC_fixed_diff(data_ac_fixed,ACPPowerModel,juniper)

###############################
############# DC ##############
###############################
# Selecting which busbars are split
splitted_bus_dc = 2
data_dc, extremes_ZIL_dc = _PMTP.DC_busbar_split_more_buses_fixed(data_dc,splitted_bus_dc)
split_elements_dc = _PMTP.elements_DC_busbar_split(data_dc)

result_dc = _PMTP.run_acdcsw_DC_fixed(data_dc,ACPPowerModel,juniper)
# Now each element needs to be linked to either part of the busbar being split.
# Test with the AC busbar split
data_dc_fixed = deepcopy(data_dc)
data_dc_fixed["branchdc"]["1"]["tbusdc"] = extremes_ZIL_dc["2"][2]

result_dc_fixed = _PMTP.run_acdcsw_DC_fixed(data_dc_fixed,ACPPowerModel,juniper)

##################################
############# AC/DC ##############
##################################
# Selecting which busbars are split
splitted_bus_ac = 2
splitted_bus_dc = 2
data_ac_dc, extremes_ZIL = _PMTP.AC_busbar_split_more_buses_fixed(data_ac_dc,splitted_bus_ac)
split_elements = _PMTP.elements_AC_busbar_split(data_ac_dc)

data_ac_dc, extremes_ZIL_dc = _PMTP.DC_busbar_split_more_buses_fixed(data_ac_dc,splitted_bus_dc)
split_elements_dc = _PMTP.elements_DC_busbar_split(data_ac_dc)

result_ac_dc = _PMTP.run_acdcsw_AC_DC_fixed(data_ac_dc,ACPPowerModel,juniper)
# Now each element needs to be linked to either part of the busbar being split.
# Test with the AC busbar split
data_ac_dc_fixed = deepcopy(data_ac_dc)
data_ac_dc_fixed["branch"]["4"]["f_bus"] = extremes_ZIL["2"][2]
data_ac_dc_fixed["branch"]["5"]["f_bus"] = extremes_ZIL["2"][2]
data_ac_dc_fixed["branchdc"]["1"]["tbusdc"] = extremes_ZIL_dc["2"][2]

result_ac_dc_fixed = _PMTP.run_acdcsw_AC_DC_fixed(data_ac_dc_fixed,ACPPowerModel,juniper)