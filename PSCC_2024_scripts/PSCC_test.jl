using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII; const _PMTP = PowerModelsTopologicalActionsII
using PowerModelsTopologicalActionsII   
using InfrastructureModels; const _IM = InfrastructureModels

#######################################################################################
## Define solver ##
#######################################################################################

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 7200)

#######################################################################################
## Input data ##
#######################################################################################

test_case = "case5.m"
test_case_sw = "case5_sw.m"
test_case_5_acdc = "case5_acdc.m"
test_case_24_acdc = "case24_3zones_acdc.m"
test_case_39_acdc = "case39_acdc.m"
test_case_3120_acdc = "case3120sp_acdc.m"
test_case_Belgium = "BE_grid_with_energy_island.json"

#######################################################################################
## Parsing input data ##
#######################################################################################

s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)
#s = Dict("output" => Dict("branch_flows" => true,"report_duals" => true), "conv_losses_mp" => true)

data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_original_5_acdc = _PM.parse_file(data_file_5_acdc)
data_5_acdc = deepcopy(data_original_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

data_file_24_acdc = joinpath(@__DIR__,"data_sources",test_case_24_acdc)
data_original_24_acdc = _PM.parse_file(data_file_24_acdc)
data_24_acdc = deepcopy(data_original_24_acdc)
_PMACDC.process_additional_data!(data_24_acdc)

data_file_39_acdc = joinpath(@__DIR__,"data_sources",test_case_39_acdc)
data_original_39_acdc = _PM.parse_file(data_file_39_acdc)
data_39_acdc = deepcopy(data_original_39_acdc)
_PMACDC.process_additional_data!(data_39_acdc)

data_file_3120_acdc = joinpath(@__DIR__,"data_sources",test_case_3120_acdc)
data_original_3120_acdc = _PM.parse_file(data_file_3120_acdc)
data_3120_acdc = deepcopy(data_original_3120_acdc)
_PMACDC.process_additional_data!(data_3120_acdc)

data_file_Belgian_grid = joinpath(@__DIR__,"data_sources",test_case_Belgium)
data_original_Belgian_grid = _PM.parse_file(data_file_Belgian_grid)
data_Belgian_grid_acdc = deepcopy(data_original_Belgian_grid)
#_PMACDC.process_additional_data!(data_Belgian_grid_acdc)

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_5    = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt)
result_opf_24   = _PMACDC.run_acdcopf(data_24_acdc,ACPPowerModel,ipopt)
result_opf_39   = _PMACDC.run_acdcopf(data_39_acdc,ACPPowerModel,ipopt)
result_opf_3120 = _PMACDC.run_acdcopf(data_3120_acdc,ACPPowerModel,ipopt)
result_Belgium = _PMACDC.run_acdcopf(data_Belgian_grid_acdc,ACPPowerModel,ipopt)

result_opf_soc_5    = _PMACDC.run_acdcopf(data_5_acdc,SOCWRPowerModel,ipopt)
result_opf_soc_24    = _PMACDC.run_acdcopf(data_24_acdc,SOCWRPowerModel,ipopt)


#result_soc = _PMACDC.run_acdcopf(data_acdc,SOCWRPowerModel,juniper)
#result_qc = _PMACDC.run_acdcopf(data_acdc,QCRMPowerModel,juniper)


# Solving AC OTS with OTS only on the AC grid part
result_AC_ots_5    = _PMTP.run_acdcots_AC(data_5_acdc,ACPPowerModel,juniper)
result_AC_ots_24   = _PMTP.run_acdcots_AC(data_24_acdc,ACPPowerModel,juniper)
result_AC_ots_39   = _PMTP.run_acdcots_AC(data_39_acdc,ACPPowerModel,juniper)
#result_AC_ots_3120 = _PMTP.run_acdcots_AC(data_3120_acdc,ACPPowerModel,juniper)

# SOC and QC are essentially the same here -> proof
result_AC_ots_5_soc    = _PMTP.run_acdcots_AC(data_5_acdc,SOCWRPowerModel,juniper)
result_AC_ots_5_qc    = _PMTP.run_acdcots_AC(data_5_acdc,QCRMPowerModel,juniper)

result_AC_ots_24_soc    = _PMTP.run_acdcots_AC(data_24_acdc,SOCWRPowerModel,juniper)
result_AC_ots_24_qc   = _PMTP.run_acdcots_AC(data_24_acdc,SOCWRPowerModel,juniper)


# Solving AC OTS with OTS only on the DC grid part 
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper)
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,SOCWRPowerModel,juniper)


result_DC_ots_24   = _PMTP.run_acdcots_DC(data_24_acdc,ACPPowerModel,juniper)
result_DC_ots_39   = _PMTP.run_acdcots_DC(data_39_acdc,ACPPowerModel,juniper)
#result_DC_ots_3120 = _PMTP.run_acdcots_DC(data_3120_acdc,ACPPowerModel,juniper)


# Solving AC OTS with OTS on both AC and DC grid part
result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper)
result_AC_DC_ots_24   = _PMTP.run_acdcots_AC_DC(data_24_acdc,ACPPowerModel,juniper)
result_AC_DC_ots_39   = _PMTP.run_acdcots_AC_DC(data_39_acdc,ACPPowerModel,juniper)
#result_AC_DC_ots_3120 = _PMTP.run_acdcots_AC_DC(data_3120_acdc,ACPPowerModel,juniper)

#######################################################################################
## Busbar splitting (one busbar) models ##
#######################################################################################
# AC OTS for AC/DC grid with AC switches state as decision variable
data_busbar_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbar_ac_split_24_acdc = deepcopy(data_24_acdc)
data_busbar_ac_split_39_acdc = deepcopy(data_39_acdc)
#data_busbar_split_3120_acdc = deepcopy(data_file_3120_acdc)

data_busbar_ac_split_5_acdc, switch_couples_ac_5,   extremes_ZIL_5_ac  = _PMTP.AC_busbar_split(data_busbar_ac_split_5_acdc,1)
#data_busbar_ac_split_24_acdc, switch_couples_ac_24, extremes_ZIL_24_ac = _PMTP.AC_busbar_split(data_busbar_ac_split_24_acdc,2)
data_busbar_ac_split_39_acdc, switch_couples_ac_39, extremes_ZIL_39_ac = _PMTP.AC_busbar_split(data_busbar_ac_split_39_acdc,1)

result_AC_DC_5_switch_AC  = _PMTP.run_acdcsw_AC(data_busbar_ac_split_5_acdc,ACPPowerModel,juniper)
result_AC_DC_24_switch_AC = _PMTP.run_acdcsw_AC(data_busbar_ac_split_24_acdc,ACPPowerModel,juniper)
result_AC_DC_39_switch_AC = _PMTP.run_acdcsw_AC(data_busbar_ac_split_39_acdc,ACPPowerModel,juniper)



# AC OTS for AC/DC grid with DC switches state as decision variable
data_busbar_dc_split_5_acdc = deepcopy(data_5_acdc)
data_busbar_dc_split_24_acdc = deepcopy(data_24_acdc)
data_busbar_dc_split_39_acdc = deepcopy(data_39_acdc)
#data_busbar_dc_split_3120_acdc = deepcopy(data_file_3120_acdc)

data_busbar_dc_split_5_acdc , switch_couples_dc_5,  extremes_ZIL_5_dc  = _PMTP.DC_busbar_split(data_busbar_dc_split_5_acdc,1)
data_busbar_dc_split_24_acdc, switch_couples_dc_24, extremes_ZIL_24_dc = _PMTP.DC_busbar_split(data_busbar_dc_split_24_acdc,2)
data_busbar_dc_split_39_acdc, switch_couples_dc_39, extremes_ZIL_39_dc = _PMTP.DC_busbar_split(data_busbar_dc_split_39_acdc,1)

result_AC_DC_5_switch_DC  = _PMTP.run_acdcsw_DC(data_busbar_dc_split_5_acdc, ACPPowerModel,juniper)
result_AC_DC_24_switch_DC = _PMTP.run_acdcsw_DC(data_busbar_dc_split_24_acdc,ACPPowerModel,juniper)
result_AC_DC_39_switch_DC = _PMTP.run_acdcsw_DC(data_busbar_dc_split_39_acdc,ACPPowerModel,juniper)



# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_busbar_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
data_busbar_ac_dc_split_24_acdc = deepcopy(data_24_acdc)
data_busbar_ac_dc_split_39_acdc = deepcopy(data_39_acdc)
#data_busbar_dc_split_3120_acdc = deepcopy(data_file_3120_acdc)

data_busbar_ac_dc_split_5_acdc , switch_couples_ac_dc_5,  extremes_ZIL_5_dc  = _PMTP.DC_busbar_split(data_busbar_dc_split_5_acdc,1)
data_busbar_ac_dc_split_24_acdc, switch_couples_ac_dc_24, extremes_ZIL_24_dc = _PMTP.DC_busbar_split(data_busbar_dc_split_24_acdc,2)
data_busbar_ac_dc_split_39_acdc, switch_couples_ac_dc_39, extremes_ZIL_39_dc = _PMTP.DC_busbar_split(data_busbar_dc_split_39_acdc,1)

result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbar_dc_split_5_acdc, ACPPowerModel,juniper)
result_AC_DC_24_switch_AC_DC = _PMTP.run_acdcsw_AC_DC(data_busbar_dc_split_24_acdc,ACPPowerModel,juniper)
result_AC_DC_39_switch_AC_DC = _PMTP.run_acdcsw_AC_DC(data_busbar_dc_split_39_acdc,ACPPowerModel,juniper)

#######################################################################################
## Busbar splitting (more than one busbar) models ##
#######################################################################################
# AC OTS for AC/DC grid with AC switches state as decision variable
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_24_acdc = deepcopy(data_24_acdc)
data_busbars_ac_split_39_acdc = deepcopy(data_39_acdc)
#data_busbar_split_3120_acdc = deepcopy(data_file_3120_acdc)

splitted_bus_ac = [4,5]

splitted_bus_ac = 5
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,ACPPowerModel,juniper)




data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
data_busbars_ac_split_24_acdc, switches_couples_ac_24, extremes_ZILs_24_ac = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_24_acdc,splitted_bus_ac)
data_busbars_ac_split_39_acdc, switches_couples_ac_39, extremes_ZILs_39_ac = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_39_acdc,splitted_bus_ac)

result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,ACPPowerModel,juniper)
result_AC_DC_24_switches_AC = _PMTP.run_acdcsw_AC(data_busbars_ac_split_24_acdc,ACPPowerModel,juniper)
result_AC_DC_39_switches_AC = _PMTP.run_acdcsw_AC(data_busbars_ac_split_39_acdc,ACPPowerModel,juniper)

result_AC_DC_39_switches_AC = _PMTP.run_acdcsw_AC(data_busbars_ac_split_39_acdc,SOCWRPowerModel,juniper)



# AC OTS for AC/DC grid with DC switches state as decision variable
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_dc_split_24_acdc = deepcopy(data_24_acdc)
data_busbar_dc_split_39_acdc = deepcopy(data_39_acdc)
#data_busbar_dc_split_3120_acdc = deepcopy(data_file_3120_acdc)

splitted_bus_dc = [1,2]

data_busbars_dc_split_5_acdc , switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc,splitted_bus_dc)
data_busbars_dc_split_24_acdc, switches_couples_dc_24, extremes_ZILs_24_dc = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_24_acdc,splitted_bus_dc)
data_busbars_dc_split_39_acdc, switches_couples_dc_39, extremes_ZILs_39_dc = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_39_acdc,splitted_bus_dc)

result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, ACPPowerModel,juniper)
result_AC_DC_24_switches_DC = _PMTP.run_acdcsw_DC(data_busbars_dc_split_24_acdc,ACPPowerModel,juniper)
result_AC_DC_39_switches_DC = _PMTP.run_acdcsw_DC(data_busbars_dc_split_39_acdc,ACPPowerModel,juniper)



# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_dc_split_24_acdc = deepcopy(data_24_acdc)
data_busbars_ac_dc_split_39_acdc = deepcopy(data_39_acdc)
#data_busbar_dc_split_3120_acdc = deepcopy(data_file_3120_acdc)

splitted_bus_ac = [1,2]
splitted_bus_dc = [1,2]

data_busbars_ac_dc_split_5_acdc,  switches_couples_ac_dc_5,  extremes_ZILs_5_ac_dc  = _PMTP.AC_busbar_split(data_busbar_ac_dc_split_5_acdc,zsplitted_bus_ac)
data_busbars_ac_dc_split_24_acdc, switches_couples_ac_dc_24, extremes_ZILs_24_ac_dc = _PMTP.AC_busbar_split(data_busbar_ac_dc_split_24_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_39_acdc, switches_couples_ac_dc_39, extremes_ZILs_39_ac_dc = _PMTP.AC_busbar_split(data_busbar_ac_dc_split_39_acdc,splitted_bus_ac)

data_busbars_ac_dc_split_5_acdc , switches_couples_ac_dc_5,  extremes_ZILs_5_ac_dc  = _PMTP.DC_busbar_split(data_busbar_ac_dc_split_5_acdc,splitted_bus_dc)
data_busbars_ac_dc_split_24_acdc, switches_couples_ac_dc_24, extremes_ZILs_24_ac_dc = _PMTP.DC_busbar_split(data_busbar_ac_dc_split_24_acdc,splitted_bus_dc)
data_busbars_ac_dc_split_39_acdc, switches_couples_ac_dc_39, extremes_ZILs_39_ac_dc = _PMTP.DC_busbar_split(data_busbar_ac_dc_split_39_acdc,splitted_bus_dc)

result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, ACPPowerModel,juniper)
result_AC_DC_24_switch_AC_DC = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_24_acdc,ACPPowerModel,juniper)
result_AC_DC_39_switch_AC_DC = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_39_acdc,ACPPowerModel,juniper)


#######################################################################################






