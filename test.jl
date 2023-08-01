using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII; const _PMTP = PowerModelsTopologicalActionsII

#######################################################################################
## Define solver ##
#######################################################################################

ipopt_solver = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt_solver, "mip_solver" => highs, "time_limit" => 7200)

#######################################################################################
## Input data ##
#######################################################################################

test_case = "case5.m"
test_case_sw = "case5_sw.m"
test_case_acdc = "case5_acdc.m"

#######################################################################################
## Parsing input data ##
#######################################################################################

data_file_acdc = joinpath(@__DIR__,"data_sources",test_case_acdc)

data_original_acdc = _PM.parse_file(data_file_acdc)
data_acdc = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_acdc)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)
data_dc_busbar_split = deepcopy(data_acdc)

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################

# AC OTS with PowerModels for AC grid
result_ots = _PM.solve_ots(data,ACPPowerModel,juniper)

# AC OPF for ACDC grid
result_opf = _PMACDC.run_acdcopf(data_acdc,ACPPowerModel,juniper)

# Solving AC OTS with OTS only on the DC grid part 
result_homemade_ots_DC = _PMTP.run_acdcots_DC(data_acdc,ACPPowerModel,juniper)

# Solving AC OTS with OTS only on the AC grid part
result_homemade_ots = _PMTP.run_acdcots_AC(data_acdc,ACPPowerModel,juniper)

# Solving AC OTS with OTS on both AC and DC grid part
result_homemade_ots_AC_DC = _PMTP.run_acdcots_AC_DC(data_acdc,ACPPowerModel,juniper)

#######################################################################################
## Busbar splitting models ##
#######################################################################################

data_sw, switch_couples, extremes_ZIL = _PMTP.AC_busbar_split(data_acdc,1)

# AC OTS for AC/DC grid with AC switches state as decision variable
result_PM_AC_DC_switch_AC = _PM._solve_oswpf(data_sw,DCPPowerModel,juniper)
result_AC_DC_switch_AC = _PMTP.run_acdcsw_AC(data_sw,ACPPowerModel,juniper)

# AC OTS for AC/DC grid with DC switches state as decision variable
data_base_sw_dc = deepcopy(data_dc_busbar_split)
data_sw_dc, switch_dccouples, extremes_ZIL_dc = _PMTP.DC_busbar_split(data_base_sw_dc,1)

result_AC_DC_switch_DC = _PMTP.run_acdcsw_DC(data_sw_dc,ACPPowerModel,juniper)


# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_base_sw_acdc = deepcopy(data_dc_busbar_split)
data_sw_acdc, switch_couples, extremes_ZIL = _PMTP.AC_busbar_split(data_base_sw_acdc,1)
data_sw_acdc, switch_dccouples, extremes_ZIL_dc = _PMTP.DC_busbar_split(data_sw_acdc,1)

result_AC_DC_switch_AC_DC = _PMTP.run_acdcsw_AC_DC(data_sw_acdc,ACPPowerModel,juniper)

#######################################################################################


