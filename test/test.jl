using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
using HiGHS

# Define solver
ipopt_solver = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt_solver, "mip_solver" => highs, "time_limit" => 7200)


## Input data
test_case = "case5.m"
test_case_sw = "case5_sw.m"
test_case_acdc = "case5_acdc.m"


## Parsing with Powermodels
data_file = joinpath(@__DIR__,"./data_sources",test_case)
data_file_sw = joinpath(@__DIR__,"data_sources",test_case_sw)
data_file_acdc = joinpath(@__DIR__,"data_sources",test_case_acdc)

data_original = _PM.parse_file(data_file)
data = deepcopy(data_original)
data_busbar_split = deepcopy(data_original)
data_sw = _PM.parse_file(data_file_sw)

data_original_acdc = _PM.parse_file(data_file_acdc)
data_acdc = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_acdc)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)
data_dc_busbar_split = deepcopy(data_acdc)

# AC OTS with PowerModels for AC grid
result_ots = _PM.solve_ots(data,ACPPowerModel,juniper)
#=
# Splitting the selected bus
grid, switch_couples = _PMTP.busbar_split_creation(data_busbar_split,1)

# AC OTS with PowerModels for AC grid with fixed switches
result_switch_fixed = _PM._solve_opf_sw(data_sw,ACPPowerModel,juniper)

# AC OTS with PowerModels and handmade for AC grid with switches state as decision variable
# Infeasible
result_PM_switch = _PM._solve_oswpf(grid,ACPPowerModel,juniper)
result_switch = _PMTP.solve_ots_switch(grid,ACPPowerModel,juniper)

# DC linearization working
result_PM_switch_DC_fixed = _PM._solve_opf_sw(grid,DCPPowerModel,juniper)
result_PM_switch_DC = _PM._solve_oswpf(grid,DCPPowerModel,juniper)
result_switch_DC = _PMTP.solve_ots_switch(grid,DCPPowerModel,juniper)
result_switch_DC_busbar_splitting = _PMTP._solve_oswpf_busbar_splitting(grid,DCPPowerModel,juniper)

# SOC relaxation working
result_PM_switch_SOC_fixed = _PM._solve_opf_sw(grid,SOCWRPowerModel,juniper)
result_PM_switch_SOC = _PM._solve_oswpf(grid,SOCWRPowerModel,juniper)
result_switch_SOC = _PMTP.solve_ots_switch(grid,SOCWRPowerModel,juniper)
result_switch_SOC_busbar_splitting = _PMTP._solve_oswpf_busbar_splitting(grid,SOCWRPowerModel,juniper)

# QC relaxation
result_PM_switch_QC_fixed = _PM._solve_opf_sw(grid,QCRMPowerModel,juniper)
result_PM_switch_QC = _PM._solve_oswpf(grid,QCRMPowerModel,juniper)
result_switch_QC = _PMTP.solve_ots_switch(grid,QCRMPowerModel,juniper)
result_switch_QC_busbar_splitting = _PMTP._solve_oswpf_busbar_splitting(grid,QCRMPowerModel,juniper)

# AC OPF for ACDC grid
result_opf = _PMACDC.run_acdcopf(data_acdc,ACPPowerModel,juniper)

# Solving AC OTS with OTS only on the DC grid part 
result_homemade_ots_DC = _PMTP.run_acdcots_DC(data_acdc,ACPPowerModel,juniper)

# Solving AC OTS with OTS only on the AC grid part
result_homemade_ots = _PMTP.run_acdcots_AC(data_acdc,ACPPowerModel,juniper)

# Solving AC OTS with OTS on both AC and DC grid part
result_homemade_ots_AC_DC = _PMTP.run_acdcots_AC_DC(data_acdc,ACPPowerModel,juniper)


grid_dc, switch_couples = _PMTP.busbar_split_creation(data_dc_busbar_split,1)


# AC OTS with handmade for AC/DC grid with switches state as decision variable

result_PM_AC_DC_switch_AC = _PM._solve_oswpf(grid,ACPPowerModel,juniper)
result_AC_DC_switch_AC = _PMTP._solve_oswpf_AC_busbar_splitting_AC_DC(grid_dc,QCRMPowerModel,juniper)
=#


data_acdc["dcswitch"] = Dict{String,Any}()

data_busbar_split_dc, dc_switch_couples = _PMTP.busbar_split_creation_dc(data_acdc,1)

dc_switch_couples = _PMTP.compute_couples_of_dcswitches(data_acdc)

data_busbar_split_dc["dc_switch_couples"] = deepcopy(dc_switch_couples)

result_AC_DC_switch_DC = _PMTP._solve_oswpf_DC_busbar_splitting_AC_DC(data_busbar_split_dc,ACPPowerModel,juniper)


bus_arcs_sw_dc = Dict{String,Any}()

bus_arcs_sw_dc = Dict((i, Tuple{Int,Int,Int}[]) for (i,bus) in data_busbar_split_dc["busdc"])
for (l,i,j) in data_busbar_split_dc[:arcs_sw_dc]
    push!(bus_arcs_sw_dc[i], (l,i,j))
end
nw_ref[:bus_arcs_sw_dc] = bus_arcs_sw_dc
