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

test_case_39 = "case39_acdc.m"


#######################################################################################
## Parsing input data ##
#######################################################################################

data_file_acdc = joinpath(@__DIR__,"data_sources",test_case_acdc)

data_original_acdc = _PM.parse_file(data_file_acdc)
data_acdc = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_acdc)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)
#s = Dict("output" => Dict("branch_flows" => true,"report_duals" => true), "conv_losses_mp" => true)
data_dc_busbar_split = deepcopy(data_acdc)
data_file = joinpath(@__DIR__,"data_sources",test_case)
data_original = _PM.parse_file(data_file_acdc)

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf = _PMACDC.run_acdcopf(data_acdc,ACPPowerModel,juniper)
result_soc = _PMACDC.run_acdcopf(data_acdc,SOCWRPowerModel,juniper)
result_qc = _PMACDC.run_acdcopf(data_acdc,QCRMPowerModel,juniper)

# AC OTS with PowerModels for AC grid
#result_ots = _PM.solve_ots(data,ACPPowerModel,juniper)
#result_soc = _PM.solve_ots(data,SOCWRPowerModel,juniper)
#result_qc  = _PM.solve_ots(data,QCRMPowerModel,juniper)

# Solving AC OTS with OTS only on the DC grid part 
result_homemade_ots_DC = _PMTP.run_acdcots_DC(data_acdc,ACPPowerModel,juniper)
result_homemade_ots_soc = _PMTP.run_acdcots_DC(data_acdc,SOCWRPowerModel,juniper)
#result_homemade_ots_soc = _PMTP.run_acdcots_DC(data_acdc,SOCBFPowerModel,juniper)
result_homemade_ots_qc = _PMTP.run_acdcots_DC(data_acdc,QCRMPowerModel,juniper)

# Solving AC OTS with OTS only on the AC grid part
result_homemade_ots = _PMTP.run_acdcots_AC(data_acdc,ACPPowerModel,juniper)
result_homemade_ots_soc = _PMTP.run_acdcots_AC(data_acdc,SOCWRPowerModel,juniper)
result_homemade_ots_qc = _PMTP.run_acdcots_AC(data_acdc,QCRMPowerModel,juniper)

for (br_id,br) in result_homemade_ots_soc["solution"]["branchdc"]
    print(br_id,"__",br["br_status"],"\n")
end
for (br_id,br) in result_homemade_ots_qc["solution"]["branchdc"]
    print(br_id,"__",br["br_status"],"\n")
end

# Solving AC OTS with OTS on both AC and DC grid part
result_homemade_ots_AC_DC = _PMTP.run_acdcots_AC_DC(data_acdc,ACPPowerModel,juniper)
result_homemade_ots_AC_DC_soc = _PMTP.run_acdcots_AC_DC(data_acdc,SOCWRPowerModel,juniper)
result_homemade_ots_AC_DC_qc = _PMTP.run_acdcots_AC_DC(data_acdc,QCRMPowerModel,juniper)

#######################################################################################
## Busbar splitting models ##
#######################################################################################
# AC OTS for AC/DC grid with AC switches state as decision variable
data_busbar_split_acdc = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_busbar_split_acdc)

splitted_buses = [4,5]

data_sw, switch_couples, extremes_ZIL = _PMTP.AC_busbar_split(data_busbar_split_acdc,1)
result_PM_AC_DC_switch_AC = _PM._solve_oswpf(data_sw,DCPPowerModel,juniper)


data_busbar_split_acdc = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_busbar_split_acdc)

data_sw, switch_couples, extremes_ZIL = _PMTP.AC_busbar_split(data_busbar_split_acdc,1)
result_AC_DC_switch_AC = _PMTP.run_acdcsw_AC(data_sw,ACPPowerModel,juniper)

# AC OTS for AC/DC grid with DC switches state as decision variable
data_base_sw_dc = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_base_sw_dc)

data_sw_dc, switch_dccouples, extremes_ZIL_dc = _PMTP.DC_busbar_split(data_base_sw_dc,1)

result_AC_DC_switch_DC = _PMTP.run_acdcsw_DC(data_sw_dc,ACPPowerModel,juniper)


# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_base_sw_acdc = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_base_sw_acdc)

data_sw_acdc, switch_couples, extremes_ZIL = _PMTP.AC_busbar_split(data_base_sw_acdc,1)
data_sw_acdc, switch_dccouples, extremes_ZIL_dc = _PMTP.DC_busbar_split(data_sw_acdc,1)

result_AC_DC_switch_AC_DC = _PMTP.run_acdcsw_AC_DC(data_sw_acdc,ACPPowerModel,juniper)

#######################################################################################

data_ac_busbar_split_more_buses = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_ac_busbar_split_more_buses)
splitted_buses = [4,5] #4,5 seems to be beneficial for the case_5_acdc.m
splitted_buses_dc = [1,2]

data_sw_more, switch_couples_more, extremes_ZIL_more  = _PMTP.AC_busbar_split_more_buses(data_ac_busbar_split_more_buses,splitted_buses)


#result_AC_DC_switch_AC_sw = _PMTP.run_acdcsw_AC(data_sw,ACPPowerModel,juniper)
result_AC_DC_switch_AC = _PMTP.run_acdcsw_AC(data_sw_more,ACPPowerModel,juniper)



data_ac_busbar_split_more_buses_dc = deepcopy(data_acdc)
data_sw_more_dc, switch_couples_dc_more, extremes_ZIL_dc_more  = _PMTP.DC_busbar_split_more_buses(data_ac_busbar_split_more_buses_dc,splitted_buses)
result_AC_DC_switch_DC_more = _PMTP.run_acdcsw_DC(data_sw_more_dc,ACPPowerModel,juniper)






arcs_dcgrid_from = [(i,branch["fbusdc"],branch["tbusdc"]) for (i,branch) in data_acdc["branchdc"]]
arcs_dcgrid_to   = [(i,branch["tbusdc"],branch["fbusdc"]) for (i,branch) in data_acdc["branchdc"]]
arcs_dcgrid = [arcs_dcgrid_from; arcs_dcgrid_to]


bus_arcs_dcgrid = Dict([(bus["busdc_i"], []) for (i,bus) in data_acdc["busdc"]])
for (l,i,j) in arcs_dcgrid
    push!(bus_arcs_dcgrid[i], (l,i,j))
end
bus_arcs_dcgrid[2][1][1]


bus_loads = Dict((bus["index"],[]) for (i,bus) in data_acdc["bus"])
for (i, load) in data_acdc["load"]
    push!(bus_loads[load["load_bus"]], i)#parse(Int64,i))
end


pd = Dict(k => data_acdc["bus"][k]["pd"] for k in bus_loads)
qd = Dict(k => _PM.ref(pm, nw, :load, k, "qd") for k in bus_loads)

for k in bus_loads
    print(k,"\n")
end