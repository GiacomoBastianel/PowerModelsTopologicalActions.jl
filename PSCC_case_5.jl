using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII ; const _PMTP = PowerModelsTopologicalActionsII  
using InfrastructureModels; const _IM = InfrastructureModels
using JSON
using Mosek, MosekTools
using SCIP


#######################################################################################
## Define solver ##
#######################################################################################

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)
mosek = JuMP.optimizer_with_attributes(Mosek.Optimizer)
scip = JuMP.optimizer_with_attributes(SCIP.Optimizer)

#######################################################################################
## Input data ##
#######################################################################################

test_case_5_acdc = "case5_acdc.m"
test_case_ac_only = "case5.m"

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

#=
result_dc = _PM.solve_opf(data_original_ac,DCPPowerModel,gurobi)
result_lpac = _PM.solve_opf(data_original_ac,LPACCPowerModel,gurobi)
result_ac = _PM.solve_opf(data_original_ac,ACPPowerModel,ipopt)

result_acdc_dc = _PMACDC.run_acdcopf(data_5_acdc,DCPPowerModel,gurobi; setting = s)
result_acdc_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi; setting = s)
result_acdc_ac = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)

result_ots_dc = _PM.solve_ots(data_original_ac,DCPPowerModel,gurobi)
result_ots_lpac = _PM.solve_ots(data_original_ac,LPACCPowerModel,gurobi)
result_ots_ac = _PM.solve_ots(data_original_ac,ACPPowerModel,juniper)
=#

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_5_lpac  = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi; setting = s)
result_opf_5_soc   = _PMACDC.run_acdcopf(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_opf_5_qc    = _PMACDC.run_acdcopf(data_5_acdc,QCRMPowerModel,gurobi; setting = s)

#=
result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,SOCWRPowerModel,ipopt; setting = s_dual)
result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,ipopt; setting = s_dual)

# Solving AC OTS with OTS only on the AC grid part
result_AC_ots_5    = _PMTP.run_acdcots_AC(data_5_acdc,ACPPowerModel,juniper; setting = s)

#result_AC_ots_5    = _PMTP.run_acdcots_AC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
#result_AC_ots_5    = _PMTP.run_acdcots_AC(data_5_acdc,QCRMPowerModel,gurobi; setting = s)
result_AC_ots_5_lpac    = _PMTP.run_acdcots_AC(data_5_acdc,LPACCPowerModel,gurobi; setting = s)

for (br_id,br) in data_5_acdc["branch"]
    print(result_AC_ots_5["solution"]["branch"][br_id]["br_status"],"\n")
end
for (br_id,br) in data_5_acdc["branch"]
    print(result_AC_ots_5_lpac["solution"]["branch"][br_id]["br_status"],"\n")
end

# Solving AC OTS with OTS only on the DC grid part 
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)
#result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
#result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,QCRMPowerModel,juniper; setting = s)
result_DC_ots_5_lpac    = _PMTP.run_acdcots_DC(data_5_acdc,LPACCPowerModel,juniper; setting = s)


for (br_id,br) in data_5_acdc["branchdc"]
    print(result_DC_ots_5["solution"]["branchdc"][br_id]["br_status"],"\n")
end
for (br_id,br) in data_5_acdc["branchdc"]
    print(result_DC_ots_5_lpac["solution"]["branchdc"][br_id]["br_status"],"\n")
end

# Solving AC OTS with OTS on both AC and DC grid part
result_AC_DC_ots_5_ac = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,QCRMPowerModel,gurobi; setting = s)
result_AC_DC_ots_5_lpac = _PMTP.run_acdcots_AC_DC(data_5_acdc,LPACCPowerModel,juniper; setting = s)

##############

# Showing the utilization of each branch, to be intended as absolute values
for (br_id, br) in result_opf_5_ac["solution"]["branch"]
    print("Utilization AC branch $(br_id) OPF $(br["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",br["pf"],"\n")
    print("Utilization AC branch $(br_id) AC OTS $(result_AC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ", result_AC_ots_5["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) DC OTS $(result_DC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",result_DC_ots_5["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) AC/DC OTS $(result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"],"\n")
    print("\n")
end

for (br_id, br) in result_opf_5_ac["solution"]["branchdc"]
    print("Utilization DC branch $(br_id) OPF $(br["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",br["pf"],"\n")
    print("Utilization DC branch $(br_id) AC OTS $(result_AC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_AC_ots_5["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC branch $(br_id) DC OTS $(result_DC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_DC_ots_5["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC branch $(br_id) AC/DC OTS $(result_AC_DC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"],"\n")
    print("\n")
end
=#
#######################################################################################
## Busbar splitting models ##
#######################################################################################

# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = [2, 4]


#data_busbars_ac_split_5_acdc, extremes_ZIL = _PMTP.AC_busbar_split_more_buses_fixed(data_busbars_ac_split_5_acdc,splitted_bus_ac)
#split_elements = _PMTP.elements_AC_busbar_split(data_5_acdc)

data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)

ac_1 = deepcopy(data_busbars_ac_split_5_acdc)
ac_2 = deepcopy(data_busbars_ac_split_5_acdc)
ac_3 = deepcopy(data_busbars_ac_split_5_acdc)
ac_4 = deepcopy(data_busbars_ac_split_5_acdc)
ac_5 = deepcopy(data_busbars_ac_split_5_acdc)
ac_6 = deepcopy(data_busbars_ac_split_5_acdc)
ac_7 = deepcopy(data_busbars_ac_split_5_acdc)
ac_8 = deepcopy(data_busbars_ac_split_5_acdc)

#lpac_3 = deepcopy(data_busbars_ac_split_5_acdc)
#lpac_4 = deepcopy(data_busbars_ac_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_AC_DC_5_switches_AC_ac  = _PMTP.run_acdcsw_AC(ac_1,ACPPowerModel,juniper)
result_AC_DC_5_switches_AC_ac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_2,ACPPowerModel,juniper)

ac_1_opf = deepcopy(ac_1)
for (sw_id,sw) in result_AC_DC_5_switches_AC_ac_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
    if sw["status"] > 0.5
        ac_1_opf["switch"][sw_id]["status"] = 1
    else
        ac_1_opf["switch"][sw_id]["status"] = 0
    end
end
ac_1_opf_results = _PMTP.run_acdcsw_opf(ac_1_opf,LPACCPowerModel,ipopt)


result_AC_DC_5_switches_AC_lpac  = _PMTP.run_acdcsw_AC(ac_3,LPACCPowerModel,juniper)
result_AC_DC_5_switches_AC_lpac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_4,LPACCPowerModel,gurobi)

ac_4_opf = deepcopy(ac_4)
for (sw_id,sw) in result_AC_DC_5_switches_AC_lpac_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
    if sw["status"] > 0.5
        ac_4_opf["switch"][sw_id]["status"] = 1
    else
        ac_4_opf["switch"][sw_id]["status"] = 0
    end
end
ac_4_opf_results = _PMTP.run_acdcsw_AC_opf(ac_4_opf,ACPPowerModel,ipopt)


result_AC_DC_5_switches_AC_soc  = _PMTP.run_acdcsw_AC(ac_5,SOCWRPowerModel,juniper)
result_AC_DC_5_switches_AC_soc_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_6,SOCWRPowerModel,juniper)

ac_6_opf = deepcopy(ac_6)
for (sw_id,sw) in result_AC_DC_5_switches_AC_soc_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
    if sw["status"] > 0.5
        ac_6_opf["switch"][sw_id]["status"] = 1
    else
        ac_6_opf["switch"][sw_id]["status"] = 0
    end
end
ac_6_opf_results = _PMTP.run_acdcsw_AC_opf(ac_6_opf,ACPPowerModel,ipopt)


result_AC_DC_5_switches_AC_qc  = _PMTP.run_acdcsw_AC(ac_7,QCRMPowerModel,juniper)
result_AC_DC_5_switches_AC_qc_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_8,QCRMPowerModel,juniper)
ac_8_opf = deepcopy(ac_8)
for (sw_id,sw) in result_AC_DC_5_switches_AC_qc_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
    if sw["status"] > 0.5
        ac_8_opf["switch"][sw_id]["status"] = 1
    else
        ac_8_opf["switch"][sw_id]["status"] = 0
    end
end
ac_8_opf_results = _PMTP.run_acdcsw_opf(ac_8_opf,ACPPowerModel,ipopt)


for (sw_id,sw) in result_AC_DC_5_switches_AC_lpac["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end

for (sw_id,sw) in result_AC_DC_5_switches_AC_ac["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end

for (sw_id,sw) in result_AC_DC_5_switches_AC_ac_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end


for (sw_id,sw) in result_AC_DC_5_switches_AC_lpac_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end



#=

result_AC_DC_5_switches_AC_qc  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,SOCWRPowerModel,gurobi)
result_AC_DC_5_switches_AC_soc  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,QCRMPowerModel,gurobi)

# Not necessary to reconnect all the branches -> deleted here

result_AC_DC_5_switches_AC_lpac  = _PMTP.run_acdcsw_AC(lpac_2,LPACCPowerModel,juniper)

result_AC_DC_5_switches_AC_lpac_lin_constrained = _PMTP.run_acdcsw_AC_lin_constrained_sp(lpac_3,LPACCPowerModel,ipopt)
result_AC_DC_5_switches_AC_lpac_lin = _PMTP.run_acdcsw_AC_lin(lpac_4,LPACCPowerModel,ipopt)


# If one wants to check the status of the switches. To be improved to make it easier and faster for the user to see the resulting grid topology
switches_results_ac = []
switches_results_lpac = []

for i in 1:length(result_AC_DC_5_switches_AC_ac["solution"]["switch"])
    push!(switches_results_ac,result_AC_DC_5_switches_AC_ac["solution"]["switch"]["$i"]["status"])
    push!(switches_results_lpac,result_AC_DC_5_switches_AC_lpac["solution"]["switch"]["$i"]["status"])
end
# TRY TO SEE WHETHER THE SOLUTION IS AC FEASIBLE

# AC OTS for AC/DC grid with DC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_dc = [1,2,3]
data_busbars_dc_split_5_acdc , switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_dc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, ACPPowerModel,juniper)
#result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, LPACCPowerModel,juniper)
result_AC_DC_5_switches_DC_ref  = _PMTP.run_acdcsw_DC_reformulation(data_busbars_dc_split_5_acdc, ACPPowerModel,juniper)

#result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, SOCWRPowerModel,gurobi)
#result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, QCRMPowerModel,gurobi)


# Not necessary to reconnect all the branches
#result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_dc_split_5_acdc, ACPPowerModel,juniper)


# If one wants to check the status of the switches. To be improved to make it easier and faster for the user to see the resulting grid topology
switches_results = []
for i in 1:length(result_AC_DC_5_switches_DC["solution"]["switch"])
    push!(switches_results,result_AC_DC_5_switches_DC["solution"]["switch"]["$i"]["status"])
end
=#
# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = [2,4]
splitted_bus_dc = [1,2,3]

data_busbars_ac_dc_split_5_acdc_ac_sw,  ac_switches_couples_ac_dc_5, ac_extremes_ZILs_5_ac_dc  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc_ac_dc_sw , dc_switches_couples_ac_dc_5, dc_extremes_ZILs_5_ac_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_ac_sw,splitted_bus_dc)

try_1 = deepcopy(data_busbars_ac_dc_split_5_acdc_ac_dc_sw)
try_2 = deepcopy(data_busbars_ac_dc_split_5_acdc_ac_dc_sw)
try_3 = deepcopy(data_busbars_ac_dc_split_5_acdc_ac_dc_sw)
try_4 = deepcopy(data_busbars_ac_dc_split_5_acdc_ac_dc_sw)
try_5 = deepcopy(data_busbars_ac_dc_split_5_acdc_ac_dc_sw)
try_6 = deepcopy(data_busbars_ac_dc_split_5_acdc_ac_dc_sw)
try_7 = deepcopy(data_busbars_ac_dc_split_5_acdc_ac_dc_sw)
try_8 = deepcopy(data_busbars_ac_dc_split_5_acdc_ac_dc_sw)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(try_1, ACPPowerModel,juniper)
result_AC_DC_5_switch_AC_DC_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(try_2, ACPPowerModel,juniper)

result_AC_DC_5_switch_AC_DC_lpac  = _PMTP.run_acdcsw_AC_DC(try_3, LPACCPowerModel,juniper)
result_AC_DC_5_switch_AC_DC_lpac_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(try_4, LPACCPowerModel,gurobi)

try_4_opf = deepcopy(try_4)
for (sw_id,sw) in result_AC_DC_5_switch_AC_DC_lpac_ref["solution"]["switch"]
    if sw["status"] > 0.5
        try_4_opf["switch"][sw_id]["status"] = 1
    else
        try_4_opf["switch"][sw_id]["status"] = 0
    end
end
for (sw_id,sw) in result_AC_DC_5_switch_AC_DC_lpac_ref["solution"]["dcswitch"]
    if sw["status"] > 0.5
        try_4_opf["dcswitch"][sw_id]["status"] = 1
    else
        try_4_opf["dcswitch"][sw_id]["status"] = 0
    end
end
ac_4_opf_results = _PMTP.run_acdcsw_AC_DC_opf(try_4_opf,ACPPowerModel,ipopt)

result_AC_DC_5_switch_AC_DC_soc  = _PMTP.run_acdcsw_AC_DC(try_5, SOCWRPowerModel,juniper)
result_AC_DC_5_switch_AC_DC_soc_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(try_6, SOCWRPowerModel,gurobi)

try_5_opf = deepcopy(try_5)
for (sw_id,sw) in result_AC_DC_5_switch_AC_DC_lpac_ref["solution"]["switch"]
    if sw["status"] > 0.5
        try_5_opf["switch"][sw_id]["status"] = 1
    else
        try_5_opf["switch"][sw_id]["status"] = 0
    end
end
for (sw_id,sw) in result_AC_DC_5_switch_AC_DC_lpac_ref["solution"]["dcswitch"]
    if sw["status"] > 0.5
        try_5_opf["dcswitch"][sw_id]["status"] = 1
    else
        try_5_opf["dcswitch"][sw_id]["status"] = 0
    end
end
ac_4_opf_results = _PMTP.run_acdcsw_AC_DC_opf(try_5_opf,ACPPowerModel,ipopt)


result_AC_DC_5_switch_AC_DC_qc  = _PMTP.run_acdcsw_AC_DC(try_7, QCRMPowerModel,juniper)
result_AC_DC_5_switch_AC_DC_qc_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(try_8, QCRMPowerModel,gurobi)

try_6_opf = deepcopy(try_6)
for (sw_id,sw) in result_AC_DC_5_switch_AC_DC_lpac_ref["solution"]["switch"]
    if sw["status"] > 0.5
        try_6_opf["switch"][sw_id]["status"] = 1
    else
        try_6_opf["switch"][sw_id]["status"] = 0
    end
end
for (sw_id,sw) in result_AC_DC_5_switch_AC_DC_lpac_ref["solution"]["dcswitch"]
    if sw["status"] > 0.5
        try_6_opf["dcswitch"][sw_id]["status"] = 1
    else
        try_6_opf["dcswitch"][sw_id]["status"] = 0
    end
end
ac_4_opf_results = _PMTP.run_acdcsw_AC_DC_opf(try_6_opf,ACPPowerModel,ipopt)


#result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, SOCWRPowerModel,gurobi)
#result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, QCRMPowerModel,gurobi)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, LPACCPowerModel,juniper)

# Not necessary to reconnect all the branches
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc, ACPPowerModel,juniper)
#result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc, SOCWRPowerModel,gurobi)
#result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc, QCRMPowerModel,gurobi)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc, LPACCPowerModel,juniper)

#=
# If one wants to check the status of the switches. To be improved to make it easier and faster for the user to see the resulting grid topology
switches_results = []
for i in 1:length(result_AC_DC_5_switch_AC_DC["solution"]["switch"])
    push!(switches_results,result_AC_DC_5_switch_AC_DC["solution"]["switch"]["$i"]["status"])
end
=#

# Showing the utilization of each branch, to be intended as absolute values
for (br_id, br) in result_AC_DC_5_switches_AC["solution"]["branch"]
    print("Utilization AC branch $(br_id) OPF $(result_opf_5_ac["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]*100) %","  ",result_opf_5_ac["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) AC BS $(br["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]*100) %","  ",br["pf"],"\n")
    print("Utilization AC branch $(br_id) DC BS $(result_AC_DC_5_switches_DC["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]*100) %","  ",result_AC_DC_5_switches_DC["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) AC/DC BS $(result_AC_DC_5_switch_AC_DC["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]*100) %","  ",result_AC_DC_5_switch_AC_DC["solution"]["branch"][br_id]["pf"],"\n")
    print("\n")
end

for (br_id, br) in result_AC_DC_5_switches_AC["solution"]["branchdc"]
    print("Utilization DC $(br_id) branch OPF "*"$(result_opf_5_ac["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100) %","  ",result_opf_5_ac["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC $(br_id) branch AC BS "*"$(br["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100) %","  ",br["pf"],"\n")
    print("Utilization DC $(br_id) branch DC BS "*"$(result_AC_DC_5_switches_DC["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100) %","  ",result_AC_DC_5_switches_DC["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC $(br_id) branch AC/DC BS "*"$(result_AC_DC_5_switch_AC_DC["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100) %","  ",result_AC_DC_5_switch_AC_DC["solution"]["branchdc"][br_id]["pf"],"\n")
    print("\n")
end



# Introducing the current parameter through the switches
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc_current = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = 2

data_busbars_ac_split_5_acdc_current,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc_current,splitted_bus_ac)

for (sw_id,sw) in data_busbars_ac_split_5_acdc_current["switch"]
    sw["rate_sw"] = 1.0
end

result_ac = _PMTP.run_acdcsw_AC_current(data_busbars_ac_split_5_acdc_current,ACPPowerModel,juniper)

for (sw_id,sw) in result_ac["solution"]["switch"]
    print("Switch $(sw_id)","\n")
    print("The status of the switch is $(sw["status"])","\n")
    #print("The active power through the switch is $(sw["psw_fr"])","\n")
    print("The real current through the switch is $(sw["i_sw_r_fr"])","\n")
    #print("The reactive power through the switch is $(sw["qsw_fr"])","\n")
    #print("The imaginary current through the switch is $(sw["i_sw_i_fr"])","\n")
end


# DC current through the dc switch
data_busbars_dc_split_5_acdc_current = deepcopy(data_5_acdc)

splitted_bus_dc = 2
data_busbars_dc_split_5_acdc_current , dc_switches_couples_ac_dc_5, dc_extremes_ZILs_5_ac_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc_current,splitted_bus_dc)

for (sw_id,sw) in data_busbars_dc_split_5_acdc_current["dcswitch"]
    sw["rate_sw"] = 10.0
end

result_dc = _PMTP.run_acdcsw_DC_current(data_busbars_dc_split_5_acdc_current,ACPPowerModel,juniper)

for (sw_id,sw) in result_dc["solution"]["dcswitch"]
    print("Switch $(sw_id)","\n")
    print("The status of the switch is $(sw["status"])","\n")
    #print("The active power through the switch is $(sw["psw_fr"])","\n")
    print("The real current through the switch is $(sw["i_sw_dc_fr"])","\n")
    #print("The reactive power through the switch is $(sw["qsw_fr"])","\n")
    #print("The imaginary current through the switch is $(sw["i_sw_i_fr"])","\n")
end

# AC and DC switches currents
data_busbars_ac_split_5_acdc_current = deepcopy(data_5_acdc)
splitted_bus_ac = 2
splitted_bus_dc = 2

data_busbars_ac_dc_split_5_acdc_current = deepcopy(data_5_acdc)

data_busbars_ac_dc_split_5_acdc_current,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_current,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc_current , switches_couples_dc_5, extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_current,splitted_bus_dc)


for (sw_id,sw) in data_busbars_ac_dc_split_5_acdc_current["switch"]
    sw["rate_sw"] = 1.0
end
for (sw_id,sw) in data_busbars_ac_dc_split_5_acdc_current["dcswitch"]
    sw["rate_sw"] = 10.0
end

result_ac_dc = _PMTP.run_acdcsw_AC_DC_current(data_busbars_ac_dc_split_5_acdc_current,ACPPowerModel,juniper)

for (sw_id,sw) in result_ac_dc["solution"]["switch"]
    print("Switch $(sw_id)","\n")
    print("The status of the switch is $(sw["status"])","\n")
    #print("The active power through the switch is $(sw["psw_fr"])","\n")
    print("The real current through the switch is $(sw["i_sw_r_fr"])","\n")
    #print("The reactive power through the switch is $(sw["qsw_fr"])","\n")
    #print("The imaginary current through the switch is $(sw["i_sw_i_fr"])","\n")
end

for (sw_id,sw) in result_ac_dc["solution"]["dcswitch"]
    print("Switch $(sw_id)","\n")
    print("The status of the switch is $(sw["status"])","\n")
    #print("The active power through the switch is $(sw["psw_fr"])","\n")
    print("The real current through the switch is $(sw["i_sw_dc_fr"])","\n")
    #print("The reactive power through the switch is $(sw["qsw_fr"])","\n")
    #print("The imaginary current through the switch is $(sw["i_sw_i_fr"])","\n")
end
