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
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => highs, "time_limit" => 36000)
mosek = JuMP.optimizer_with_attributes(Mosek.Optimizer)

#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

test_case_5_acdc = "case5_acdc.m"
data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)

data_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

data_original_5_acdc = _PM.parse_file(data_file_5_acdc)
_PMACDC.process_additional_data!(data_original_5_acdc)

#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi; setting = s)


##############
# Showing the utilization of each branch, to be intended as absolute values
for (br_id, br) in result_opf_ac["solution"]["branch"]
    println("AC Branch $(br_id) with f_bus $(data_5_acdc["branch"][br_id]["f_bus"]) and t_bus $(data_5_acdc["branch"][br_id]["t_bus"])")
    println("Utilization AC branch $(br_id) OPF $(br["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",br["pf"])
    #print("Utilization AC branch $(br_id) AC/DC OTS - AC $(result_ots_ac["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ", result_ots_ac["solution"]["branch"][br_id]["pf"],"\n")
end


for (br_id, br) in result_opf_ac["solution"]["branchdc"]
    println("DC Branch $(br_id) with f_bus $(data_5_acdc["branchdc"][br_id]["fbusdc"]) and t_bus $(data_5_acdc["branchdc"][br_id]["tbusdc"])")
    #println("Utilization DC branch $(br_id) AC/DC OTS - AC $(result_ots_ac["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ", result_ots_ac["solution"]["branchdc"][br_id]["pf"])
    print("\n")
end


#######################################################################################
## Busbar splitting models ##
#######################################################################################

###### AC Busbar splitting models ######
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = 2

split_elements = _PMTP.elements_AC_busbar_split(data_5_acdc)
data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)

ac_bs_ac = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac = deepcopy(data_busbars_ac_split_5_acdc)

ac_bs_ac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac_ref = deepcopy(data_busbars_ac_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_AC_ac  = _PMTP.run_acdcsw_AC(ac_bs_ac,ACPPowerModel,juniper)
result_switches_AC_lpac  = _PMTP.run_acdcsw_AC(ac_bs_lpac,LPACCPowerModel,juniper)
result_switches_AC_ac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_ac_ref,ACPPowerModel,juniper)
result_switches_AC_lpac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_lpac_ref,LPACCPowerModel,gurobi)

# Feasibility check
feasibility_check_AC_BS_opf_lpac_ref_status = deepcopy(data_busbars_ac_split_5_acdc)
for (br_id, br) in result_switches_AC_soc_ref["solution"]["switch"]
    feasibility_check_AC_BS_opf_lpac_ref_status["switch"][br_id]["status"] = deepcopy(result_switches_AC_lpac_ref["solution"]["switch"][br_id]["status"])
end
feasibility_check_opf_lpac_ref_status = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_lpac_ref_status,ACPPowerModel,ipopt; setting = s)

###### DC Busbar splitting models ######
# DC BS for AC/DC grid with DC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_dc = 1

data_busbars_dc_split_5_acdc,  switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc,splitted_bus_dc)

dc_bs_ac = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_lpac = deepcopy(data_busbars_dc_split_5_acdc)

dc_bs_ac_ref = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_lpac_ref = deepcopy(data_busbars_dc_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_DC_ac  = _PMTP.run_acdcsw_DC(dc_bs_ac,ACPPowerModel,juniper)
result_switches_DC_lpac  = _PMTP.run_acdcsw_DC(dc_bs_lpac,LPACCPowerModel,juniper)

result_switches_DC_ac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_ac_ref,ACPPowerModel,juniper)
result_switches_DC_lpac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_lpac_ref,LPACCPowerModel,gurobi)

##############
# AC feasibility checks for AC busbar splitting
feasibility_check_DC_BS_opf_lpac = deepcopy(data_busbars_dc_split_5_acdc)
feasibility_check_DC_BS_opf_lpac_ref = deepcopy(data_busbars_dc_split_5_acdc)

for (br_id, br) in result_switches_DC_ac["solution"]["dcswitch"]
    feasibility_check_DC_BS_opf_lpac["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_lpac["solution"]["dcswitch"][br_id]["status"])
end
for (br_id, br) in result_switches_DC_ac["solution"]["dcswitch"]
    feasibility_check_DC_BS_opf_lpac_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_lpac_ref["solution"]["dcswitch"][br_id]["status"])
end
feasibility_check_opf_lpac = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_lpac,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac_ref = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_lpac_ref,ACPPowerModel,ipopt; setting = s)



###### AC/DC Busbar splitting models ######
# AC/DC BS for AC/DC grid with AC and DC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_dc_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = 2
splitted_bus_dc = collect(1:3)

data_busbars_ac_dc_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc,  switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_dc)

ac_dc_bs_ac = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_lpac = deepcopy(data_busbars_ac_dc_split_5_acdc)

ac_dc_bs_ac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_lpac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_AC_DC_ac  = _PMTP.run_acdcsw_AC_DC(ac_dc_bs_ac,ACPPowerModel,juniper)
result_switches_AC_DC_lpac  = _PMTP.run_acdcsw_AC_DC(ac_dc_bs_lpac,LPACCPowerModel,juniper)

result_switches_AC_DC_ac_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(ac_dc_bs_ac_ref,ACPPowerModel,juniper)
result_switches_AC_DC_lpac_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(ac_dc_bs_lpac_ref,LPACCPowerModel,gurobi)


print_connections_DC_switches(result_switches_AC_DC_ac_ref,ac_dc_bs_ac_ref)

##############
# AC feasibility checks for AC busbar splitting
feasibility_check_AC_DC_BS_opf_lpac = deepcopy(data_busbars_ac_dc_split_5_acdc)
feasibility_check_AC_DC_BS_opf_lpac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)

for (br_id, br) in result_switches_AC_DC_ac["solution"]["switch"]
    feasibility_check_AC_DC_BS_opf_lpac["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac["solution"]["switch"][br_id]["status"])
end
for (br_id, br) in result_switches_AC_DC_soc_ref["solution"]["switch"]
    feasibility_check_AC_DC_BS_opf_lpac_ref["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac_ref["solution"]["switch"][br_id]["status"])
end

for (br_id, br) in result_switches_AC_DC_soc_ref["solution"]["dcswitch"]
    feasibility_check_AC_DC_BS_opf_soc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_soc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_qc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_qc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_lpac_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac_ref["solution"]["dcswitch"][br_id]["status"])
end

feasibility_check_opf_lpac = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_lpac,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac_ref = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_lpac_ref,ACPPowerModel,ipopt; setting = s)

