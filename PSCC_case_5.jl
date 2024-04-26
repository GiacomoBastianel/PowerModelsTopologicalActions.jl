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
test_case_ac = "case24.m"
#=
test_case_5_acdc = "case67.m"
test_case_5_acdc = "case39_acdc.m"
test_case_5_acdc = "case3120sp_mcdc.m"
test_case_5_acdc = "cigre_b4_dc_grid.m"
=#
#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_original_5_acdc = _PM.parse_file(data_file_5_acdc)

data_5_acdc = deepcopy(data_original_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)


data_file_ac = joinpath(@__DIR__,"data_sources",test_case_ac)
data_original_ac = _PM.parse_file(data_file_ac)

data_ac = deepcopy(data_original_ac)

#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_opf_soc = _PMACDC.run_acdcopf(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_opf_qc = _PMACDC.run_acdcopf(data_5_acdc,QCRMPowerModel,gurobi; setting = s)
result_opf_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,ipopt; setting = s)

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC/DC OTS simulations
result_ots_ac = _PMTP.run_acdcots_AC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_ots_soc = _PMTP.run_acdcots_AC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_ots_qc = _PMTP.run_acdcots_AC(data_5_acdc,QCRMPowerModel,gurobi; setting = s)
result_ots_lpac = _PMTP.run_acdcots_AC(data_5_acdc,LPACCPowerModel,gurobi; setting = s)

result_ots_ac = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_ots_soc = _PMTP.run_acdcots_DC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_ots_qc = _PMTP.run_acdcots_DC(data_5_acdc,QCRMPowerModel,gurobi; setting = s)
result_ots_lpac = _PMTP.run_acdcots_DC(data_5_acdc,LPACCPowerModel,gurobi; setting = s)

result_ots_ac = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_ots_soc = _PMTP.run_acdcots_AC_DC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_ots_qc = _PMTP.run_acdcots_AC_DC(data_5_acdc,QCRMPowerModel,gurobi; setting = s)
result_ots_lpac = _PMTP.run_acdcots_AC_DC(data_5_acdc,LPACCPowerModel,gurobi; setting = s)

##############
# Showing the utilization of each branch, to be intended as absolute values
for (br_id, br) in result_opf_ac["solution"]["branch"]
    print("AC Branch $(br_id) with f_bus $(data_5_acdc["branch"][br_id]["f_bus"]) and t_bus $(data_5_acdc["branch"][br_id]["t_bus"])")
    print("Utilization AC branch $(br_id) OPF $(br["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",br["pf"],"\n")
    print("Utilization AC branch $(br_id) AC/DC OTS - AC $(result_ots_ac["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ", result_ots_ac["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) AC/DC OTS - SOC $(result_ots_soc["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",result_ots_soc["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) AC/DC OTS - QC $(result_ots_qc["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",result_ots_qc["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) AC/DC OTS - LPAC $(result_ots_lpac["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",result_ots_lpac["solution"]["branch"][br_id]["pf"],"\n")
    print("\n")
end


for (br_id, br) in result_ots_ac["solution"]["branchdc"]
    print("DC Branch $(br_id) with f_bus $(data_5_acdc["branchdc"][br_id]["fbusdc"]) and t_bus $(data_5_acdc["branchdc"][br_id]["tbusdc"])")
    print("Utilization DC branch $(br_id) AC/DC OTS - AC $(result_ots_ac["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ", result_ots_ac["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC branch $(br_id) AC/DC OTS - SOC $(result_ots_soc["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_ots_soc["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC branch $(br_id) AC/DC OTS - QC $(result_ots_qc["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_ots_qc["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC branch $(br_id) AC/DC OTS - LPAC $(result_ots_lpac["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_ots_lpac["solution"]["branchdc"][br_id]["pf"],"\n")
    print("\n")
end

##############
# AC feasibility checks
data_5_acdc_soc  = deepcopy(data_5_acdc)
data_5_acdc_qc   = deepcopy(data_5_acdc)
data_5_acdc_lpac = deepcopy(data_5_acdc)
for (br_id, br) in result_opf_ac["solution"]["branch"]
    data_5_acdc_soc["branch"][br_id]["br_status"] = deepcopy(result_ots_soc["solution"]["branch"][br_id]["br_status"])
    data_5_acdc_qc["branch"][br_id]["br_status"] = deepcopy(result_ots_qc["solution"]["branch"][br_id]["br_status"])
    data_5_acdc_lpac["branch"][br_id]["br_status"] = deepcopy(result_ots_lpac["solution"]["branch"][br_id]["br_status"])
end
for (br_id, br) in result_opf_ac["solution"]["branchdc"]
    data_5_acdc_soc["branchdc"][br_id]["br_status"] = deepcopy(result_ots_soc["solution"]["branchdc"][br_id]["br_status"])
    data_5_acdc_qc["branchdc"][br_id]["br_status"] = deepcopy(result_ots_qc["solution"]["branchdc"][br_id]["br_status"])
    data_5_acdc_lpac["branchdc"][br_id]["br_status"] = deepcopy(result_ots_lpac["solution"]["branchdc"][br_id]["br_status"])
end
feasibility_check_opf_soc = _PMACDC.run_acdcopf(data_5_acdc_soc,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_qc  = _PMACDC.run_acdcopf(data_5_acdc_qc,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac = _PMACDC.run_acdcopf(data_5_acdc_lpac,ACPPowerModel,ipopt; setting = s)



#######################################################################################
## Busbar splitting models ##
#######################################################################################
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = collect(1:5)
splitted_bus_ac = [2,4]

#data_busbars_ac_split_5_acdc, extremes_ZIL = _PMTP.AC_busbar_split_more_buses_fixed(data_busbars_ac_split_5_acdc,splitted_bus_ac)
#split_elements = _PMTP.elements_AC_busbar_split(data_5_acdc)

data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)

ac_bs_ac = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_soc = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_qc = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac = deepcopy(data_busbars_ac_split_5_acdc)

ac_bs_ac_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_soc_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_qc_ref = deepcopy(data_busbars_ac_split_5_acdc)
ac_bs_lpac_ref = deepcopy(data_busbars_ac_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_AC_ac  = _PMTP.run_acdcsw_AC(ac_bs_ac,ACPPowerModel,juniper)
result_switches_AC_soc  = _PMTP.run_acdcsw_AC(ac_bs_soc,SOCWRPowerModel,juniper)
result_switches_AC_qc  = _PMTP.run_acdcsw_AC(ac_bs_qc,QCRMPowerModel,juniper)
result_switches_AC_lpac  = _PMTP.run_acdcsw_AC(ac_bs_lpac,LPACCPowerModel,juniper)

result_switches_AC_ac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_ac_ref,ACPPowerModel,juniper)
result_switches_AC_soc_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_soc_ref,SOCWRPowerModel,gurobi)
result_switches_AC_qc_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_qc_ref,QCRMPowerModel,gurobi)
result_switches_AC_lpac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_lpac_ref,LPACCPowerModel,gurobi)

##############
# AC feasibility checks for AC busbar splitting
feasibility_check_AC_BS_opf_soc  = deepcopy(data_busbars_ac_split_5_acdc)
feasibility_check_AC_BS_opf_qc   = deepcopy(data_busbars_ac_split_5_acdc)
feasibility_check_AC_BS_opf_lpac = deepcopy(data_busbars_ac_split_5_acdc)

feasibility_check_AC_BS_opf_soc_ref  = deepcopy(data_busbars_ac_split_5_acdc)
feasibility_check_AC_BS_opf_qc_ref   = deepcopy(data_busbars_ac_split_5_acdc)
feasibility_check_AC_BS_opf_lpac_ref = deepcopy(data_busbars_ac_split_5_acdc)

for (br_id, br) in result_switches_AC_ac["solution"]["switch"]
    feasibility_check_AC_BS_opf_soc["switch"][br_id]["status"] = deepcopy(result_switches_AC_soc["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_BS_opf_qc["switch"][br_id]["status"] = deepcopy(result_switches_AC_qc["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_BS_opf_lpac["switch"][br_id]["status"] = deepcopy(result_switches_AC_lpac["solution"]["switch"][br_id]["status"])
end
for (br_id, br) in result_switches_AC_ac["solution"]["switch"]
    feasibility_check_AC_BS_opf_soc_ref["switch"][br_id]["status"] = deepcopy(result_switches_AC_soc_ref["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_BS_opf_qc_ref["switch"][br_id]["status"] = deepcopy(result_switches_AC_qc_ref["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_BS_opf_lpac_ref["switch"][br_id]["status"] = deepcopy(result_switches_AC_lpac_ref["solution"]["switch"][br_id]["status"])
end
feasibility_check_opf_soc = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_soc,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_qc = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_qc,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_lpac,ACPPowerModel,ipopt; setting = s)

feasibility_check_opf_soc_ref = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_soc_ref,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_qc_ref = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_qc_ref,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac_ref = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_lpac_ref,ACPPowerModel,ipopt; setting = s)


for (sw_id,sw) in result_switches_AC_soc_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end
for (sw_id,sw) in result_switches_AC_qc_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end
for (sw_id,sw) in result_switches_AC_lpac_ref["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end




# DC BS for AC/DC grid with DC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_dc_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_dc = collect(1:3)

data_busbars_dc_split_5_acdc,  switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc,splitted_bus_dc)

dc_bs_ac = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_soc = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_qc = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_lpac = deepcopy(data_busbars_dc_split_5_acdc)

dc_bs_ac_ref = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_soc_ref = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_qc_ref = deepcopy(data_busbars_dc_split_5_acdc)
dc_bs_lpac_ref = deepcopy(data_busbars_dc_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_DC_ac  = _PMTP.run_acdcsw_DC(dc_bs_ac,ACPPowerModel,juniper)
result_switches_DC_soc  = _PMTP.run_acdcsw_DC(dc_bs_soc,SOCWRPowerModel,juniper)
result_switches_DC_qc  = _PMTP.run_acdcsw_DC(dc_bs_qc,QCRMPowerModel,juniper)
result_switches_DC_lpac  = _PMTP.run_acdcsw_DC(dc_bs_lpac,LPACCPowerModel,juniper)

result_switches_DC_ac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_ac_ref,ACPPowerModel,juniper)
result_switches_DC_soc_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_soc_ref,SOCWRPowerModel,gurobi)
result_switches_DC_qc_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_qc_ref,QCRMPowerModel,gurobi)
result_switches_DC_lpac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_lpac_ref,LPACCPowerModel,gurobi)

##############
# AC feasibility checks for AC busbar splitting
feasibility_check_DC_BS_opf_soc  = deepcopy(data_busbars_dc_split_5_acdc)
feasibility_check_DC_BS_opf_qc   = deepcopy(data_busbars_dc_split_5_acdc)
feasibility_check_DC_BS_opf_lpac = deepcopy(data_busbars_dc_split_5_acdc)

feasibility_check_DC_BS_opf_soc_ref  = deepcopy(data_busbars_dc_split_5_acdc)
feasibility_check_DC_BS_opf_qc_ref   = deepcopy(data_busbars_dc_split_5_acdc)
feasibility_check_DC_BS_opf_lpac_ref = deepcopy(data_busbars_dc_split_5_acdc)

for (br_id, br) in result_switches_DC_ac["solution"]["dcswitch"]
    feasibility_check_DC_BS_opf_soc["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_soc["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_DC_BS_opf_qc["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_qc["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_DC_BS_opf_lpac["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_lpac["solution"]["dcswitch"][br_id]["status"])
end
for (br_id, br) in result_switches_DC_ac["solution"]["dcswitch"]
    feasibility_check_DC_BS_opf_soc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_soc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_DC_BS_opf_qc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_qc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_DC_BS_opf_lpac_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_DC_lpac_ref["solution"]["dcswitch"][br_id]["status"])
end
feasibility_check_opf_soc = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_soc,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_qc = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_qc,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_lpac,ACPPowerModel,ipopt; setting = s)

feasibility_check_opf_soc_ref = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_soc_ref,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_qc_ref = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_qc_ref,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac_ref = _PMTP.run_acdcsw_DC_opf(feasibility_check_DC_BS_opf_lpac_ref,ACPPowerModel,ipopt; setting = s)


for (sw_id,sw) in result_switches_DC_soc_ref["solution"]["dcswitch"]
    print([sw_id,sw["status"]],"\n")
end
for (sw_id,sw) in result_switches_DC_qc_ref["solution"]["dcswitch"]
    print([sw_id,sw["status"]],"\n")
end
for (sw_id,sw) in result_switches_Dlpac_ref["solution"]["dcswitch"]
    print([sw_id,sw["status"]],"\n")
end



# AC?DC BS for AC/DC grid with AC and DC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_dc_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = collect(1:5)
splitted_bus_dc = collect(1:3)

data_busbars_ac_dc_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc,  switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_dc)

ac_dc_bs_ac = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_soc = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_qc = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_lpac = deepcopy(data_busbars_ac_dc_split_5_acdc)

ac_dc_bs_ac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_soc_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_qc_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)
ac_dc_bs_lpac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_switches_AC_DC_ac  = _PMTP.run_acdcsw_AC_DC(ac_dc_bs_ac,ACPPowerModel,juniper)
result_switches_AC_DC_soc  = _PMTP.run_acdcsw_AC_DC(ac_dc_bs_soc,SOCWRPowerModel,juniper)
result_switches_AC_DC_qc  = _PMTP.run_acdcsw_AC_DC(ac_dc_bs_qc,QCRMPowerModel,juniper)
result_switches_AC_DC_lpac  = _PMTP.run_acdcsw_AC_DC(ac_dc_bs_lpac,LPACCPowerModel,juniper)

result_switches_AC_DC_ac_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(ac_dc_bs_ac_ref,ACPPowerModel,juniper)
result_switches_AC_DC_soc_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(ac_dc_bs_soc_ref,SOCWRPowerModel,gurobi)
result_switches_AC_DC_qc_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(ac_dc_bs_qc_ref,QCRMPowerModel,gurobi)
result_switches_AC_DC_lpac_ref  = _PMTP.run_acdcsw_AC_DC_reformulation(ac_dc_bs_lpac_ref,LPACCPowerModel,gurobi)

##############
# AC feasibility checks for AC busbar splitting
feasibility_check_AC_DC_BS_opf_soc  = deepcopy(data_busbars_ac_dc_split_5_acdc)
feasibility_check_AC_DC_BS_opf_qc   = deepcopy(data_busbars_ac_dc_split_5_acdc)
feasibility_check_AC_DC_BS_opf_lpac = deepcopy(data_busbars_ac_dc_split_5_acdc)

feasibility_check_AC_DC_BS_opf_soc_ref  = deepcopy(data_busbars_ac_dc_split_5_acdc)
feasibility_check_AC_DC_BS_opf_qc_ref   = deepcopy(data_busbars_ac_dc_split_5_acdc)
feasibility_check_AC_DC_BS_opf_lpac_ref = deepcopy(data_busbars_ac_dc_split_5_acdc)

for (br_id, br) in result_switches_AC_DC_ac["solution"]["switch"]
    feasibility_check_AC_DC_BS_opf_soc["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_soc["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_qc["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_qc["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_lpac["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac["solution"]["switch"][br_id]["status"])
end
for (br_id, br) in result_switches_AC_DC_ac["solution"]["switch"]
    feasibility_check_AC_DC_BS_opf_soc["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_soc["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_qc["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_qc["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_lpac["switch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac["solution"]["switch"][br_id]["status"])
end

for (br_id, br) in result_switches_AC_DC_ac["solution"]["dcswitch"]
    feasibility_check_AC_DC_BS_opf_soc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_soc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_qc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_qc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_lpac_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac_ref["solution"]["dcswitch"][br_id]["status"])
end
for (br_id, br) in result_switches_AC_DC_ac_ref["solution"]["dcswitch"]
    feasibility_check_AC_DC_BS_opf_soc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_soc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_qc_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_qc_ref["solution"]["dcswitch"][br_id]["status"])
    feasibility_check_AC_DC_BS_opf_lpac_ref["dcswitch"][br_id]["status"] = deepcopy(result_switches_AC_DC_lpac_ref["solution"]["dcswitch"][br_id]["status"])
end

feasibility_check_opf_soc = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_soc,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_qc = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_qc,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_lpac,ACPPowerModel,ipopt; setting = s)

feasibility_check_opf_soc_ref = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_soc_ref,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_qc_ref = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_qc_ref,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac_ref = _PMTP.run_acdcsw_AC_DC_opf(feasibility_check_AC_DC_BS_opf_lpac_ref,ACPPowerModel,ipopt; setting = s)


for (sw_id,sw) in result_switches_AC_DC_soc["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end
for (sw_id,sw) in result_switches_AC_DC_qc["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end
for (sw_id,sw) in result_switches_AC_DC_lpac["solution"]["switch"]
    print([sw_id,sw["status"]],"\n")
end

for (sw_id,sw) in result_switches_AC_DC_soc_ref["solution"]["dcswitch"]
    print([sw_id,sw["status"]],"\n")
end
for (sw_id,sw) in result_switches_AC_DC_qc_ref["solution"]["dcswitch"]
    print([sw_id,sw["status"]],"\n")
end
for (sw_id,sw) in result_switches_AC_DC_lpac_ref["solution"]["dcswitch"]
    print([sw_id,sw["status"]],"\n")
end












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
