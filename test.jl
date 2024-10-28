using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
using PowerModelsTopologicalActionsII ; const _PMTP = PowerModelsTopologicalActionsII  
using InfrastructureModels; const _IM = InfrastructureModels
using JSON
using Mosek, MosekTools

#######################################################################################
## Define solver ##
#######################################################################################
model = Model(Ipopt.Optimizer)

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)
mosek = JuMP.optimizer_with_attributes(Mosek.Optimizer)

#######################################################################################
## Input data ##
#######################################################################################
test_case = "case5_acdc.m"

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file = joinpath(@__DIR__,"data_sources",test_case)
data_original = _PM.parse_file(data_file)

data = deepcopy(data_original)
_PMACDC.process_additional_data!(data)

#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
formulation = ACPPowerModel
solver = juniper

result_opf_ac = _PMACDC.run_acdcopf(data,formulation,solver; setting = s)

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC/DC OTS simulations
result_ots_ac = _PMTP.run_acdcots_AC(data,formulation,solver; setting = s)

#######################################################################################
## Busbar splitting models ##
#######################################################################################
# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data)

# Selecting which busbars are split
#splitted_bus_dc = collect(1:3)

splitted_bus_ac = [2,4]

split_elements = _PMTP.elements_AC_busbar_split(data)
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

feasibility_check_AC_BS_opf_soc_ref_status  = deepcopy(data_busbars_ac_split_5_acdc)
feasibility_check_AC_BS_opf_qc_ref_status   = deepcopy(data_busbars_ac_split_5_acdc)
feasibility_check_AC_BS_opf_lpac_ref_status = deepcopy(data_busbars_ac_split_5_acdc)

for (br_id, br) in result_switches_AC_soc_ref["solution"]["switch"]
    feasibility_check_AC_BS_opf_soc_ref_status["switch"][br_id]["status"] = deepcopy(result_switches_AC_soc_ref["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_BS_opf_qc_ref_status["switch"][br_id]["status"] = deepcopy(result_switches_AC_qc_ref["solution"]["switch"][br_id]["status"])
    feasibility_check_AC_BS_opf_lpac_ref_status["switch"][br_id]["status"] = deepcopy(result_switches_AC_lpac_ref["solution"]["switch"][br_id]["status"])
end

feasibility_check_opf_soc_ref_status = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_soc_ref_status,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_qc_ref_status = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_qc_ref_status,ACPPowerModel,ipopt; setting = s)
feasibility_check_opf_lpac_ref_status = _PMTP.run_acdcsw_AC_opf(feasibility_check_AC_BS_opf_lpac_ref_status,ACPPowerModel,ipopt; setting = s)














