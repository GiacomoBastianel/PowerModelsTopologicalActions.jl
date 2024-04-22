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

result_dc = _PM.solve_opf(data_original_ac,DCPPowerModel,gurobi)
result_lpac = _PM.solve_opf(data_original_ac,LPACCPowerModel,gurobi)
result_ac = _PM.solve_opf(data_original_ac,ACPPowerModel,ipopt)

result_acdc_dc = _PMACDC.run_acdcopf(data_5_acdc,DCPPowerModel,gurobi; setting = s)
result_acdc_lpac = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,gurobi; setting = s)
result_acdc_ac = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s)

result_ots_dc = _PM.solve_ots(data_original_ac,DCPPowerModel,gurobi)
result_ots_lpac = _PM.solve_ots(data_original_ac,LPACCPowerModel,gurobi)
result_ots_ac = _PM.solve_ots(data_original_ac,ACPPowerModel,juniper)

for (br_id,br) in result_ots_dc["solution"]["branch"]
    print(br["br_status"],"\n")
end
for (br_id,br) in result_ots_lpac["solution"]["branch"]
    print(br["br_status"],"\n")
end
for (br_id,br) in result_ots_ac["solution"]["branch"]
    print(br["br_status"],"\n")
end