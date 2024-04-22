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

test_case_5_acdc = "dcoverlay_grid.m"
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

for (g_id,g) in data_5_acdc["gen"]
    if length(g["cost"]) != 2
        push!(g["cost"],0.0)
        push!(g["cost"],0.0)
    end
end

for (g_id,g) in data_5_acdc["gen"]
    print(g_id,g["cost"],"\n")
end

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s_dual)
#result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,SOCWRPowerModel,ipopt; setting = s_dual)
result_opf_5_lpac  = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,ipopt; setting = s_dual)

# Solving AC OTS with OTS only on the AC grid part
result_AC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)

result_AC_ots_5_lpac    = _PMTP.run_acdcots_AC_DC(data_5_acdc,LPACCPowerModel,juniper; setting = s)

for (br_id,br) in result_AC_ots_5["solution"]["branchdc"]
    print([br_id,br["br_status"]],"\n")
end
#  11 , 6
for (br_id,br) in result_AC_ots_5["solution"]["convdc"]
    print([br_id,br["conv_status"]],"\n")
end
#  27 , 16 , 9

# Solving AC OTS with OTS only on the DC grid part 
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)
#result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
#result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,QCRMPowerModel,juniper; setting = s)
result_DC_ots_5_lpac    = _PMTP.run_acdcots_DC(data_5_acdc,LPACCPowerModel,juniper; setting = s)

