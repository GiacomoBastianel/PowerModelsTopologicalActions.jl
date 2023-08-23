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
test_case_acdc = "case39_acdc.m"
#test_case_acdc = "case24_3zones_acdc.m"

#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_acdc = joinpath(@__DIR__,"data_sources",test_case_acdc)
data_original_acdc = _PM.parse_file(data_file_acdc)
data_acdc = deepcopy(data_original_acdc)
_PMACDC.process_additional_data!(data_acdc)
_PMACDC.process_additional_data!(data_original_acdc)

for (l_id,l) in data_acdc["load"]
    if l_id != "4"
        l["pd"] = 0
        l["qd"] = 0
    else
        l["pd"] = l["pd"]*6
        l["qd"] = l["qd"]*6
    end
end

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_ac    = _PMACDC.run_acdcopf(data_acdc,ACPPowerModel,ipopt; setting = s_dual)

# Solving AC OTS with OTS only on the AC grid part
result_AC_ots    = _PMTP.run_acdcots_AC(data_acdc,ACPPowerModel,juniper; setting = s)

for (br_id,br) in result_AC_ots["solution"]["branch"]
    if br["br_status"] <= 0.5
        print([br_id,br["br_status"]],"\n")
    end
end


data_AC = deepcopy(data_original_acdc)
for (b, branch) in result_AC_ots["solution"]["branch"]
    if branch["br_status"] < 0.5
        data_AC["branch"][b]["br_status_initial"] = 0
    else
        data_AC["branch"][b]["br_status_initial"] = 1
    end
end