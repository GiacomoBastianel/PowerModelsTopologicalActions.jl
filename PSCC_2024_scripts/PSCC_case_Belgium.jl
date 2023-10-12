using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII; const _PMTP = PowerModelsTopologicalActionsII
using PowerModelsTopologicalActionsII   
using InfrastructureModels; const _IM = InfrastructureModels
using JSON

#######################################################################################
## Define solver ##
#######################################################################################

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-6, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

#######################################################################################
## Input data ##
#######################################################################################
Belgium = "BE_grid_with_energy_island.json"

#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_Belgium = joinpath(@__DIR__,"data_sources",Belgium)
data_original_Belgium = _PM.parse_file(data_file_Belgium)

# Correcting the HVDC links from Stevn to Gezelle (380kV)
data_original_Belgium["convdc"]["5"]["busac_i"] = 26

data_Belgium = deepcopy(data_original_Belgium)
delete!(data_Belgium,"interconnectors")
#_PMACDC.process_additional_data!(data_Belgium) -> the data is already added
for (l_id,l) in data_Belgium["load"]
    l["qd"] = l["pd"]/10
end
for (br_id,br) in data_Belgium["convdc"]
    br["Imax"] = 3.50
end  
for (l_id,l) in data_Belgium["gen"]
    l["qmax"] = l["qmax"]*2
end

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_BE = _PMACDC.run_acdcopf(data_Belgium,ACPPowerModel,ipopt; setting = s_dual)

zones = []
for (br_id,br) in data_Belgium["gen"]
    push!(zones,br["gen_bus"])
    #if br["name_substation"] == "BE_Elisabeth_Zone_offshore_windfarm"
    #    print(br_id,"\n")
    #end
    if br["gen_bus"] == 148
        print(br_id,"\n")
    end
end
data_Belgium["gen"]["308"]

println(unique(zones))