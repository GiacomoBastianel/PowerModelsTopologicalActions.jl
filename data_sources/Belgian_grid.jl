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

test_case_Belgium = "BE_grid_with_energy_island.json"

#######################################################################################
## Parsing input data ##
#######################################################################################

data_file = "./data_sources/$test_case_Belgium"
data_Belgium = _PM.parse_file(data_file)

# This creates errors apparently
delete!(data_Belgium,"interconnectors")

s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)
result_opf = _PMACDC.run_acdcopf(data_Belgium,DCPPowerModel,gurobi;setting = s)

gen_setpoint = Dict{String,Any}()
for (g_id,g) in result_opf["solution"]["gen"]
    if g["pg"] != 0.0
        gen_setpoint[g_id] = deepcopy(g["pg"])
    end
end

count_ = 0
for (b_id,b) in data_Belgium["bus"]
    if b["bus_name"] == "Lat_long_nd"
        count_ += 1
    end
end