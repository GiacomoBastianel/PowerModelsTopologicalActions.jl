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
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)

#######################################################################################
## Input data ##
#######################################################################################

test_case_5_acdc = "case5_acdc.m"
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


#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_5_ac      = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s_dual)
result_opf_5_lpac    = _PMACDC.run_acdcopf(data_5_acdc,LPACCPowerModel,ipopt; setting = s_dual)

# Solving AC OTS with OTS only on the AC grid part
result_AC_ots_5_ac      = _PMTP.run_acdcots_AC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_AC_ots_5_lpac    = _PMTP.run_acdcots_AC(data_5_acdc,LPACCPowerModel,juniper; setting = s)

# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1
result_AC_ots_5_lin_ac   = _PMTP.run_acdcots_AC_lin(data_5_acdc,ACPPowerModel,ipopt; setting = s)
result_AC_ots_5_lin_lpac = _PMTP.run_acdcots_AC_lin(data_5_acdc,LPACCPowerModel,ipopt; setting = s)

# assigning the branch status of the grid to 0 or 1 based on the previous simulation
data_5_acdc_linearised_ac = deepcopy(data_5_acdc)
for (b, branch) in result_AC_ots_5_lin_ac["solution"]["branch"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised_ac["branch"][b]["br_status"] = 0
    else
        data_5_acdc_linearised_ac["branch"][b]["br_status"] = 1
    end
end
data_5_acdc_linearised_lpac = deepcopy(data_5_acdc)
for (b, branch) in result_AC_ots_5_lin_lpac["solution"]["branch"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised_lpac["branch"][b]["br_status"] = 0
    else
        data_5_acdc_linearised_lpac["branch"][b]["br_status"] = 1
    end
end

# Running the OPF to check for feasibility/(local) optimality
result_AC_ots_5_ac_lin    = _PMACDC.run_acdcopf(data_5_acdc_linearised_ac,ACPPowerModel,ipopt; setting = s)
result_AC_ots_5_lpac_lin  = _PMACDC.run_acdcopf(data_5_acdc_linearised_lpac,ACPPowerModel,ipopt; setting = s)

# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1
result_AC_ots_5_boom_boom_ac_lin   = _PMTP.run_acdcots_AC_lin_constrained(data_boom_boom_ac,ACPPowerModel,ipopt; setting = s)
result_AC_ots_5_boom_boom_lpac_lin = _PMTP.run_acdcots_AC_lin_constrained(data_boom_boom_lpac,LPACCPowerModel,ipopt; setting = s)


# Solving a two steps OTS:
# 1) AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1
# 2) Using results from the previous simulation as starting point for the OTS with linearised variables and constraint z(z-1) < err added 
result_AC_ots_5_lin_ac   = _PMTP.run_acdcots_AC_lin(data_boom_boom_ac  ,ACPPowerModel,ipopt; setting = s)
result_AC_ots_5_lin_lpac = _PMTP.run_acdcots_AC_lin(data_boom_boom_lpac,LPACCPowerModel,gurobi; setting = s)


# assigning the branch status of the grid to 0 or 1 based on the previous simulation
data_5_acdc_linearised_ac   = deepcopy(data_5_acdc)
data_5_acdc_linearised_lpac = deepcopy(data_5_acdc)

for (b, branch) in result_AC_ots_5_lin_ac["solution"]["branch"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised_ac["branch"][b]["br_status_initial"] = 0
    else
        data_5_acdc_linearised_ac["branch"][b]["br_status_initial"] = 1
    end
end
for (b, branch) in result_AC_ots_5_lin_lpac["solution"]["branch"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised_lpac["branch"][b]["br_status_initial"] = 0
    else
        data_5_acdc_linearised_lpac["branch"][b]["br_status_initial"] = 1
    end
end

# Running the OPF to check for feasibility/(local) optimality
result_AC_ots_5_lin_const_ac   = _PMTP.run_acdcots_AC_lin_constrained_sp(data_5_acdc_linearised_ac,ACPPowerModel,ipopt; setting = s)
result_AC_ots_5_lin_const_lpac = _PMTP.run_acdcots_AC_lin_constrained_sp(data_5_acdc_linearised_lpac,LPACCPowerModel,ipopt; setting = s)


data_boom_boom_ac   = deepcopy(data_5_acdc)
data_boom_boom_lpac = deepcopy(data_5_acdc)


# assigning the branch status of the grid to 0 or 1 based on the previous simulation
data_5_acdc_linearised = deepcopy(data_5_acdc)
for (b, branch) in data_5_acdc_linearised["branch"]
    branch["br_status_initial"] = 0.0
end
# Running the OPF to check for feasibility/(local) optimality
result_AC_ots_5_lin_const = _PMTP.run_acdcots_AC_lin_constrained_sp(data_5_acdc_linearised,ACPPowerModel,ipopt; setting = s)



data_boom_boom_ac   = deepcopy(data_5_acdc)
data_boom_boom_lpac = deepcopy(data_5_acdc)

# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1 and constraint z(z-1) < err added 
l = collect(0.0:0.1:1.0) #collecting list of starting point

results_starting_point = Dict{String,Any}()
@time for i in l
    data_boom_boom_ac   = deepcopy(data_5_acdc)
    for (b, branch) in data_boom_boom_ac["branch"]
        branch["br_status_initial"] = deepcopy(i)
    end
    result_AC_ots_starting_point_i = _PMTP.run_acdcots_AC_lin_constrained_sp(data_boom_boom_ac,ACPPowerModel,ipopt; setting = s)
    
    # The dictionay here can be refined as one prefers
    results_starting_point["$i"] = Dict{String,Any}()
    results_starting_point["$i"]["termination_status"] = result_AC_ots_starting_point_i["termination_status"]
    results_starting_point["$i"]["primal_status"] = result_AC_ots_starting_point_i["primal_status"]
    results_starting_point["$i"]["objective"] = result_AC_ots_starting_point_i["objective"]
end

results_starting_point = Dict{String,Any}()
@time for i in l
    data_boom_boom_lpac   = deepcopy(data_5_acdc)
    for (b, branch) in data_boom_boom_lpac["branch"]
        branch["br_status_initial"] = deepcopy(i)
    end
    result_AC_ots_starting_point_i = _PMTP.run_acdcots_AC_lin_constrained_sp(data_boom_boom_lpac,LPACCPowerModel,ipopt; setting = s)
    
    # The dictionay here can be refined as one prefers
    results_starting_point["$i"] = Dict{String,Any}()
    results_starting_point["$i"]["termination_status"] = result_AC_ots_starting_point_i["termination_status"]
    results_starting_point["$i"]["primal_status"] = result_AC_ots_starting_point_i["primal_status"]
    results_starting_point["$i"]["objective"] = result_AC_ots_starting_point_i["objective"]
end

for i in l
    print([i,results_starting_point["$i"]["termination_status"],results_starting_point["$i"]["objective"]],"\n")
end


##########################################
##############################
# Solving AC OTS with OTS only on the DC grid part
result_AC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)

# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1
result_AC_ots_5_lin = _PMTP.run_acdcots_DC_lin(data_5_acdc,ACPPowerModel,ipopt; setting = s)

# assigning the branch status of the grid to 0 or 1 based on the previous simulation
data_5_acdc_linearised = deepcopy(data_5_acdc)
for (b, branch) in result_AC_ots_5_lin["solution"]["branchdc"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised["branchdc"][b]["br_status"] = 0
    else
        data_5_acdc_linearised["branchdc"][b]["br_status"] = 1
    end
end
for (c, conv) in result_AC_ots_5_lin["solution"]["convdc"]
    if conv["conv_status"] < 0.5
        data_5_acdc_linearised["convdc"][c]["conv_status"] = 0
    else
        data_5_acdc_linearised["convdc"][c]["conv_status"] = 1
    end
end

# Running the OPF to check for feasibility/(local) optimality
result_AC_ots_5    = _PMACDC.run_acdcopf(data_5_acdc_linearised,ACPPowerModel,ipopt; setting = s)

# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1
result_AC_ots_5_lin = _PMTP.run_acdcots_DC_lin_constrained(data_5_acdc,ACPPowerModel,ipopt; setting = s)


# Solving a two steps OTS:
# 1) AC OTS with OTS only on the DC grid part, binary constraint for each branch linearised between 0 and 1
# 2) Using results from the previous simulation as starting point for the OTS with linearised variables and constraint z(z-1) < err added 
result_AC_ots_5_lin = _PMTP.run_acdcots_DC_lin(data_5_acdc,ACPPowerModel,ipopt; setting = s)

# assigning the branch status of the grid to 0 or 1 based on the previous simulation
data_5_acdc_linearised = deepcopy(data_5_acdc)
for (b, branch) in result_AC_ots_5_lin["solution"]["branchdc"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised["branchdc"][b]["br_status_initial"] = 0
    else
        data_5_acdc_linearised["branchdc"][b]["br_status_initial"] = 1
    end
end
for (c, conv) in result_AC_ots_5_lin["solution"]["convdc"]
    if conv["conv_status"] < 0.5
        data_5_acdc_linearised["convdc"][c]["conv_status_initial"] = 0
    else
        data_5_acdc_linearised["convdc"][c]["conv_status_initial"] = 1
    end
end
# Running the OPF to check for feasibility/(local) optimality
result_AC_ots_5_lin_const = _PMTP.run_acdcots_DC_lin_constrained_sp(data_5_acdc_linearised,ACPPowerModel,ipopt; setting = s)


# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1 and constraint z(z-1) < err added 
l = collect(0.0:0.1:1.0) #collecting list of starting point

results_starting_point = Dict{String,Any}()
for i in l
    for (b, branch) in data_5_acdc["branchdc"]
        branch["br_status_initial"] = deepcopy(i)
    end
    for (c, conv) in data_5_acdc["convdc"]
        conv["conv_status_initial"] = deepcopy(i)
    end
    result_AC_ots_starting_point_i = _PMTP.run_acdcots_DC_lin_constrained_sp(data_5_acdc,ACPPowerModel,ipopt; setting = s)
    
    # The dictionay here can be refined as one prefers
    results_starting_point["$i"] = Dict{String,Any}()
    results_starting_point["$i"]["termination_status"] = result_AC_ots_starting_point_i["termination_status"]
    results_starting_point["$i"]["primal_status"] = result_AC_ots_starting_point_i["primal_status"]
    results_starting_point["$i"]["objective"] = result_AC_ots_starting_point_i["objective"]
end

for i in l
    print([i,results_starting_point["$i"]["termination_status"],results_starting_point["$i"]["objective"]],"\n")
end



##########################################
##############################
# Solving AC OTS with both the AC and DC grid part
result_AC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)

# Solving AC OTS with both the AC and DC grid part, binary constraint for each branch linearised between 0 and 1
result_AC_ots_5_lin = _PMTP.run_acdcots_AC_DC_lin(data_5_acdc,ACPPowerModel,ipopt; setting = s)

# assigning the branch status of the grid to 0 or 1 based on the previous simulation
data_5_acdc_linearised = deepcopy(data_5_acdc)
for (b, branch) in result_AC_ots_5_lin["solution"]["branch"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised["branch"][b]["br_status"] = 0
    else
        data_5_acdc_linearised["branch"][b]["br_status"] = 1
    end
end
for (b, branch) in result_AC_ots_5_lin["solution"]["branchdc"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised["branchdc"][b]["br_status"] = 0
    else
        data_5_acdc_linearised["branchdc"][b]["br_status"] = 1
    end
end
for (c, conv) in result_AC_ots_5_lin["solution"]["convdc"]
    if conv["conv_status"] < 0.5
        data_5_acdc_linearised["convdc"][c]["conv_status"] = 0
    else
        data_5_acdc_linearised["convdc"][c]["conv_status"] = 1
    end
end

# Running the OPF to check for feasibility/(local) optimality
result_AC_ots_5    = _PMACDC.run_acdcopf(data_5_acdc_linearised,ACPPowerModel,ipopt; setting = s)

# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1
result_AC_ots_5_lin = _PMTP.run_acdcots_AC_DC_lin_constrained(data_5_acdc,ACPPowerModel,ipopt; setting = s)


# Solving a two steps OTS:
# 1) AC OTS with OTS only on the DC grid part, binary constraint for each branch linearised between 0 and 1
# 2) Using results from the previous simulation as starting point for the OTS with linearised variables and constraint z(z-1) < err added 
result_AC_ots_5_lin = _PMTP.run_acdcots_AC_DC_lin(data_5_acdc,ACPPowerModel,ipopt; setting = s)

# assigning the branch status of the grid to 0 or 1 based on the previous simulation
data_5_acdc_linearised = deepcopy(data_5_acdc)
for (b, branch) in result_AC_ots_5_lin["solution"]["branch"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised["branch"][b]["br_status_initial"] = 0
    else
        data_5_acdc_linearised["branch"][b]["br_status_initial"] = 1
    end
end
for (b, branch) in result_AC_ots_5_lin["solution"]["branchdc"]
    if branch["br_status"] < 0.5
        data_5_acdc_linearised["branchdc"][b]["br_status_initial"] = 0
    else
        data_5_acdc_linearised["branchdc"][b]["br_status_initial"] = 1
    end
end
for (c, conv) in result_AC_ots_5_lin["solution"]["convdc"]
    if conv["conv_status"] < 0.5
        data_5_acdc_linearised["convdc"][c]["conv_status_initial"] = 0
    else
        data_5_acdc_linearised["convdc"][c]["conv_status_initial"] = 1
    end
end
# Running the OPF to check for feasibility/(local) optimality
result_AC_ots_5_lin_const = _PMTP.run_acdcots_AC_DC_lin_constrained_sp(data_5_acdc_linearised,ACPPowerModel,ipopt; setting = s)


# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1 and constraint z(z-1) < err added 
l = collect(0.0:0.1:1.0) #collecting list of starting point

results_starting_point = Dict{String,Any}()
for i in l
    for (b, branch) in data_5_acdc["branch"]
        branch["br_status_initial"] = deepcopy(i)
    end
    for (b, branch) in data_5_acdc["branchdc"]
        branch["br_status_initial"] = deepcopy(i)
    end
    for (c, conv) in data_5_acdc["convdc"]
        conv["conv_status_initial"] = deepcopy(i)
    end
    result_AC_ots_starting_point_i = _PMTP.run_acdcots_AC_DC_lin_constrained_sp(data_5_acdc,ACPPowerModel,ipopt; setting = s)
    
    # The dictionay here can be refined as one prefers
    results_starting_point["$i"] = Dict{String,Any}()
    results_starting_point["$i"]["termination_status"] = result_AC_ots_starting_point_i["termination_status"]
    results_starting_point["$i"]["primal_status"] = result_AC_ots_starting_point_i["primal_status"]
    results_starting_point["$i"]["objective"] = result_AC_ots_starting_point_i["objective"]
end

for i in l
    print([i,results_starting_point["$i"]["termination_status"],results_starting_point["$i"]["objective"]],"\n")
end
