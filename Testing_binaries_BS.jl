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

#test_case_5_acdc = "case5_acdc.m"
test_case_5_acdc = "case39_acdc.m"
#test_case_5_acdc = "case67.m"
#test_case_5_acdc = "case3120sp_mcdc.m"

test_case_ac_only = "case24.m"

#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_original_5_acdc = _PM.parse_file(data_file_5_acdc)

data_5_acdc = deepcopy(data_original_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

# For case 39
for (l_id,l) in data_5_acdc["load"]
    l["pd"] = l["pd"]/2 
    l["qd"] = l["qd"]/2 
end

data_boom_boom = deepcopy(data_5_acdc)

#######################################################################################
## Busbar splitting models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s_dual)

for (b_id,b) in result_opf_5_ac["solution"]["bus"]
    print([b_id,"lam_kcl_i $(b["lam_kcl_i"])", "lam_kcl_r $(b["lam_kcl_r"])"],"\n")
end


# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_lin = deepcopy(data_5_acdc)

# Selecting which busbars are split
#splitted_bus_ac = [2,5] # -> 0.9 the best (better than MINLP)
splitted_bus_ac = [35,38] # -> 0.8 the best (better than MINLP)
#splitted_bus_ac = [4,5] # -> 1.0 the best (better than MINLP)

#data_busbars_ac_split_5_acdc, extremes_ZIL = _PMTP.AC_busbar_split_more_buses_fixed(data_busbars_ac_split_5_acdc,splitted_bus_ac)
#split_elements = _PMTP.elements_AC_busbar_split(data_5_acdc)

data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
data_busbars_ac_split_5_acdc_no_OTS,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc_no_OTS,splitted_bus_ac)
data_busbars_ac_split_5_acdc_lin,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc_lin,splitted_bus_ac)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,SOCWRPowerModel,gurobi)
data_busbars_ac_split_5_acdc_ = deepcopy(data_busbars_ac_split_5_acdc)
for i in 1:10
    data_busbars_ac_split_5_acdc_["switch"]["$i"]["status"] = deepcopy(result_AC_DC_5_switches_AC["solution"]["switch"]["$i"]["status"])
end
result_opf_5_soc    = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s_dual)

#result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,SOCWRPowerModel,juniper)
#result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,QCRMPowerModel,ipopt)

# Not necessary to reconnect all the branches
result_AC_DC_5_switches_AC_no_OTS  = _PMTP.run_acdcsw_AC_no_OTS(data_busbars_ac_split_5_acdc_no_OTS,ACPPowerModel,juniper)


data_busbars_ac_split_5_acdc_linearised = deepcopy(data_busbars_ac_split_5_acdc)
result_AC_DC_5_switches_AC_lin  = _PMTP.run_acdcsw_AC_lin(data_busbars_ac_split_5_acdc_linearised,SOCWRPowerModel,ipopt)


for i in eachindex(switches_couples_ac_5)
    if result_AC_DC_5_switches_AC_lin["solution"]["switch"]["$(switches_couples_ac_5["$i"]["f_sw"])"]["status"] < 0.5
        data_busbars_ac_split_5_acdc_linearised["switch"]["$(switches_couples_ac_5["$i"]["f_sw"])"]["status"] = 0
        data_busbars_ac_split_5_acdc_linearised["switch"]["$(switches_couples_ac_5["$i"]["t_sw"])"]["status"] = 1
    else
        data_busbars_ac_split_5_acdc_linearised["switch"]["$(switches_couples_ac_5["$i"]["f_sw"])"]["status"] = 1
        data_busbars_ac_split_5_acdc_linearised["switch"]["$(switches_couples_ac_5["$i"]["t_sw"])"]["status"] = 0
    end
end

# Running the OPF to check for feasibility/(local) optimality
result_AC_DC_5_switches_AC_opf = _PMACDC.run_acdcopf(data_busbars_ac_split_5_acdc_linearised,ACPPowerModel,ipopt; setting = s)






data_busbars_ac_split_5_acdc_linearised = deepcopy(data_busbars_ac_split_5_acdc)

# Solving a two steps OTS:
# 1) AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1
# 2) Using results from the previous simulation as starting point for the OTS with linearised variables and constraint z(z-1) < err added 
#result_AC_DC_5_switches_AC_lin = _PMTP.run_acdcsw_AC_lin_constrained(data_busbars_ac_split_5_acdc_linearised,SOCWRPowerModel,ipopt; setting = s)

# assigning the branch status of the grid to 0 or 1 based on the previous simulation
for (sw_id,sw) in result_AC_DC_5_switches_AC_lin["solution"]["switch"]
    if sw["status"] < 0.5
        data_busbars_ac_split_5_acdc_linearised["switch"][sw_id]["switch_status_initial"] = 0
    else
        data_busbars_ac_split_5_acdc_linearised["switch"][sw_id]["switch_status_initial"] = 1
    end
end
result_AC_DC_5_switches_AC_lin = _PMTP.run_acdcsw_AC_lin_constrained_sp(data_busbars_ac_split_5_acdc_linearised,SOCWRPowerModel,ipopt; setting = s)


data_busbars_ac_split_5_acdc_linearised = deepcopy(data_busbars_ac_split_5_acdc_lin)
# Running the OPF to check for feasibility/(local) optimality
result_AC_ots_5_lin_const = _PMTP.run_acdcsw_AC_lin_constrained(data_busbars_ac_split_5_acdc_linearised,ACPPowerModel,ipopt; setting = s)

# Solving AC OTS with OTS only on the AC grid part, binary constraint for each branch linearised between 0 and 1 and constraint z(z-1) < err added 
l = collect(0.0:0.1:1.0) #collecting list of starting point
data_busbars_ac_split_5_acdc_linearised = deepcopy(data_busbars_ac_split_5_acdc)

results_starting_point = Dict{String,Any}()
@time for i in l
    for (b, branch) in data_busbars_ac_split_5_acdc_linearised["switch"]
        if branch["ZIL"] == false
            branch["switch_status_initial"] = deepcopy(i)
        else
            branch["switch_status_initial"] = deepcopy(0.0)
        end
    end
    result_AC_ots_starting_point_i = _PMTP.run_acdcsw_AC_lin_constrained_sp(data_busbars_ac_split_5_acdc_linearised,SOCWRPowerModel,ipopt; setting = s)
    
    # The dictionay here can be refined as one prefers
    results_starting_point["$i"] = Dict{String,Any}()
    results_starting_point["$i"]["termination_status"] = result_AC_ots_starting_point_i["termination_status"]
    results_starting_point["$i"]["primal_status"] = result_AC_ots_starting_point_i["primal_status"]
    results_starting_point["$i"]["objective"] = result_AC_ots_starting_point_i["objective"]
    results_starting_point["$i"]["solution"] = result_AC_ots_starting_point_i["solution"]
end

for i in l
    print([i,results_starting_point["$i"]["termination_status"],results_starting_point["$i"]["objective"]],"\n")
end

results_sw_opf = Dict{String,Any}()
@time for i in l
        data_busbars_ac_split_5_acdc_fixed = deepcopy(data_busbars_ac_split_5_acdc_linearised)
        # assigning the branch status of the grid to 0 or 1 based on the previous simulation
        for (sw_id,sw) in results_starting_point["$i"]["solution"]["switch"]
                if sw["status"] < 0.5
                    data_busbars_ac_split_5_acdc_fixed["switch"][sw_id]["status"] = 0
                else
                    data_busbars_ac_split_5_acdc_fixed["switch"][sw_id]["status"] = 1
                end
        end
        # Running the OPF to check for feasibility/(local) optimality
        result_AC_DC_5_switches_AC_opf_i = _PMTP.run_acdcsw_opf_AC(data_busbars_ac_split_5_acdc_fixed,ACPPowerModel,ipopt; setting = s)
        # The dictionay here can be refined as one prefers
        results_sw_opf["$i"] = Dict{String,Any}()
        results_sw_opf["$i"]["termination_status"] = result_AC_DC_5_switches_AC_opf_i["termination_status"]
        results_sw_opf["$i"]["primal_status"] = result_AC_DC_5_switches_AC_opf_i["primal_status"]
        results_sw_opf["$i"]["objective"] = result_AC_DC_5_switches_AC_opf_i["objective"]
        results_sw_opf["$i"]["solution"] = result_AC_DC_5_switches_AC_opf_i["solution"]
        results_sw_opf["$i"]["data"] = deepcopy(data_busbars_ac_split_5_acdc_fixed)
end
for i in l
    print([i,results_sw_opf["$i"]["termination_status"],results_sw_opf["$i"]["objective"]],"\n")
end


for (sw_id,sw) in data_busbars_ac_split_5_acdc["switch"]
    print([sw_id,result_AC_DC_5_switches_AC["solution"]["switch"][sw_id]["status"]],"\n")
end

for (sw_id,sw) in results_sw_opf["0.9"]["data"]["switch"]
    if haskey(results_sw_opf["0.9"]["solution"]["switch"],sw_id)
        print([sw_id,"1.0"],"\n")
    else
        print([sw_id,"0.0"],"\n")
    end
end
results_sw_opf["0.9"]["objective"]
result_AC_DC_5_switches_AC["objective"]







#=
# Define an OPF with switches?


for (sw_id,sw) in data_busbars_ac_split_5_acdc["switch"]
    print(result_AC_DC_5_switches_AC["solution"]["switch"][sw_id]["status"],"\n")
end

for (sw_id,sw) in data_busbars_ac_split_5_acdc_fixed["switch"]
    print(sw["status"],"\n")
end



#=
# If one wants to check the status of the switches. To be improved to make it easier and faster for the user to see the resulting grid topology
switches_results = []
for i in 1:length(result_AC_DC_5_switches_AC["solution"]["switch"])
    push!(switches_results,result_AC_DC_5_switches_AC["solution"]["switch"]["$i"]["status"])
end
=#

# AC OTS for AC/DC grid with DC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_dc = [1,2,3]
data_busbars_dc_split_5_acdc , switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc,splitted_bus_dc)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, ACPPowerModel,juniper)

result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, SOCWRPowerModel,gurobi)
result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, QCRMPowerModel,gurobi)


# Not necessary to reconnect all the branches
#result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_dc_split_5_acdc, ACPPowerModel,juniper)

#=
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

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, ACPPowerModel,juniper)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, SOCWRPowerModel,gurobi)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, QCRMPowerModel,gurobi)

# Not necessary to reconnect all the branches
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc, ACPPowerModel,juniper)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc, SOCWRPowerModel,gurobi)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc, QCRMPowerModel,gurobi)

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
=#