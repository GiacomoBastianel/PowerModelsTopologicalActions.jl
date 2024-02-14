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
test_case_5_acdc = "case39_acdc.m"
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

data_file_ac = joinpath(@__DIR__,"data_sources",test_case_ac_only)
data_original_ac = _PM.parse_file(data_file_ac)

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_5_ac_ac    = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s_dual)
result_opf_5_ac_soc    = _PMACDC.run_acdcopf(data_5_acdc,SOCWRPowerModel,ipopt; setting = s_dual)
result_opf_5_ac_qc    = _PMACDC.run_acdcopf(data_5_acdc,QCRMPowerModel,ipopt; setting = s_dual)


# Solving AC OTS with OTS only on the AC grid part
result_AC_ots_5_ac    = _PMTP.run_acdcots_AC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_AC_ots_5_soc    = _PMTP.run_acdcots_AC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_AC_ots_5_qc    = _PMTP.run_acdcots_AC(data_5_acdc,QCRMPowerModel,gurobi; setting = s)

for (br_id,br) in data_5_acdc["branch"]
    print(result_AC_ots_5_ac["solution"]["branch"][br_id]["br_status"],"\n")
end
for (br_id,br) in data_5_acdc["branch"]
    print(result_AC_ots_5_soc["solution"]["branch"][br_id]["br_status"],"\n")
end
for (br_id,br) in data_5_acdc["branch"]
    print(result_AC_ots_5_qc["solution"]["branch"][br_id]["br_status"],"\n")
end

# Solving AC OTS with OTS only on the DC grid part 
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,SOCWRPowerModel,gurobi; setting = s)
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,QCRMPowerModel,juniper; setting = s)


# Solving AC OTS with OTS on both AC and DC grid part
result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)
result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,SOCWRPowerModel,juniper; setting = s)
result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,QCRMPowerModel,juniper; setting = s)

##############

# Showing the utilization of each branch, to be intended as absolute values
for (br_id, br) in result_opf_5_ac["solution"]["branch"]
    print("Utilization AC branch $(br_id) OPF $(br["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",br["pf"],"\n")
    print("Utilization AC branch $(br_id) AC OTS $(result_AC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ", result_AC_ots_5["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) DC OTS $(result_DC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",result_DC_ots_5["solution"]["branch"][br_id]["pf"],"\n")
    print("Utilization AC branch $(br_id) AC/DC OTS $(result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100) %","  ",result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"],"\n")
    print("\n")
end

for (br_id, br) in result_opf_5_ac["solution"]["branchdc"]
    print("Utilization DC branch $(br_id) OPF $(br["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",br["pf"],"\n")
    print("Utilization DC branch $(br_id) AC OTS $(result_AC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_AC_ots_5["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC branch $(br_id) DC OTS $(result_DC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_DC_ots_5["solution"]["branchdc"][br_id]["pf"],"\n")
    print("Utilization DC branch $(br_id) AC/DC OTS $(result_AC_DC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100) %","  ",result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"],"\n")
    print("\n")
end

#######################################################################################
## Busbar splitting models ##
#######################################################################################

# AC BS for AC/DC grid with AC switches state as decision variable. Creating deepcopies of the original dictionary as the grid topology is modified with busbar splitting
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Selecting which busbars are split
splitted_bus_ac = 2


#data_busbars_ac_split_5_acdc, extremes_ZIL = _PMTP.AC_busbar_split_more_buses_fixed(data_busbars_ac_split_5_acdc,splitted_bus_ac)
#split_elements = _PMTP.elements_AC_busbar_split(data_5_acdc)

data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)

# One can select whether the branches originally linked to the split busbar are reconnected to either part of the split busbar or not
# Reconnect all the branches
result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,ACPPowerModel,juniper)
for (br_id,br) in data_busbars_ac_split_5_acdc["branch"]
    print(result_AC_DC_5_switches_AC["solution"]["switch"][br_id]["status"],"\n")
end
result_AC_DC_5_switches_soc  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,SOCWRPowerModel,gurobi)
for (br_id,br) in data_busbars_ac_split_5_acdc["branch"]
    print(result_AC_DC_5_switches_soc["solution"]["switch"][br_id]["status"],"\n")
end

result_AC_DC_5_switches_qc  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,QCRMPowerModel,gurobi)

# Not necessary to reconnect all the branches
result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC_no_OTS(data_busbars_ac_split_5_acdc,ACPPowerModel,juniper)


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
for (br_id,br) in data_busbars_ac_split_5_acdc["branch"]
    print(result_AC_DC_5_switches_DC["solution"]["dcswitch"][br_id]["status"],"\n")
end


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
