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

test_case = "case5.m"
test_case_sw = "case5_sw.m"
test_case_5_acdc = "case5_acdc.m"
#test_case_5_acdc = "case39_acdc.m"
Belgium = "BE_grid_with_energy_island.json"

#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_original_5_acdc = _PM.parse_file(data_file_5_acdc)
data_5_acdc = deepcopy(data_original_5_acdc)
_PMACDC.process_additional_data!(data_5_acdc)

#=
# Configuration 1, nice results
for (l_id,l) in data_5_acdc["load"]
    if l_id != "4"
        l["pd"] = 0
        l["qd"] = 0
    else
        l["pd"] = l["pd"]*3
        l["qd"] = l["qd"]*3
    end
end
#for (l_id,l) in data_5_acdc["branch"]
#    l["rate_a"] = 0.8
#end
=#

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s_dual)

# Solving AC OTS with OTS only on the AC grid part
result_AC_ots_5    = _PMTP.run_acdcots_AC(data_5_acdc,ACPPowerModel,juniper; setting = s)

# Solving AC OTS with OTS only on the DC grid part 
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)

# Solving AC OTS with OTS on both AC and DC grid part
result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)

for (br_id,br) in result_AC_ots_BE["solution"]["branch"]
    #if br["br_status"] <= 0.5
        print([br_id,br["br_status"]],"\n")
    #end
end
for (br_id,br) in result_AC_ots_5_NLP["solution"]["branch"]
    #if br["br_status"] <= 0.5
    print(br["br_status"],"\n")
    #end
end
for (br_id,br) in result_AC_DC_ots_5["solution"]["branch"]
    #if br["br_status"] <= 0.5
        print(br["br_status"],"\n")
    #end
end


#######################################################################################
## Busbar splitting models ##
#######################################################################################
# AC BS for AC/DC grid with AC switches state as decision variable
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
#splitted_bus_ac = [1,2,3,4,5]
#splitted_bus_ac = [4,5]
splitted_bus_ac = [2,4]
#splitted_bus_ac = 4

#splitted_bus_ac = 1
data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,ACPPowerModel,juniper)


# AC OTS for AC/DC grid with DC switches state as decision variable
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)
splitted_bus_dc = [1,2,3]
data_busbars_dc_split_5_acdc , switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc,splitted_bus_dc)
result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, ACPPowerModel,juniper)


# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
splitted_bus_ac = [2,4]
splitted_bus_dc = [1,2,3]

splitted_bus_dc = 1 # With this configuration it works!
#splitted_bus_dc = 2

data_busbars_ac_dc_split_5_acdc_ac_sw,  ac_switches_couples_ac_dc_5, ac_extremes_ZILs_5_ac_dc  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc_ac_dc_sw , dc_switches_couples_ac_dc_5, dc_extremes_ZILs_5_ac_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_ac_sw,splitted_bus_dc)

result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, ACPPowerModel,juniper)


ac_branches_power_flow = Dict{String,Any}()
for (br_id,br) in data_5_acdc["branch"]
    ac_branches_power_flow[br_id] = Dict{String,Any}()
    ac_branches_power_flow[br_id]["OPF"] = result_opf_5_ac["solution"]["branch"][br_id]["pt"]
    ac_branches_power_flow[br_id]["OTS"] = result_AC_DC_ots_5["solution"]["branch"][br_id]["pt"]
    ac_branches_power_flow[br_id]["BS"] = result_AC_DC_5_switch_AC_DC["solution"]["branch"][br_id]["pt"]
end

dc_branches_power_flow = Dict{String,Any}()
for (br_id,br) in data_5_acdc["branchdc"]
    dc_branches_power_flow[br_id] = Dict{String,Any}()
    dc_branches_power_flow[br_id]["OPF"] = result_opf_5_ac["solution"]["branchdc"][br_id]["pt"]
    dc_branches_power_flow[br_id]["OTS"] = result_AC_DC_ots_5["solution"]["branchdc"][br_id]["pt"]
    dc_branches_power_flow[br_id]["BS"] = result_AC_DC_5_switch_AC_DC["solution"]["branchdc"][br_id]["pt"]
end

pg_gen = Dict{String,Any}()
for (br_id,br) in data_5_acdc["gen"]
    pg_gen[br_id] = Dict{String,Any}()
    pg_gen[br_id]["OPF"] = result_opf_5_ac["solution"]["gen"][br_id]["pg"]
    pg_gen[br_id]["OTS"] = result_AC_DC_ots_5["solution"]["gen"][br_id]["pg"]
    pg_gen[br_id]["BS"] = result_AC_DC_5_switch_AC_DC["solution"]["gen"][br_id]["pg"]
end

qg_gen = Dict{String,Any}()
for (br_id,br) in data_5_acdc["gen"]
    qg_gen[br_id] = Dict{String,Any}()
    qg_gen[br_id]["OPF"] = result_opf_5_ac["solution"]["gen"][br_id]["qg"]
    qg_gen[br_id]["OTS"] = result_AC_DC_ots_5["solution"]["gen"][br_id]["qg"]
    qg_gen[br_id]["BS"] = result_AC_DC_5_switch_AC_DC["solution"]["gen"][br_id]["qg"]
end

branches_ots = Dict{String,Any}()
for (br_id,br) in data_5_acdc["branch"]
    branches_ots[br_id] = Dict{String,Any}()
    branches_ots[br_id]["f_bus"] = br["f_bus"]
    branches_ots[br_id]["t_bus"] = br["t_bus"]
    branches_ots[br_id]["status"] = result_AC_DC_ots_5["solution"]["branch"][br_id]["br_status"]
end

switches = Dict{String,Any}()
for (br_id,br) in data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]
    switches[br_id] = Dict{String,Any}()
    switches[br_id]["f_bus"] = br["f_bus"]
    switches[br_id]["t_bus"] = br["t_bus"]
    if haskey(br,"original")
        switches[br_id]["original"] = br["original"]
    else
        switches[br_id]["original"] = 0
    end
    if haskey(br,"auxiliary")
        switches[br_id]["auxiliary"] = br["auxiliary"]
    else
        switches[br_id]["auxiliary"] = 0
    end
    if haskey(data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch_couples"],br_id)
        switches[br_id]["switch_couple"] = data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch_couples"][br_id]
    else
        switches[br_id]["switch_couple"] = false
    end
    switches[br_id]["status"] = result_AC_DC_5_switch_AC_DC["solution"]["switch"][br_id]["status"]
end
switches["3"]
switches["4"]


data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["3"]
data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["4"]
data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["5"]
data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["6"]