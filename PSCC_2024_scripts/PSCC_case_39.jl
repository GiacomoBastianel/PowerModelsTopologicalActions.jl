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

test_case_39_acdc = "case39_acdc.m"

#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_39_acdc = joinpath(@__DIR__,"data_sources",test_case_39_acdc)


data_original_39_acdc = _PM.parse_file(data_file_39_acdc)
data_39_acdc = deepcopy(data_original_39_acdc)
_PMACDC.process_additional_data!(data_39_acdc)
_PMACDC.process_additional_data!(data_original_39_acdc)


# Configuration 1, nice results
for (l_id,l) in data_39_acdc["load"]
    if l_id != "4"
        l["pd"] = 0
        l["qd"] = 0
    else
        l["pd"] = l["pd"]*6
        l["qd"] = l["qd"]*6
    end
end
#-> with this configuration, you have the best improvement with busbar 16

#=
# Configuration 2, 
for (l_id,l) in data_39_acdc["load"]
    if l_id != "3"
        l["pd"] = 0
        l["qd"] = 0
    else
        l["pd"] = l["pd"]*2
        l["qd"] = l["qd"]*2
    end
end

#for (l_id,l) in data_39_acdc["branch"]
#    l["rate_a"] = 0.8
#end
=#

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_39_ac    = _PMACDC.run_acdcopf(data_39_acdc,ACPPowerModel,ipopt; setting = s_dual)

# Solving AC OTS with OTS only on the AC grid part
result_AC_ots_39    = _PMTP.run_acdcots_AC(data_39_acdc,ACPPowerModel,juniper; setting = s)

count_ = 0
for (br_id,br) in result_AC_ots_39["solution"]["branch"]
    if br["br_status"] == 0.0
        count_ += 1
    end
end

# Solving AC OTS with OTS only on the DC grid part 
result_DC_ots_39    = _PMTP.run_acdcots_DC(data_39_acdc,ACPPowerModel,juniper; setting = s)

# Solving AC OTS with OTS on both AC and DC grid part
result_AC_DC_ots_39    = _PMTP.run_acdcots_AC_DC(data_39_acdc,ACPPowerModel,juniper; setting = s)
count_ = 0
for (br_id,br) in result_AC_DC_ots_39["solution"]["branch"]
    if br["br_status"] == 0.0
        count_ += 1
    end
end

data_39_acdc["branchdc"]["2"]
data_39_acdc["branchdc"]["4"]
data_39_acdc["branchdc"]["12"]

# it actually works, multiple branchdc are connected to the convdc 2
data_39_acdc["convdc"]["4"]
data_39_acdc["convdc"]["2"]
data_39_acdc["convdc"]["3"]
data_39_acdc["convdc"]["10"]
data_39_acdc["convdc"]["8"]



AC_DC_OTS_br = [[br_id,br["br_status"]] for (br_id,br) in result_AC_DC_ots_39["solution"]["branchdc"]]
AC_DC_OTS_conv = [[br_id,br["conv_status"]] for (br_id,br) in result_AC_DC_ots_39["solution"]["convdc"]]


#######################################################################################
## Busbar splitting models ##
#######################################################################################
# AC BS for AC/DC grid with AC switches state as decision variable

data_busbars_ac_split_39_acdc = deepcopy(data_39_acdc)
data_busbars_ac_split_39_acdc_no_OTS = deepcopy(data_39_acdc)

#splitted_bus_ac converging -> 4, 7, 11, 14, 17, 19, 29
result_AC_DC_39_switches_AC_buses = Dict{String,Any}()
result_AC_DC_39_switches_AC_no_OTS = Dict{String,Any}()
term = []
obj = []
term_no_OTS = []
obj_no_OTS = []
for i in 1:39
    data_busbars_ac_split_39_acdc = deepcopy(data_39_acdc)
    data_busbars_ac_split_39_acdc_no_OTS = deepcopy(data_39_acdc)
    splitted_bus_ac = i
    data_busbars_ac_split_39_acdc,  switches_couples_ac_5,  extremes_ZILs_39_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_39_acdc,splitted_bus_ac)
    result_AC_DC_39_switches_AC = _PMTP.run_acdcsw_AC(data_busbars_ac_split_39_acdc,ACPPowerModel,juniper)
    #data_busbars_ac_split_39_acdc_no_OTS,  switches_couples_ac_5_no_OTS,  extremes_ZILs_39_ac_no_OTS  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_39_acdc_no_OTS,splitted_bus_ac)
    #result_AC_DC_39_switches_AC_no_OTS  = _PMTP.run_acdcsw_AC_no_OTS(data_busbars_ac_split_39_acdc_no_OTS,ACPPowerModel,juniper)
    #if result_AC_DC_39_switches_AC["termination_status"] != "LOCALLY_INFEASIBLE::TerminationStatusCode = 5"
    result_AC_DC_39_switches_AC_buses["$i"] = deepcopy(result_AC_DC_39_switches_AC)
    #end
    push!(term,result_AC_DC_39_switches_AC["termination_status"])
    push!(obj,result_AC_DC_39_switches_AC["objective"])

    #if result_AC_DC_39_switches_AC_no_OTS["termination_status"] != "LOCALLY_INFEASIBLE::TerminationStatusCode = 5"
    #    result_AC_DC_39_switches_AC_no_OTS["$i"] = deepcopy(result_AC_DC_39_switches_AC_no_OTS)
    #end
    #push!(term_no_OTS,result_AC_DC_39_switches_AC_no_OTS["termination_status"])
    #push!(obj_no_OTS,result_AC_DC_39_switches_AC_no_OTS["objective"])
end

data_busbars_ac_split_39_acdc = deepcopy(data_39_acdc)
data_busbars_ac_split_39_acdc_no_OTS = deepcopy(data_39_acdc)
splitted_bus_ac = [16, 26, 30, 1, 4, 21, 39]
#splitted_bus_ac = 16

#splitted_bus_ac = collect(20:30)

data_busbars_ac_split_39_acdc,  switches_couples_ac_39,  extremes_ZILs_39_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_39_acdc,splitted_bus_ac)
result_AC_DC_39_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_39_acdc,ACPPowerModel,juniper)

data_busbars_ac_split_39_acdc_no_OTS,  switches_couples_ac_5_no_OTS,  extremes_ZILs_39_ac_no_OTS  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_39_acdc_no_OTS,splitted_bus_ac)
result_AC_DC_39_switches_AC_no_OTS  = _PMTP.run_acdcsw_AC_no_OTS(data_busbars_ac_split_39_acdc_no_OTS,ACPPowerModel,juniper)

count_ = 0
for (br_id,br) in result_AC_DC_39_switches_AC["solution"]["branch"]
    if br["status"] == 0.0
        count_ += 1
    end
end
count_ = 0
for (br_id,br) in result_AC_DC_39_switches_AC_no_OTS["solution"]["branch"]
    if br["br_status"] == 0.0
        count_ += 1
    end
end


data_busbars_ac_dc_split_39_dc = deepcopy(data_39_acdc)
data_busbars_ac_split_39_dc_no_OTS = deepcopy(data_39_acdc)
#splitted_bus_ac = [16, 26, 30, 1, 4, 21, 39]
#splitted_bus_ac = 16
splitted_bus_dc = collect(1:5)

data_busbars_ac_dc_split_39_acdc_dc_sw , dc_switches_couples_dc_39, dc_extremes_ZILs_39_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_39_dc,splitted_bus_dc)
result_AC_DC_39_switch_DC  = _PMTP.run_acdcsw_DC(data_busbars_ac_dc_split_39_acdc_dc_sw, ACPPowerModel,juniper)







data_busbars_ac_dc_split_39_acdc = deepcopy(data_39_acdc)
data_busbars_ac_split_39_acdc_no_OTS = deepcopy(data_39_acdc)
#splitted_bus_ac = [16, 26, 30, 1, 4, 21, 39]
splitted_bus_ac = 16
splitted_bus_dc = collect(1:10)

data_busbars_ac_dc_split_39_acdc_ac_sw,  ac_switches_couples_ac_dc_39, ac_extremes_ZILs_39_ac_dc  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_39_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_39_acdc_ac_dc_sw , dc_switches_couples_ac_dc_39, dc_extremes_ZILs_39_ac_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_39_acdc_ac_sw,splitted_bus_dc)
result_AC_DC_39_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_39_acdc_ac_dc_sw, ACPPowerModel,juniper)


result_1_5_dc = deepcopy(result_AC_DC_39_switch_AC_DC)
data_1_5_dc = deepcopy(data_busbars_ac_dc_split_39_acdc_ac_dc_sw)


result_1_10_dc = deepcopy(result_AC_DC_39_switch_AC_DC)
data_1_10_dc = deepcopy(data_busbars_ac_dc_split_39_acdc_ac_dc_sw)


for i in collect(1:78)
    #(br_id,br) in result_AC_DC_5_switches_AC["solution"]["switch"]
    if !haskey(data_busbars_ac_dc_split_39_acdc_ac_dc_sw["dcswitch"]["$i"],"original")
        print([i,data_busbars_ac_dc_split_39_acdc_ac_dc_sw["dcswitch"]["$i"]["t_busdc"],result_1_10_dc["solution"]["dcswitch"]["$i"]["status"]],"\n")    
    else
        print([i,data_busbars_ac_dc_split_39_acdc_ac_dc_sw["dcswitch"]["$i"]["original"],data_busbars_ac_dc_split_39_acdc_ac_dc_sw["dcswitch"]["$i"]["auxiliary"],data_busbars_ac_dc_split_39_acdc_ac_dc_sw["dcswitch"]["$i"]["t_busdc"],result_1_10_dc["solution"]["dcswitch"]["$i"]["status"]],"\n")
    end
end







data_busbars_ac_dc_split_39_acdc_ac_sw_no_OTS,  ac_switches_couples_ac_dc_39_no_OTS, ac_extremes_ZILs_39_ac_dc_no_OTS  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_39_acdc_no_OTS,splitted_bus_ac)
data_busbars_ac_dc_split_39_acdc_ac_dc_sw_no_OTS , dc_switches_couples_ac_dc_39_no_OTS, dc_extremes_ZILs_39_ac_dc_no_OTS  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_39_acdc_ac_sw_no_OTS,splitted_bus_dc)
result_AC_DC_39_switch_AC_DC_no_OTS  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_39_acdc_ac_dc_sw_no_OTS, ACPPowerModel,juniper)


obj = [dc_switch["status"] for (sw_id,dc_switch) in result_AC_DC_39_switch_AC_DC["solution"]["dcswitch"]]
obj_no_OTS = [dc_switch["status"] for (sw_id,dc_switch) in result_AC_DC_39_switch_AC_DC_no_OTS["solution"]["dcswitch"]]

#=
# AC OTS for AC/DC grid with DC switches state as decision variable
data_busbars_dc_split_39_acdc = deepcopy(data_39_acdc)
splitted_bus_dc = [1,2,3]
data_busbars_dc_split_39_acdc , switches_couples_dc_5,  extremes_ZILs_39_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_39_acdc,splitted_bus_dc)
result_AC_DC_39_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_39_acdc, ACPPowerModel,juniper)


# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_busbars_ac_dc_split_39_acdc = deepcopy(data_39_acdc)
splitted_bus_ac = [2,4]
splitted_bus_dc = 1

#splitted_bus_dc = 1 # With this configuration it works!
#splitted_bus_dc = 2

data_busbars_ac_dc_split_39_acdc_ac_sw,  ac_switches_couples_ac_dc_5, ac_extremes_ZILs_39_ac_dc  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_39_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_39_acdc_ac_dc_sw , dc_switches_couples_ac_dc_5, dc_extremes_ZILs_39_ac_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_39_acdc_ac_sw,splitted_bus_dc)

result_AC_DC_39_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_39_acdc, ACPPowerModel,juniper)
=#


ac_branches_power_flow = Dict{String,Any}()
for (br_id,br) in data_39_acdc["branch"]
    ac_branches_power_flow[br_id] = Dict{String,Any}()
    ac_branches_power_flow[br_id]["OPF"] = result_opf_39_ac["solution"]["branch"][br_id]["pt"]
    ac_branches_power_flow[br_id]["OTS"] = result_AC_ots_5["solution"]["branch"][br_id]["pt"]
    ac_branches_power_flow[br_id]["BS"] = result_AC_DC_39_switches_AC["solution"]["branch"][br_id]["pt"]
    ac_branches_power_flow[br_id]["BS_NO_OTS"] = result_AC_DC_39_switches_AC_no_OTS["solution"]["branch"][br_id]["pt"]
end

dc_branches_power_flow = Dict{String,Any}()
for (br_id,br) in data_39_acdc["branchdc"]
    dc_branches_power_flow[br_id] = Dict{String,Any}()
    dc_branches_power_flow[br_id]["OPF"] = result_opf_39_ac["solution"]["branchdc"][br_id]["pt"]
    dc_branches_power_flow[br_id]["OTS"] = result_AC_ots_5["solution"]["branchdc"][br_id]["pt"]
    dc_branches_power_flow[br_id]["BS"] = result_AC_DC_39_switches_AC["solution"]["branchdc"][br_id]["pt"]
    dc_branches_power_flow[br_id]["BS_NO_OTS"] = result_AC_DC_39_switches_AC_no_OTS["solution"]["branchdc"][br_id]["pt"]    
end

pg_gen = Dict{String,Any}()
for (br_id,br) in data_39_acdc["gen"]
    pg_gen[br_id] = Dict{String,Any}()
    pg_gen[br_id]["OPF"] = result_opf_39_ac["solution"]["gen"][br_id]["pg"]
    pg_gen[br_id]["OTS"] = result_AC_ots_5["solution"]["gen"][br_id]["pg"]
    pg_gen[br_id]["BS"] = result_AC_DC_39_switches_AC["solution"]["gen"][br_id]["pg"]
    pg_gen[br_id]["BS_NO_OTS"] = result_AC_DC_39_switches_AC_no_OTS["solution"]["gen"][br_id]["pg"]
end

qg_gen = Dict{String,Any}()
for (br_id,br) in data_39_acdc["gen"]
    qg_gen[br_id] = Dict{String,Any}()
    qg_gen[br_id]["OPF"] = result_opf_39_ac["solution"]["gen"][br_id]["qg"]
    qg_gen[br_id]["OTS"] = result_AC_ots_5["solution"]["gen"][br_id]["qg"]
    qg_gen[br_id]["BS"] = result_AC_DC_39_switches_AC["solution"]["gen"][br_id]["qg"]
    qg_gen[br_id]["BS_NO_OTS"] = result_AC_DC_39_switches_AC_no_OTS["solution"]["gen"][br_id]["qg"]
end

branches_ots = Dict{String,Any}()
for (br_id,br) in data_39_acdc["branch"]
    branches_ots[br_id] = Dict{String,Any}()
    branches_ots[br_id]["f_bus"] = br["f_bus"]
    branches_ots[br_id]["t_bus"] = br["t_bus"]
    branches_ots[br_id]["status"] = result_AC_ots_5["solution"]["branch"][br_id]["br_status"]
    branches_ots[br_id]["status"] = result_AC_ots_5["solution"]["branch"][br_id]["br_status"]
end

voltage_angles = Dict{String,Any}()
for (br_id,br) in data_39_acdc["bus"]
    voltage_angles[br_id] = Dict{String,Any}()
    voltage_angles[br_id]["OPF"] = result_opf_39_ac["solution"]["bus"][br_id]["va"]
    voltage_angles[br_id]["OTS"] = result_AC_ots_5["solution"]["bus"][br_id]["va"]
    voltage_angles[br_id]["BS"] = result_AC_DC_39_switches_AC["solution"]["bus"][br_id]["va"]
    voltage_angles[br_id]["BS_NO_OTS"] = result_AC_DC_39_switches_AC_no_OTS["solution"]["bus"][br_id]["va"]
end

voltage_magnitudes = Dict{String,Any}()
for (br_id,br) in data_39_acdc["bus"]
    voltage_magnitudes[br_id] = Dict{String,Any}()
    voltage_magnitudes[br_id]["OPF"] = result_opf_39_ac["solution"]["bus"][br_id]["vm"]
    voltage_magnitudes[br_id]["OTS"] = result_AC_ots_5["solution"]["bus"][br_id]["vm"]
    voltage_magnitudes[br_id]["BS"] = result_AC_DC_39_switches_AC["solution"]["bus"][br_id]["vm"]
    voltage_magnitudes[br_id]["BS_NO_OTS"] = result_AC_DC_39_switches_AC_no_OTS["solution"]["bus"][br_id]["vm"]
end

for (br_id,br) in result_AC_DC_39_switches_AC["solution"]["bus"]
    print([br_id,br["vm"],br["va"]],"\n")
end

switches = Dict{String,Any}()
for (br_id,br) in data_busbars_ac_split_39_acdc["switch"]
    switches[br_id] = Dict{String,Any}()
    switches[br_id]["f_bus"] = br["f_bus"]
    switches[br_id]["t_bus"] = br["t_bus"]
    if haskey(br,"original")
        switches[br_id]["original"] = br["original"]
    else
        switches[br_id]["original"] = false
    end
    if haskey(br,"auxiliary")
        switches[br_id]["auxiliary"] = br["auxiliary"]
    else
        switches[br_id]["auxiliary"] = false
    end
    if haskey(br,"bus_split")
        switches[br_id]["bus_split"] = br["bus_split"]
    else
        switches[br_id]["bus_split"] = false
    end
    if haskey(data_busbars_ac_split_39_acdc["switch_couples"],br_id)
        switches[br_id]["switch_couple"] = data_busbars_ac_split_39_acdc["switch_couples"][br_id]
    else
        switches[br_id]["switch_couple"] = false
    end
    switches[br_id]["status"] = result_AC_DC_39_switches_AC["solution"]["switch"][br_id]["status"]
end

switches_no_OTS = Dict{String,Any}()
for (br_id,br) in data_busbars_ac_split_39_acdc_no_OTS["switch"]
    switches_no_OTS[br_id] = Dict{String,Any}()
    switches_no_OTS[br_id]["f_bus"] = br["f_bus"]
    switches_no_OTS[br_id]["t_bus"] = br["t_bus"]
    if haskey(br,"original")
        switches_no_OTS[br_id]["original"] = br["original"]
    else
        switches_no_OTS[br_id]["original"] = false
    end
    if haskey(br,"auxiliary")
        switches_no_OTS[br_id]["auxiliary"] = br["auxiliary"]
    else
        switches_no_OTS[br_id]["auxiliary"] = false
    end
    if haskey(br,"bus_split")
        switches_no_OTS[br_id]["bus_split"] = br["bus_split"]
    else
        switches_no_OTS[br_id]["bus_split"] = false
    end
    if haskey(data_busbars_ac_split_39_acdc_no_OTS["switch_couples"],br_id)
        switches_no_OTS[br_id]["switch_couple"] = data_busbars_ac_split_39_acdc_no_OTS["switch_couples"][br_id]
    else
        switches_no_OTS[br_id]["switch_couple"] = false
    end
    switches_no_OTS[br_id]["status"] = result_AC_DC_39_switches_AC_no_OTS["solution"]["switch"][br_id]["status"]
end

v_as = [[i,result_AC_DC_39_switches_AC["solution"]["bus"][i]["va"]] for i in eachindex(result_AC_DC_39_switches_AC["solution"]["bus"])]
v_as_no_OTS = [[i,result_AC_DC_39_switches_AC_no_OTS["solution"]["bus"][i]["va"]] for i in eachindex(result_AC_DC_39_switches_AC_no_OTS["solution"]["bus"])]



switches = [[i,result_AC_DC_39_switches_AC["solution"]["switch"][i]["status"]] for i in eachindex(result_AC_DC_39_switches_AC["solution"]["switch"])]
switches_no_OTS = [[i,result_AC_DC_39_switches_AC_no_OTS["solution"]["switch"][i]["status"]] for i in eachindex(result_AC_DC_39_switches_AC_no_OTS["solution"]["switch"])]




branches = Dict{String,Any}()
for (br_id,br) in data_39_acdc["branch"]
    branches[br_id] = Dict{String,Any}()
    branches[br_id]["f_bus"] = br["f_bus"]
    branches[br_id]["t_bus"] = br["t_bus"]
end


