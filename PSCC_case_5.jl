using PowerModels; const _PM = PowerModels
using Ipopt, JuMP
using HiGHS, Gurobi, Juniper
using PowerModelsACDC; const _PMACDC = PowerModelsACDC
import PowerModelsTopologicalActionsII; const _PMTP = PowerModelsTopologicalActionsII
using PowerModelsTopologicalActionsII   
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
test_case_5_acdc = "case67_acdc.m"
#######################################################################################
## Parsing input data ##
#######################################################################################

s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file_5_acdc = joinpath(@__DIR__,"data_sources",test_case_5_acdc)
data_original_5_acdc = _PM.parse_file(data_file_5_acdc)

# If these lines are deleted, OPF, OTS and BS have the same results
#delete!(data_original_5_acdc["branch"],"1")
#delete!(data_original_5_acdc["branch"],"2")
#delete!(data_original_5_acdc["branch"],"3")
#delete!(data_original_5_acdc["branch"],"4")
#delete!(data_original_5_acdc["branch"],"5")
#delete!(data_original_5_acdc["branch"],"6")
#delete!(data_original_5_acdc["branch"],"7")

data_5_acdc = deepcopy(data_original_5_acdc)

_PMACDC.process_additional_data!(data_5_acdc)


# Configuration 1 -> normal case 5
#for (g_id,g) in data_5_acdc["gen"]
#    g["cost"][1] = g["cost"][1]*1000 
#end

# Configuration 2 -> this case, good for testing the methodology, not great improvement in gen costs
# 4 AC lines off
# 1 DC line off
#for (cv_id,cv) in data_5_acdc["load"]
#    if cv_id != "2"
#        cv["pd"] = 0
#        cv["qd"] = 0
#    else
#        cv["pd"] = cv["pd"]/10
#        cv["qd"] = cv["qd"]/10
#    end
#end

# Configuration 3 -> isolating load 4 and putting extremely high cost gen 2
#for (cv_id,cv) in data_5_acdc["load"]
#    if cv_id != "2"
#        cv["pd"] = 0
#        cv["qd"] = 0
#    else
#        cv["pd"] = cv["pd"]*3
#        cv["qd"] = cv["qd"]*3
#    end
#end

# Configuration 4 -> Limiting branch 1 and increasing the generation cost of the gen 2
data_5_acdc["gen"]["2"]["cost"][1] = 10000.0
data_5_acdc["branch"]["1"]["rate_a"] = 0.5

#######################################################################################
## Optimal transmission switching models ##
#######################################################################################
# AC OPF for ACDC grid
result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,ACPPowerModel,ipopt; setting = s_dual)

result_opf_5_ac    = _PMACDC.run_acdcopf(data_5_acdc,SOCWRPowerModel,ipopt; setting = s_dual)


# Solving AC OTS with OTS only on the AC grid part
result_AC_ots_5    = _PMTP.run_acdcots_AC(data_5_acdc,ACPPowerModel,juniper; setting = s)

result_AC_ots_5    = _PMTP.run_acdcots_AC(data_5_acdc,SOCWRPowerModel,juniper; setting = s)

# Solving AC OTS with OTS only on the DC grid part 
result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)

result_DC_ots_5    = _PMTP.run_acdcots_DC(data_5_acdc,SOCWRPowerModel,juniper; setting = s)

# Solving AC OTS with OTS on both AC and DC grid part
result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)

result_AC_DC_ots_5    = _PMTP.run_acdcots_AC_DC(data_5_acdc,ACPPowerModel,juniper; setting = s)

##############
for (br_id,br) in result_AC_ots_5["solution"]["branch"]
    print([br_id,br["br_status"]],"\n")
end

for (br_id,br) in result_AC_DC_ots_5["solution"]["branch"]
    print([br_id,br["br_status"]],"\n")
end

for (br_id, br) in result_opf_5_ac["solution"]["branch"]
    print([br_id,br["pf"],br["qf"]],"\n")
end

for (br_id, br) in result_opf_5_ac["solution"]["branch"]
    print([br_id,"Utilization AC branch OPF "*"$(br["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100)"*" %",br["pf"]],"\n")
    print([br_id,"Utilization AC branch AC OTS "*"$(result_AC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100)"*" %",result_AC_ots_5["solution"]["branch"][br_id]["pf"]],"\n")#br["qf"]/data_5_acdc["branch"][br_id]["rate_a"]],"\n")
    print([br_id,"Utilization AC branch DC OTS "*"$(result_DC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100)"*" %",result_DC_ots_5["solution"]["branch"][br_id]["pf"]],"\n")#br["qf"]/data_5_acdc["branch"][br_id]["rate_a"]],"\n")
    print([br_id,"Utilization AC branch AC/DC OTS "*"$(result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"]/data_5_acdc["branch"][br_id]["rate_a"]*100)"*" %",result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"]],"\n")#br["qf"]/data_5_acdc["branch"][br_id]["rate_a"]],"\n")
    print("\n")
end

for (br_id, br) in result_opf_5_ac["solution"]["branchdc"]
    print([br_id,"Utilization DC branch OPF "*"$(br["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100)"*" %",br["pf"]],"\n")
    print([br_id,"Utilization DC branch AC OTS "*"$(result_AC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100)"*" %",result_AC_ots_5["solution"]["branchdc"][br_id]["pf"]],"\n")#br["qf"]/data_5_acdc["branch"][br_id]["rate_a"]],"\n")
    print([br_id,"Utilization DC branch DC OTS "*"$(result_DC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100)"*" %",result_DC_ots_5["solution"]["branchdc"][br_id]["pf"]],"\n")#br["qf"]/data_5_acdc["branch"][br_id]["rate_a"]],"\n")
    print([br_id,"Utilization DC branch AC/DC OTS "*"$(result_AC_DC_ots_5["solution"]["branchdc"][br_id]["pf"]/data_5_acdc["branchdc"][br_id]["rateA"]*100)"*" %",result_AC_DC_ots_5["solution"]["branch"][br_id]["pf"]],"\n")#br["qf"]/data_5_acdc["branch"][br_id]["rate_a"]],"\n")
    print("\n")
end


#######################################################################################
## Busbar splitting models ##
#######################################################################################

# AC BS for AC/DC grid with AC switches state as decision variable
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Busbars being split
#splitted_bus_ac = [1,2,3,4,5]
splitted_bus_ac = [2,4]

data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,ACPPowerModel,juniper)
switches_results = []
for i in 1:length(result_AC_DC_5_switches_AC["solution"]["switch"])
    push!(switches_results,result_AC_DC_5_switches_AC["solution"]["switch"]["$i"]["status"])
end

# Working on QC and SOC relaxations
data_busbars_ac_split_5_acdc = deepcopy(data_5_acdc)
data_busbars_ac_split_5_acdc_no_OTS = deepcopy(data_5_acdc)

# Busbars being split
#splitted_bus_ac = [1,2,3,4,5]
splitted_bus_ac = [2,4]

data_busbars_ac_split_5_acdc,  switches_couples_ac_5,  extremes_ZILs_5_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc,splitted_bus_ac)
result_AC_DC_5_switches_AC  = _PMTP.run_acdcsw_AC(data_busbars_ac_split_5_acdc,SOCWRPowerModel,juniper)
switches_results = []
for i in 1:length(result_AC_DC_5_switches_AC["solution"]["switch"])
    push!(switches_results,result_AC_DC_5_switches_AC["solution"]["switch"]["$i"]["status"])
end




# AC OTS for AC/DC grid with DC switches state as decision variable
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)
splitted_bus_dc = [1,2,3]
data_busbars_dc_split_5_acdc , switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc,splitted_bus_dc)
result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, SOCWRPowerModel,juniper)





# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
splitted_bus_ac = [2,4]
splitted_bus_dc = [1,2,3]
#data_busbars_ac_dc_split_5_acdc["branchdc"]["2"]["status"] = 0

data_busbars_ac_dc_split_5_acdc_ac_sw,  ac_switches_couples_ac_dc_5, ac_extremes_ZILs_5_ac_dc  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc_ac_dc_sw , dc_switches_couples_ac_dc_5, dc_extremes_ZILs_5_ac_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_ac_sw,splitted_bus_dc)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, SOCWRPowerModel,juniper)



data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
splitted_bus_ac = [2,4]
splitted_bus_dc = [1,2,3]
#data_busbars_ac_dc_split_5_acdc["branchdc"]["2"]["status"] = 0

data_busbars_ac_dc_split_5_acdc_ac_sw,  ac_switches_couples_ac_dc_5, ac_extremes_ZILs_5_ac_dc  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc_ac_dc_sw , dc_switches_couples_ac_dc_5, dc_extremes_ZILs_5_ac_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_ac_sw,splitted_bus_dc)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc, QCRMPowerModel,juniper)

















data_busbars_ac_split_5_acdc_no_OTS,  switches_couples_ac_5_no_OTS,  extremes_ZILs_5_ac_no_OTS  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split_5_acdc_no_OTS,splitted_bus_ac)
result_AC_DC_5_switches_AC_no_OTS  = _PMTP.run_acdcsw_AC_no_OTS(data_busbars_ac_split_5_acdc_no_OTS,ACPPowerModel,juniper)

result_1 = deepcopy(result_AC_DC_5_switches_AC)
#result_1_no_OTS = deepcopy(result_AC_DC_5_switches_AC_no_OTS)


for i in collect(1:length(result_AC_DC_5_switches_AC["solution"]["switch"]))
    #(br_id,br) in result_AC_DC_5_switches_AC["solution"]["switch"]
    if !haskey(data_busbars_ac_split_5_acdc["switch"]["$i"],"original")
        print([i,data_busbars_ac_split_5_acdc["switch"]["$i"]["f_bus"],data_busbars_ac_split_5_acdc["switch"]["$i"]["t_bus"],result_AC_DC_5_switches_AC["solution"]["switch"]["$i"]["status"]],"\n")    
    else
        print([i,data_busbars_ac_split_5_acdc["switch"]["$i"]["original"],data_busbars_ac_split_5_acdc["switch"]["$i"]["auxiliary"],data_busbars_ac_split_5_acdc["switch"]["$i"]["t_bus"],result_AC_DC_5_switches_AC["solution"]["switch"]["$i"]["status"]],"\n")
    end
end


# AC OTS for AC/DC grid with DC switches state as decision variable
data_busbars_dc_split_5_acdc = deepcopy(data_5_acdc)
splitted_bus_dc = [1,2,3]
data_busbars_dc_split_5_acdc , switches_couples_dc_5,  extremes_ZILs_5_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split_5_acdc,splitted_bus_dc)
result_AC_DC_5_switches_DC  = _PMTP.run_acdcsw_DC(data_busbars_dc_split_5_acdc, ACPPowerModel,juniper)


# AC OTS for AC/DC grid with AC and DC switches state as decision variable
data_busbars_ac_dc_split_5_acdc = deepcopy(data_5_acdc)
splitted_bus_ac = [2,4]
splitted_bus_dc = [1,2,3]
#data_busbars_ac_dc_split_5_acdc["branchdc"]["2"]["status"] = 0

data_busbars_ac_dc_split_5_acdc_ac_sw,  ac_switches_couples_ac_dc_5, ac_extremes_ZILs_5_ac_dc  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc,splitted_bus_ac)
data_busbars_ac_dc_split_5_acdc_ac_dc_sw , dc_switches_couples_ac_dc_5, dc_extremes_ZILs_5_ac_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_ac_sw,splitted_bus_dc)
result_AC_DC_5_switch_AC_DC  = _PMTP.run_acdcsw_AC_DC(data_busbars_ac_dc_split_5_acdc, ACPPowerModel,juniper)

result_1_acdc = deepcopy(result_AC_DC_5_switch_AC_DC)

for i in collect(1:length(result_AC_DC_5_switch_AC_DC["solution"]["switch"]))
    #(br_id,br) in result_AC_DC_5_switches_AC["solution"]["switch"]
    if !haskey(data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["$i"],"original")
        print([i,data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["$i"]["t_bus"],result_AC_DC_5_switch_AC_DC["solution"]["switch"]["$i"]["status"]],"\n")    
    else
        print([i,data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["$i"]["original"],data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["$i"]["auxiliary"],data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]["$i"]["t_bus"],result_AC_DC_5_switch_AC_DC["solution"]["switch"]["$i"]["status"]],"\n")
    end
end

for i in collect(1:length(result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"]))
    #(br_id,br) in result_AC_DC_5_dcswitches_AC["solution"]["dcswitch"]
    if !haskey(data_busbars_ac_dc_split_5_acdc_ac_dc_sw["dcswitch"]["$i"],"original")
        print([i,data_busbars_ac_dc_split_5_acdc_ac_dc_sw["dcswitch"]["$i"]["t_busdc"],result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"]["$i"]["status"]],"\n")    
    else
        print([i,data_busbars_ac_dc_split_5_acdc_ac_dc_sw["dcswitch"]["$i"]["original"],data_busbars_ac_dc_split_5_acdc_ac_dc_sw["dcswitch"]["$i"]["auxiliary"],data_busbars_ac_dc_split_5_acdc_ac_dc_sw["dcswitch"]["$i"]["t_busdc"],result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"]["$i"]["status"]],"\n")
    end
end

for (br_id, br) in result_AC_DC_5_switch_AC_DC["solution"]["branch"]
    print([br_id,br["pf"],br["qf"]],"\n")
end


for (br_id, br) in result_AC_DC_5_switches_AC["solution"]["branch"]
    print([br_id,"Utilization AC branch OPF "*"$(result_opf_5_ac["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]*100)"*" %",result_opf_5_ac["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]],"\n")
    print([br_id,"Utilization AC branch AC BS "*"$(br["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]*100)"*" %",br["qf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]],"\n")
    print([br_id,"Utilization AC branch DC BS "*"$(result_AC_DC_5_switches_DC["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]*100)"*" %",result_AC_DC_5_switches_DC["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]],"\n")
    print([br_id,"Utilization AC branch AC/DC BS "*"$(result_AC_DC_5_switch_AC_DC["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]*100)"*" %",result_AC_DC_5_switch_AC_DC["solution"]["branch"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branch"][br_id]["rate_a"]],"\n")
    print("\n")
end

for (br_id, br) in result_AC_DC_5_switches_AC["solution"]["branchdc"]
    print([br_id,"Utilization DC branch OPF "*"$(result_opf_5_ac["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100)"*" %",result_opf_5_ac["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]],"\n")
    print([br_id,"Utilization DC branch AC BS "*"$(br["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100)"*" %",br["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]],"\n")
    print([br_id,"Utilization DC branch DC BS "*"$(result_AC_DC_5_switches_DC["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100)"*" %",result_AC_DC_5_switches_DC["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]],"\n")
    print([br_id,"Utilization DC branch AC/DC BS "*"$(result_AC_DC_5_switch_AC_DC["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100)"*" %",result_AC_DC_5_switch_AC_DC["solution"]["branchdc"][br_id]["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]],"\n")
    print("\n")
end



for (br_id, br) in result_AC_DC_5_switch_AC_DC["solution"]["branchdc"]
    print([br_id,br["pf"],br["pt"]],"\n")
end

for (br_id, br) in result_AC_DC_5_switch_AC_DC["solution"]["branchdc"]
    print([br_id,"Utilization DC branch "*"$(br["pf"]/data_busbars_ac_dc_split_5_acdc_ac_dc_sw["branchdc"][br_id]["rateA"]*100)"*" %",br["pt"]],"\n")
end


# AC OTS for AC/DC grid with AC and DC switches state as decision variable NO OTS
#data_busbars_ac_dc_split_5_acdc_no_OTS = deepcopy(data_5_acdc)
#splitted_bus_ac = [2,4]
#splitted_bus_dc = [1,2,3]
##splitted_bus_dc = 1
#
#data_busbars_ac_dc_split_5_acdc_ac_sw_no_OTS,  ac_switches_couples_ac_dc_5_no_OTS, ac_extremes_ZILs_5_ac_dc_no_OTS  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_no_OTS,splitted_bus_ac)
#data_busbars_ac_dc_split_5_acdc_ac_dc_sw_no_OTS , dc_switches_couples_ac_dc_5_no_OTS, dc_extremes_ZILs_5_ac_dc_no_OTS  = _PMTP.DC_busbar_split_more_buses(data_busbars_ac_dc_split_5_acdc_ac_sw_no_OTS,splitted_bus_dc)
#result_AC_DC_5_switch_AC_DC_no_OTS  = _PMTP.run_acdcsw_AC_DC_no_OTS(data_busbars_ac_dc_split_5_acdc_no_OTS, ACPPowerModel,juniper)



branches = Dict{String,Any}()
for (br_id,br) in data_5_acdc["branch"]
    branches[br_id] = Dict{String,Any}()
    branches[br_id]["f_bus"] = br["f_bus"]
    branches[br_id]["t_bus"] = br["t_bus"]
end

branches_dc = Dict{String,Any}()
for (br_id,br) in data_5_acdc["branchdc"]
    branches_dc[br_id] = Dict{String,Any}()
    branches_dc[br_id]["fbusdc"] = br["fbusdc"]
    branches_dc[br_id]["tbusdc"] = br["tbusdc"]
end











for (br_id,br) in result_AC_DC_5_switches_DC["solution"]["dcswitch"]
    print([br_id,br["status"]],"\n")
end

#=
for (br_id,br) in result_AC_DC_5_switch_AC_DC["solution"]["switch"]
    print([br_id,br["status"]],"\n")
end
for (br_id,br) in result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"]
    print([br_id,br["status"]],"\n")
end












for i in 1:21
    #for (swdc_id,swdc) in result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"]
    #print([swdc_id,swdc["status"]],"\n")
    #print([i,result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"]["$i"]["status"],data_busbars_ac_dc_split_5_acdc_ac_dc_sw["dcswitch"]["$i"]["f_busdc"],data_busbars_ac_dc_split_5_acdc_ac_dc_sw["dcswitch"]["$i"]["t_busdc"]],"\n")
    print([i,result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"]["$i"]["status"]],"\n")
end

#=
# To check the results
ac_branches_power_flow = Dict{String,Any}()
for (br_id,br) in data_5_acdc["branch"]
    ac_branches_power_flow[br_id] = Dict{String,Any}()
    ac_branches_power_flow[br_id]["OPF"] = result_opf_5_ac["solution"]["branch"][br_id]["pt"]
    ac_branches_power_flow[br_id]["OTS"] = result_AC_ots_5["solution"]["branch"][br_id]["pt"]
    ac_branches_power_flow[br_id]["BS"] = result_AC_DC_5_switches_AC["solution"]["branch"][br_id]["pt"]
    ac_branches_power_flow[br_id]["BS_NO_OTS"] = result_AC_DC_5_switches_AC_no_OTS["solution"]["branch"][br_id]["pt"]
end

dc_branches_power_flow = Dict{String,Any}()
for (br_id,br) in data_5_acdc["branchdc"]
    dc_branches_power_flow[br_id] = Dict{String,Any}()
    dc_branches_power_flow[br_id]["OPF"] = result_opf_5_ac["solution"]["branchdc"][br_id]["pt"]
    dc_branches_power_flow[br_id]["OTS"] = result_AC_ots_5["solution"]["branchdc"][br_id]["pt"]
    dc_branches_power_flow[br_id]["BS"] = result_AC_DC_5_switches_AC["solution"]["branchdc"][br_id]["pt"]
    dc_branches_power_flow[br_id]["BS_NO_OTS"] = result_AC_DC_5_switches_AC_no_OTS["solution"]["branchdc"][br_id]["pt"]    
end

pg_gen = Dict{String,Any}()
for (br_id,br) in data_5_acdc["gen"]
    pg_gen[br_id] = Dict{String,Any}()
    pg_gen[br_id]["OPF"] = result_opf_5_ac["solution"]["gen"][br_id]["pg"]
    pg_gen[br_id]["OTS"] = result_AC_ots_5["solution"]["gen"][br_id]["pg"]
    pg_gen[br_id]["BS"] = result_AC_DC_5_switches_AC["solution"]["gen"][br_id]["pg"]
    pg_gen[br_id]["BS_NO_OTS"] = result_AC_DC_5_switches_AC_no_OTS["solution"]["gen"][br_id]["pg"]
end

qg_gen = Dict{String,Any}()
for (br_id,br) in data_5_acdc["gen"]
    qg_gen[br_id] = Dict{String,Any}()
    qg_gen[br_id]["OPF"] = result_opf_5_ac["solution"]["gen"][br_id]["qg"]
    qg_gen[br_id]["OTS"] = result_AC_ots_5["solution"]["gen"][br_id]["qg"]
    qg_gen[br_id]["BS"] = result_AC_DC_5_switches_AC["solution"]["gen"][br_id]["qg"]
    qg_gen[br_id]["BS_NO_OTS"] = result_AC_DC_5_switches_AC_no_OTS["solution"]["gen"][br_id]["qg"]
end

branches_ots = Dict{String,Any}()
for (br_id,br) in data_5_acdc["branch"]
    branches_ots[br_id] = Dict{String,Any}()
    branches_ots[br_id]["f_bus"] = br["f_bus"]
    branches_ots[br_id]["t_bus"] = br["t_bus"]
    branches_ots[br_id]["status"] = result_AC_ots_5["solution"]["branch"][br_id]["br_status"]
    branches_ots[br_id]["status"] = result_AC_ots_5["solution"]["branch"][br_id]["br_status"]
end

voltage_angles = Dict{String,Any}()
for (br_id,br) in data_5_acdc["bus"]
    voltage_angles[br_id] = Dict{String,Any}()
    voltage_angles[br_id]["OPF"] = result_opf_5_ac["solution"]["bus"][br_id]["va"]
    voltage_angles[br_id]["OTS"] = result_AC_ots_5["solution"]["bus"][br_id]["va"]
    voltage_angles[br_id]["BS"] = result_AC_DC_5_switches_AC["solution"]["bus"][br_id]["va"]
    voltage_angles[br_id]["BS_NO_OTS"] = result_AC_DC_5_switches_AC_no_OTS["solution"]["bus"][br_id]["va"]
end

voltage_magnitudes = Dict{String,Any}()
for (br_id,br) in data_5_acdc["bus"]
    voltage_magnitudes[br_id] = Dict{String,Any}()
    voltage_magnitudes[br_id]["OPF"] = result_opf_5_ac["solution"]["bus"][br_id]["vm"]
    voltage_magnitudes[br_id]["OTS"] = result_AC_ots_5["solution"]["bus"][br_id]["vm"]
    voltage_magnitudes[br_id]["BS"] = result_AC_DC_5_switches_AC["solution"]["bus"][br_id]["vm"]
    voltage_magnitudes[br_id]["BS_NO_OTS"] = result_AC_DC_5_switches_AC_no_OTS["solution"]["bus"][br_id]["vm"]
end

for (br_id,br) in result_AC_DC_5_switches_AC["solution"]["bus"]
    print([br_id,br["vm"],br["va"]],"\n")
end

switches = Dict{String,Any}()
for (br_id,br) in data_busbars_ac_split_5_acdc["switch"]
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
    if haskey(data_busbars_ac_split_5_acdc["switch_couples"],br_id)
        switches[br_id]["switch_couple"] = data_busbars_ac_split_5_acdc["switch_couples"][br_id]
    else
        switches[br_id]["switch_couple"] = false
    end
    switches[br_id]["status"] = result_AC_DC_5_switches_AC["solution"]["switch"][br_id]["status"]
end

switches_dc = Dict{String,Any}()
for (br_id,br) in data_busbars_ac_split_5_acdc["switch"]
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
    if haskey(data_busbars_ac_split_5_acdc["switch_couples"],br_id)
        switches[br_id]["switch_couple"] = data_busbars_ac_split_5_acdc["switch_couples"][br_id]
    else
        switches[br_id]["switch_couple"] = false
    end
    switches[br_id]["status"] = result_AC_DC_5_switches_AC["solution"]["switch"][br_id]["status"]
end

switches_no_OTS = Dict{String,Any}()
for (br_id,br) in data_busbars_ac_split_5_acdc_no_OTS["switch"]
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
    if haskey(data_busbars_ac_split_5_acdc_no_OTS["switch_couples"],br_id)
        switches_no_OTS[br_id]["switch_couple"] = data_busbars_ac_split_5_acdc_no_OTS["switch_couples"][br_id]
    else
        switches_no_OTS[br_id]["switch_couple"] = false
    end
    switches_no_OTS[br_id]["status"] = result_AC_DC_5_switches_AC_no_OTS["solution"]["switch"][br_id]["status"]
end

v_as = [[i,result_AC_DC_5_switches_AC["solution"]["bus"][i]["va"]] for i in eachindex(result_AC_DC_5_switches_AC["solution"]["bus"])]
v_as_no_OTS = [[i,result_AC_DC_5_switches_AC_no_OTS["solution"]["bus"][i]["va"]] for i in eachindex(result_AC_DC_5_switches_AC_no_OTS["solution"]["bus"])]



switches_ = [[i,result_AC_DC_5_switches_AC["solution"]["switch"][i]["status"]] for i in eachindex(result_AC_DC_5_switches_AC["solution"]["switch"])]
switches_no_OTS_ = [[i,result_AC_DC_5_switches_AC_no_OTS["solution"]["switch"][i]["status"]] for i in eachindex(result_AC_DC_5_switches_AC_no_OTS["solution"]["switch"])]






switches_ac_dc = Dict{String,Any}()
for (br_id,br) in data_busbars_ac_dc_split_5_acdc_ac_dc_sw["switch"]
    switches_ac_dc[br_id] = Dict{String,Any}()
    switches_ac_dc[br_id]["f_bus"] = br["f_bus"]
    switches_ac_dc[br_id]["t_bus"] = br["t_bus"]
    if haskey(br,"original")
        switches_ac_dc[br_id]["original"] = br["original"]
    else
        switches_ac_dc[br_id]["original"] = false
    end
    if haskey(br,"auxiliary")
        switches_ac_dc[br_id]["auxiliary"] = br["auxiliary"]
    else
        switches_ac_dc[br_id]["auxiliary"] = false
    end
    if haskey(br,"bus_split")
        switches_ac_dc[br_id]["bus_split"] = br["bus_split"]
    else
        switches_ac_dc[br_id]["bus_split"] = false
    end
    if haskey(data_busbars_ac_split_5_acdc["switch_couples"],br_id)
        switches_ac_dc[br_id]["switch_couple"] = data_busbars_ac_split_5_acdc["switch_couples"][br_id]
    else
        switches_ac_dc[br_id]["switch_couple"] = false
    end
    switches_ac_dc[br_id]["status"] = result_AC_DC_5_switch_AC_DC["solution"]["switch"][br_id]["status"]
end

dc_switches_ac_dc = Dict{String,Any}()
for (br_id,br) in data_busbars_ac_dc_split_5_acdc_ac_dc_sw["dcswitch"]
    dc_switches_ac_dc[br_id] = Dict{String,Any}()
    dc_switches_ac_dc[br_id]["f_busdc"] = br["f_busdc"]
    dc_switches_ac_dc[br_id]["t_busdc"] = br["t_busdc"]
    if haskey(br,"original")
        dc_switches_ac_dc[br_id]["original"] = br["original"]
    else
        dc_switches_ac_dc[br_id]["original"] = false
    end
    if haskey(br,"auxiliary")
        dc_switches_ac_dc[br_id]["auxiliary"] = br["auxiliary"]
    else
        dc_switches_ac_dc[br_id]["auxiliary"] = false
    end
    if haskey(br,"bus_split")
        dc_switches_ac_dc[br_id]["busdc_split"] = br["busdc_split"]
    else
        dc_switches_ac_dc[br_id]["busdc_split"] = false
    end
    if haskey(data_busbars_ac_split_5_acdc["switch_couples"],br_id)
        dc_switches_ac_dc[br_id]["switch_couple"] = data_busbars_ac_split_5_acdc["switch_couples"][br_id]
    else
        dc_switches_ac_dc[br_id]["switch_couple"] = false
    end
    dc_switches_ac_dc[br_id]["status"] = result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"][br_id]["status"]
end


switches_ac_dc_ = [[i,result_AC_DC_5_switch_AC_DC["solution"]["switch"][i]["status"]] for i in eachindex(result_AC_DC_5_switch_AC_DC["solution"]["switch"])]
dc_switches_ac_dc_ = [[i,result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"][i]["status"]] for i in eachindex(result_AC_DC_5_switch_AC_DC["solution"]["dcswitch"])]
=#

result_opf_5_ac["solution"]["convdc"]["1"]
result_AC_DC_5_switch_AC_DC["solution"]["convdc"]["1"]

result_opf_5_ac["solution"]["convdc"]["2"]
result_AC_DC_5_switch_AC_DC["solution"]["convdc"]["2"]

