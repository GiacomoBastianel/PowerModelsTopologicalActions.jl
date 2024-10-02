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

ipopt = JuMP.optimizer_with_attributes(Ipopt.Optimizer, "tol" => 1e-4, "print_level" => 0)
highs = JuMP.optimizer_with_attributes(HiGHS.Optimizer)
gurobi = JuMP.optimizer_with_attributes(Gurobi.Optimizer)
set_optimizer_attribute(gurobi, "MIPGap", 1e-6)
juniper = JuMP.optimizer_with_attributes(Juniper.Optimizer, "nl_solver" => ipopt, "mip_solver" => gurobi, "time_limit" => 36000)
mosek = JuMP.optimizer_with_attributes(Mosek.Optimizer)

#######################################################################################
## Input data ##
#######################################################################################

test_case = "case39_acdc.m"


#######################################################################################
## Parsing input data ##
#######################################################################################
s_dual = Dict("output" => Dict("branch_flows" => true,"duals" => true), "conv_losses_mp" => true)
s = Dict("output" => Dict("branch_flows" => true), "conv_losses_mp" => true)

data_file = joinpath(@__DIR__,"data_sources",test_case)
data_original = _PM.parse_file(data_file)
data = deepcopy(data_original)
_PMACDC.process_additional_data!(data)


for (l_id,l) in data["load"]
    if l_id != "4"
        l["pd"] = 0
        l["qd"] = 0
    else
        l["pd"] = l["pd"]*6
        l["qd"] = l["qd"]*6
    end
end


#######################################################################################
## Optimal Power Flow models ##
#######################################################################################
# OPF simulations
result_opf_ac = _PMACDC.run_acdcopf(data,ACPPowerModel,ipopt; setting = s)
result_opf_ac_lpac = _PMACDC.run_acdcopf(data,LPACCPowerModel,gurobi; setting = s)


#######################################################################################
## Read results ##
#######################################################################################

folder_results = "Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting"

data_screening_file = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting/case_588/Results_lpac_screening_dc_bs.json.json"

data_screening_file = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting/case_588/Results_lpac_screening_dc_bs.json"

result_file = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting/case_39/Results_lpac_screening_dc_bs.json"
result_original = JSON.parsefile(data_screening_file)

result_ac_bs_file = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting/case_39/Results_ac_bs_16_ac.json"
result_ac_bs_check_file = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting/case_39/Results_ac_bs_16_ac_check.json"

result_lpac_bs_file = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting/case_39/Results_ac_bs_16_lpac.json"
result_lpac_bs_check_file = "/Users/giacomobastianel/Library/CloudStorage/OneDrive-KULeuven/Papers/Topological_actions/Results/Busbar_splitting/case_39/Results_ac_bs_16_lpac_check.json"





# Selecting which busbars are split
splitted_bus_ac = 13
data_busbars_ac_split = deepcopy(data)

split_elements = _PMTP.elements_AC_busbar_split(data)
data_busbars_ac_split,  switches_couples_ac,  extremes_ZILs_ac  = _PMTP.AC_busbar_split_more_buses(data_busbars_ac_split,splitted_bus_ac)


ac_bs_ac_ref = deepcopy(data_busbars_ac_split)
ac_bs_lpac_ref = deepcopy(data_busbars_ac_split)

result_switches_AC_lpac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_lpac_ref,LPACCPowerModel,gurobi)
result_switches_AC_ac_ref  = _PMTP.run_acdcsw_AC_reformulation(ac_bs_ac_ref,ACPPowerModel,juniper)


try_ac_check = deepcopy(data_busbars_ac_split)
try_ac_check_lpac = deepcopy(data_busbars_ac_split) 

prepare_AC_feasibility_check(result_switches_AC_lpac_ref,data_busbars_ac_split,try_ac_check_lpac,switches_couples_ac,extremes_ZILs_ac)
#prepare_AC_feasibility_check(result_switches_AC_ac_ref,data_busbars_ac_split,try_ac_check_ac,switches_couples_ac,extremes_ZILs_ac)

result_opf_lpac_check = _PMACDC.run_acdcopf(try_ac_check_lpac,ACPPowerModel,ipopt; setting = s)












data_screening = JSON.parsefile(data_screening_file)

result_ac_bs = JSON.parsefile(result_ac_bs_file)
result_ac_bs_check = JSON.parsefile(result_ac_bs_check_file)

result_lpac_bs = JSON.parsefile(result_lpac_bs_file)
result_lpac_bs_check = JSON.parsefile(result_lpac_bs_check_file)

obj = []
for i in 1:10
    #print([i,results["lpac"][i]["objective"]],"\n")
    push!(obj,data_screening["lpac"]["$i"]["objective"])
end

# Selecting which busbars are split
splitted_bus_ac = 4
data_busbars_ac_split = deepcopy(data)

split_elements = _PMTP.elements_DC_busbar_split(data)
data_busbars_dc_split,  switches_couples_dc,  extremes_ZILs_dc  = _PMTP.DC_busbar_split_more_buses(data_busbars_dc_split,splitted_bus_dc)


dc_bs_ac_ref = deepcopy(data_busbars_dc_split)
dc_bs_lpac_ref = deepcopy(data_busbars_dc_split)

result_switches_DC_lpac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_lpac_ref,LPACCPowerModel,gurobi)
result_switches_DC_ac_ref  = _PMTP.run_acdcsw_DC_reformulation(dc_bs_ac_ref,ACPPowerModel,juniper)


try_dc_check = deepcopy(data_busbars_dc_split)
try_dc_check_lpac = deepcopy(data_busbars_dc_split) 


function prepare_DC_feasibility_check(result_dict, input_dict, input_ac_check, switch_couples, extremes_dict)
    orig_buses = length(input_dict["bus"]) # original bus length
    for (br_id,br) in result_dict["solution"]["dcswitch"]
        if haskey(br,"ZIL") && br["ZIL"] == true
            if result_dict["solution"]["dcswitch"][br_id]["status"] >= 0.9
                delete!(input_ac_check["bus"],"$(extremes_dict[br_id][2])")
            else
                delete!(input_ac_check["dcswitch"],br_id)
            end
        end
    end
    for l in eachindex(switch_couples)
        if result_dict["solution"]["dcswitch"]["$(switch_couples["$l"]["dcswitch_split"])"]["status"] >= 0.9
            aux =  deepcopy(input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["auxiliary"])
            orig = deepcopy(input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["original"])
            if aux == "convdc"
                input_ac_check["convdc"]["$(orig)"]["busdc_i"] = deepcopy(switch_couples[l]["busdc_split"])
            elseif aux == "branchdc"                
                if input_ac_check["branchdc"]["$(orig)"]["fbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["t_busdc"] == switch_couples[l]["busdc_split"]
                    input_ac_check["branchdc"]["$(orig)"]["fbusdc"] = deepcopy(switch_couples[l]["busdc_split"])
                elseif input_ac_check["branchdc"]["$(orig)"]["tbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["t_busdc"] == switch_couples[l]["busdc_split"]
                    input_ac_check["branchdc"]["$(orig)"]["tbusdc"] = deepcopy(switch_couples[l]["busdc_split"])
                end
            end
        elseif result_dict["solution"]["dcswitch"]["$(switch_couples["$l"]["dcswitch_split"])"]["status"] <= 0.1
            switch_t = deepcopy(input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]) 
            aux_t = switch_t["auxiliary"]
            orig_t = switch_t["original"]
            print([l,aux_t,orig_t],"\n")
            if result_dict["solution"]["dcswitch"]["$(switch_t["index"])"]["status"] <= 0.1
                delete!(input_ac_check["dcswitch"],"$(switch_t["index"])")
            elseif result_dict["solution"]["dcswitch"]["$(switch_t["index"])"]["status"] >= 0.9
                if aux_t == "convdc"
                    input_ac_check["convdc"]["$(orig_t)"]["busac_i"] = deepcopy(input_ac_check["dcswitch"]["$(switch_t["index"])"]["t_busdc"])
                    print([l,aux_t,orig_t,input_ac_check["convdc"]["$(orig_t)"]["busdc_i"]],"\n")
                elseif aux_t == "branchdc" 
                    if input_ac_check["branchdc"]["$(orig_t)"]["fbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["busdc_split"] == switch_couples[l]["busdc_split"]
                        input_ac_check["branchdc"]["$(orig_t)"]["fbusdc"] = deepcopy(input_ac_check["dcswitch"]["$(switch_t["index"])"]["t_busdc"])
                        print([l,aux_t,orig_t,input_ac_check["branchdc"]["$(orig_t)"]["fbusdc"]],"\n")
                    elseif input_ac_check["branchdc"]["$(orig_t)"]["tbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["t_sw"])"]["busdc_split"] == switch_couples[l]["busdc_split"]
                        input_ac_check["branchdc"]["$(orig_t)"]["tbusdc"] = deepcopy(input_ac_check["dcswitch"]["$(switch_t["index"])"]["t_busdc"])
                        print([l,aux_t,orig_t,input_ac_check["branchdc"]["$(orig_t)"]["tbusdc"]],"\n")
                    end
                end
            end

            switch_f = deepcopy(input_ac_check["dcswitch"]["$(switch_couples["$l"]["f_sw"])"]) 
            aux_f = switch_f["auxiliary"]
            orig_f = switch_f["original"]
            print([l,aux_f,orig_f],"\n")
            if result_dict["solution"]["dcswitch"]["$(switch_f["index"])"]["status"] <= 0.1
                delete!(input_ac_check["dcswitch"],"$(switch_t["index"])")
            else
                if aux_f == "convdc"
                    input_ac_check["convdc"]["$(orig_f)"]["busac_i"] = deepcopy(input_ac_check["dcswitch"]["$(switch_f["index"])"]["tbusdc"])
                    print([l,aux_t,orig_t,input_ac_check["convdc"]["$(orig_t)"]["busac_i"]],"\n")
                elseif aux_f == "branchdc"
                    if input_ac_check["branchdc"]["$(orig_f)"]["fbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["f_sw"])"]["busdc_split"] == switch_couples[l]["busdc_split"]
                        input_ac_check["branchdc"]["$(orig_f)"]["fbusdc"] = deepcopy(input_ac_check["dcswitch"]["$(switch_f["index"])"]["tbusdc"])
                        print([l,aux_t,orig_t,input_ac_check["branchdc"]["$(orig_t)"]["fbusdc"]],"\n")
                    elseif input_ac_check["branchdc"]["$(orig_f)"]["tbusdc"] > orig_buses && input_ac_check["dcswitch"]["$(switch_couples["$l"]["f_sw"])"]["busdc_split"] == switch_couples[l]["busdc_split"]
                        input_ac_check["branchdc"]["$(orig_f)"]["tbusdc"] = deepcopy(input_ac_check["dcswitch"]["$(switch_f["index"])"]["tbusdc"])
                        print([l,aux_t,orig_t,input_ac_check["branchdc"]["$(orig_t)"]["tbusdc"]],"\n")
                    end
                end
            end
        end
    end
end

prepare_DC_feasibility_check(result_switches_DC_lpac_ref,data_busbars_dc_split,try_dc_check_lpac,switches_couples_dc,extremes_ZILs_dc)

result_opf_lpac_check = _PMACDC.run_acdcopf(try_dc_check_lpac,ACPPowerModel,ipopt; setting = s)
