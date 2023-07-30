function AC_busbar_split(data,bus_to_be_split)

    # Adding a new key to indicate which bus can be split, choosing it for now -> to be updated
    for i in keys(data["bus"])
        if parse(Int64,i) != bus_to_be_split
            data["bus"]["$i"]["split"] = false
        end
    end
    data["bus"]["$bus_to_be_split"]["split"] = true
    
    # Adding a bus next to the split
    n_buses_original = length(data["bus"])
    for (b_id,b) in data["bus"] 
        n_buses = length(data["bus"])
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original
            b["bus_split"] = deepcopy(b["index"])
            added_bus = n_buses + 1
            data["bus"]["$added_bus"] = deepcopy(b)
            data["bus"]["$added_bus"]["bus_i"] = added_bus 
            data["bus"]["$added_bus"]["source_id"][2] = added_bus 
            data["bus"]["$added_bus"]["index"] = added_bus
            data["bus"]["$added_bus"]["split"] = false
        end
    end 
    
    # Creating a dictionary with the split buses
    extremes_ZIL = Dict{String,Any}()
    for (b_id,b) in data["bus"]
        if b["split"] == true && !haskey(extremes_ZIL,b["bus_split"])
            split_bus = b["bus_split"]
            extremes_ZIL["$split_bus"] = []
        end
        if haskey(b,"bus_split")
            for i in eachindex(extremes_ZIL)
                if b["bus_split"] == parse(Int64,i)
                    push!(extremes_ZIL[i],b["index"])
                end
            end
        end
    end
    
    
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses
    switch_id = 0
    for i in eachindex(extremes_ZIL)
        switch_id += 1
        data["switch"]["$switch_id"] = Dict{String,Any}()
        #data["switch"]["$switch_id"] = deepcopy(data_sw["switch"]["1"])
        data["switch"]["$switch_id"]["f_bus"] = deepcopy(extremes_ZIL[i][1]) 
        data["switch"]["$switch_id"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
        data["switch"]["$switch_id"]["index"] = switch_id
        data["switch"]["$switch_id"]["psw"] = 10.0
        data["switch"]["$switch_id"]["qsw"] = 10.0
        data["switch"]["$switch_id"]["thermal_rating"] = 10.0
        data["switch"]["$switch_id"]["state"] = 1
        data["switch"]["$switch_id"]["status"] = 1
        data["switch"]["$switch_id"]["source_id"] = []
        push!(data["switch"]["$switch_id"]["source_id"],"switch")
        push!(data["switch"]["$switch_id"]["source_id"],switch_id)
    end
    
    # Add a bus for each grid element
    # Gen
    for (g_id,g) in data["gen"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL)
            if g["gen_bus"] == parse(Int64,i)
                added_gen_bus = n_buses + 1
                data["bus"]["$added_gen_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_gen_bus"]["bus_type"] = 1
                data["bus"]["$added_gen_bus"]["bus_i"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["auxiliary_bus"] = true
                data["bus"]["$added_gen_bus"]["original"] = parse(Int64,g_id)  
                data["bus"]["$added_gen_bus"]["auxiliary"] = "gen"
                data["bus"]["$added_gen_bus"]["split"] = false
                g["gen_bus"] = added_gen_bus
            end
        end
    end 
    
    # Load
    for (l_id,l) in data["load"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL)
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["bus"]["$added_load_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_load_bus"]["bus_type"] = 1
                data["bus"]["$added_load_bus"]["bus_i"] = added_load_bus 
                data["bus"]["$added_load_bus"]["index"] = added_load_bus 
                data["bus"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["bus"]["$added_load_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_load_bus"]["original"] = parse(Int64,l_id)  
                data["bus"]["$added_load_bus"]["auxiliary"] = "load"
                data["bus"]["$added_load_bus"]["bus_split"] = parse(Int64,i) 
                data["bus"]["$added_load_bus"]["split"] = false
                l["load_bus"] = added_load_bus
            end
        end
    end 
    
    # Branch
    for (br_id,br) in data["branch"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL)
            if br["f_bus"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["original"] = parse(Int64,br_id)
                data["bus"]["$added_branch_bus"]["bus_split"] = parse(Int64,i) 
                if haskey(data["bus"]["$added_branch_bus"],"split")
                    delete!(data["bus"]["$added_branch_bus"],"split")
                end
                br["f_bus"] = added_branch_bus
            elseif br["t_bus"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["bus"]["$added_branch_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_branch_bus"]["bus_type"] = 1
                data["bus"]["$added_branch_bus"]["bus_i"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["bus"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_branch_bus"]["original"] = parse(Int64,br_id) 
                data["bus"]["$added_branch_bus"]["auxiliary"] = "branch"
                data["bus"]["$added_branch_bus"]["bus_split"] = parse(Int64,i) 
                data["bus"]["$added_branch_bus"]["split"] = false
                br["t_bus"] = added_branch_bus
            end
        end
    end

    # Converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL)
            if cv["busac_i"] == parse(Int64,i)
                added_conv_bus = n_buses + 1
                data["bus"]["$added_conv_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_conv_bus"]["bus_type"] = 1
                data["bus"]["$added_conv_bus"]["bus_i"] = added_conv_bus 
                data["bus"]["$added_conv_bus"]["index"] = added_conv_bus 
                data["bus"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                data["bus"]["$added_conv_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_conv_bus"]["original"] = parse(Int64,cv_id) 
                data["bus"]["$added_conv_bus"]["auxiliary"] = "convdc"
                data["bus"]["$added_conv_bus"]["bus_split"] = parse(Int64,i) 
                data["bus"]["$added_conv_bus"]["split"] = false
         
                cv["busac_i"] = added_conv_bus
            end
        end
        end
    end
    
    # Linking the auxiliary buses to both split buses with switches
    for (b_id,b) in data["bus"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["split"] == true
            for i in eachindex(extremes_ZIL)
                number_switches = length(data["switch"])
                added_switch = number_switches + 1
                data["switch"]["$added_switch"] = deepcopy(data["switch"]["1"])
                data["switch"]["$added_switch"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                data["switch"]["$added_switch"]["t_bus"] = deepcopy(extremes_ZIL[i][1])
                data["switch"]["$added_switch"]["index"] = added_switch 
                data["switch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
                data["switch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                data["switch"]["$added_switch"]["original"] = deepcopy(b["original"]) 
                data["switch"]["$added_switch"]["index"] = added_switch 
            end
        end
        # Connecting to the second bus
        if haskey(b,"auxiliary_bus") #&& b["split"] == false && b["auxiliary"] == "gen"
            for i in eachindex(extremes_ZIL)
                number_switches = length(data["switch"])
                added_switch = number_switches + 1
                data["switch"]["$added_switch"] = deepcopy(data["switch"]["1"])
                data["switch"]["$added_switch"]["f_bus"] = deepcopy(parse(Int64,b_id)) 
                data["switch"]["$added_switch"]["t_bus"] = deepcopy(extremes_ZIL[i][2])
                data["switch"]["$added_switch"]["index"] = added_switch
                data["switch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                data["switch"]["$added_switch"]["original"] = deepcopy(b["original"])  
                data["switch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
            end
        end
    end

    switch_couples = compute_couples_of_switches(data)
    data["switch_couples"] = Dict{String,Any}()
    data["switch_couples"] = deepcopy(switch_couples)

    return data, switch_couples
end

function compute_couples_of_switches(data)
    switch_couples = Dict{String,Any}()
    for (sw_id,sw) in data["switch"]
        for l in keys(data["switch"])
            if (haskey(sw, "auxiliary") && haskey(data["switch"][l], "auxiliary")) && (sw["original"] == data["switch"][l]["original"]) && (sw["index"] != data["switch"][l]["index"])
                switch_couples["$sw_id"] = Dict{String,Any}()
                switch_couples["$sw_id"]["f_sw"] = sw["index"]
                switch_couples["$sw_id"]["t_sw"] = data["switch"][l]["index"]
            end
        end
    end
    return switch_couples
end


function compute_couples_of_dcswitches(data)
    switch_couples = Dict{String,Any}()
    for (sw_id,sw) in data["dcswitch"]
        for l in keys(data["dcswitch"])
            if (haskey(sw, "auxiliary") && haskey(data["dcswitch"][l], "auxiliary")) && (sw["original"] == data["dcswitch"][l]["original"]) && (sw["index"] != data["dcswitch"][l]["index"])
                switch_couples["$sw_id"] = Dict{String,Any}()
                switch_couples["$sw_id"]["f_sw"] = sw["index"]
                switch_couples["$sw_id"]["t_sw"] = data["dcswitch"][l]["index"]
            end
        end
    end
    return switch_couples
end

function DC_busbar_split(data,bus_to_be_split)

    # Adding a new key to indicate which bus can be split, choosing it for now -> to be updated
    for i in keys(data["busdc"])
        if parse(Int64,i) != bus_to_be_split
            data["busdc"]["$i"]["split"] = false
        end
    end
    data["busdc"]["$bus_to_be_split"]["split"] = true
    
    # Adding a bus next to the split
    n_buses_original = length(data["busdc"])
    for (b_id,b) in data["busdc"] 
        n_buses = length(data["busdc"])
        if b["split"] == true && parse(Int64,b_id) <= n_buses_original
            b["busdc_split"] = deepcopy(b["index"])
            added_bus = n_buses + 1
            data["busdc"]["$added_bus"] = deepcopy(b)
            data["busdc"]["$added_bus"]["index"] = added_bus 
            data["busdc"]["$added_bus"]["source_id"][2] = added_bus 
            data["busdc"]["$added_bus"]["index"] = added_bus
            data["busdc"]["$added_bus"]["split"] = false
        end
    end 
    


    # Creating a dictionary with the split buses
    extremes_ZIL_dc = Dict{String,Any}()
    for (b_id,b) in data["busdc"]
        if b["split"] == true && !haskey(extremes_ZIL_dc,b["busdc_split"])
            split_bus = b["busdc_split"]
            extremes_ZIL_dc["$split_bus"] = []
        end
        if haskey(b,"busdc_split")
            for i in eachindex(extremes_ZIL_dc)
                if b["busdc_split"] == parse(Int64,i)
                    push!(extremes_ZIL_dc[i],b["index"])
                end
            end
        end
    end
    
    data["dcswitch"] = Dict{String,Any}()
    # Adding the Zero Impedance Line (ZIL) through a switch between the split buses
    switch_id = 0
    for i in eachindex(extremes_ZIL_dc)
        switch_id += 1
        data["dcswitch"]["$switch_id"] = Dict{String,Any}()
        #data["switch"]["$switch_id"] = deepcopy(data_sw["switch"]["1"])
        data["dcswitch"]["$switch_id"]["f_busdc"] = deepcopy(extremes_ZIL_dc[i][1]) 
        data["dcswitch"]["$switch_id"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][2])
        data["dcswitch"]["$switch_id"]["index"] = switch_id
        data["dcswitch"]["$switch_id"]["psw"] = 10.0
        data["dcswitch"]["$switch_id"]["thermal_rating"] = 10.0
        data["dcswitch"]["$switch_id"]["state"] = 1
        data["dcswitch"]["$switch_id"]["status"] = 1
        data["dcswitch"]["$switch_id"]["source_id"] = []
        push!(data["dcswitch"]["$switch_id"]["source_id"],"dcswitch")
        push!(data["dcswitch"]["$switch_id"]["source_id"],switch_id)
    end

    
    # Add a busdc for each grid element
    # Gen_dc
    for (g_id,g) in data["gen"] 
        n_buses = length(data["busdc"])
        for i in eachindex(extremes_ZIL_dc)
            if g["gen_bus"] == parse(Int64,i)
                added_gen_bus = n_buses + 1
                data["bus"]["$added_gen_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_gen_bus"]["bus_type"] = 1
                data["bus"]["$added_gen_bus"]["bus_i"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["index"] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["source_id"][2] = added_gen_bus 
                data["bus"]["$added_gen_bus"]["auxiliary_bus"] = true
                data["bus"]["$added_gen_bus"]["original"] = parse(Int64,g_id)  
                data["bus"]["$added_gen_bus"]["auxiliary"] = "gen"
                data["bus"]["$added_gen_bus"]["split"] = false
                g["gen_bus"] = added_gen_bus
            end
        end
    end 
    
    # Load_dc
    for (l_id,l) in data["load"] 
        n_buses = length(data["bus"])
        for i in eachindex(extremes_ZIL_dc)
            if l["load_bus"] == parse(Int64,i)
                added_load_bus = n_buses + 1
                data["bus"]["$added_load_bus"] = deepcopy(data["bus"]["1"])
                data["bus"]["$added_load_bus"]["bus_type"] = 1
                data["bus"]["$added_load_bus"]["bus_i"] = added_load_bus 
                data["bus"]["$added_load_bus"]["index"] = added_load_bus 
                data["bus"]["$added_load_bus"]["source_id"][2] = added_load_bus 
                data["bus"]["$added_load_bus"]["auxiliary_bus"] = true 
                data["bus"]["$added_load_bus"]["original"] = parse(Int64,l_id)  
                data["bus"]["$added_load_bus"]["auxiliary"] = "load"
                data["bus"]["$added_load_bus"]["bus_split"] = parse(Int64,i) 
                data["bus"]["$added_load_bus"]["split"] = false
                l["load_bus"] = added_load_bus
            end
        end
    end 
    
    # Branch dc
    for (br_id,br) in data["branchdc"] 
        n_buses = length(data["busdc"])
        for i in eachindex(extremes_ZIL_dc)
            if br["fbusdc"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["busdc"]["$added_branch_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_branch_bus"]["busdc_i"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_branch_bus"]["auxiliary"] = "branchdc"
                data["busdc"]["$added_branch_bus"]["original"] = parse(Int64,br_id)
                data["busdc"]["$added_branch_bus"]["busdc_split"] = parse(Int64,i) 
                if haskey(data["busdc"]["$added_branch_bus"],"split")
                    delete!(data["busdc"]["$added_branch_bus"],"split")
                end
                br["fbusdc"] = added_branch_bus
            elseif br["tbusdc"] == parse(Int64,i) && !haskey(br,"ZIL") 
                added_branch_bus = n_buses + 1
                data["busdc"]["$added_branch_bus"] = deepcopy(data["bus"]["1"])
                data["busdc"]["$added_branch_bus"]["busdc_i"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["index"] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["source_id"][2] = added_branch_bus 
                data["busdc"]["$added_branch_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_branch_bus"]["original"] = parse(Int64,br_id) 
                data["busdc"]["$added_branch_bus"]["auxiliary"] = "branchdc"
                data["busdc"]["$added_branch_bus"]["busdc_split"] = parse(Int64,i) 
                data["busdc"]["$added_branch_bus"]["split"] = false
                br["tbusdc"] = added_branch_bus
            end
        end
    end

    # Converters
    if haskey(data,"convdc")
        for (cv_id,cv) in data["convdc"] 
        n_buses = length(data["busdc"])
        for i in eachindex(extremes_ZIL_dc)
            if cv["busdc_i"] == parse(Int64,i)
                added_conv_bus = n_buses + 1
                data["busdc"]["$added_conv_bus"] = deepcopy(data["busdc"]["1"])
                data["busdc"]["$added_conv_bus"]["bus_type"] = 1
                data["busdc"]["$added_conv_bus"]["busdc_i"] = added_conv_bus 
                data["busdc"]["$added_conv_bus"]["index"] = added_conv_bus 
                data["busdc"]["$added_conv_bus"]["source_id"][2] = added_conv_bus 
                data["busdc"]["$added_conv_bus"]["auxiliary_bus"] = true 
                data["busdc"]["$added_conv_bus"]["original"] = parse(Int64,cv_id) 
                data["busdc"]["$added_conv_bus"]["auxiliary"] = "convdc"
                data["busdc"]["$added_conv_bus"]["busdc_split"] = parse(Int64,i) 
                data["busdc"]["$added_conv_bus"]["split"] = false
                cv["busdc_i"] = added_conv_bus
            end
        end
        end
    end
    
    # Linking the auxiliary buses to both split buses with switches
    for (b_id,b) in data["busdc"]
        # Connecting to the first bus
        if haskey(b,"auxiliary_bus") #&& b["split"] == true
            for i in eachindex(extremes_ZIL_dc)
                number_switches = length(data["dcswitch"])
                added_switch = number_switches + 1
                data["dcswitch"]["$added_switch"] = deepcopy(data["dcswitch"]["1"])
                data["dcswitch"]["$added_switch"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                data["dcswitch"]["$added_switch"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][1])
                data["dcswitch"]["$added_switch"]["index"] = added_switch 
                data["dcswitch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
                data["dcswitch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                data["dcswitch"]["$added_switch"]["original"] = deepcopy(b["original"]) 
                data["dcswitch"]["$added_switch"]["index"] = added_switch 
            end
        end
        # Connecting to the second bus
        if haskey(b,"auxiliary_bus") #&& b["split"] == false && b["auxiliary"] == "gen"
            for i in eachindex(extremes_ZIL_dc)
                number_switches = length(data["dcswitch"])
                added_switch = number_switches + 1
                data["dcswitch"]["$added_switch"] = deepcopy(data["dcswitch"]["1"])
                data["dcswitch"]["$added_switch"]["f_busdc"] = deepcopy(parse(Int64,b_id)) 
                data["dcswitch"]["$added_switch"]["t_busdc"] = deepcopy(extremes_ZIL_dc[i][2])
                data["dcswitch"]["$added_switch"]["index"] = added_switch
                data["dcswitch"]["$added_switch"]["auxiliary"] = deepcopy(b["auxiliary"]) 
                data["dcswitch"]["$added_switch"]["original"] = deepcopy(b["original"])  
                data["dcswitch"]["$added_switch"]["source_id"][2] = deepcopy(added_switch)
            end
        end
    end

    dcswitch_couples = compute_couples_of_switches(data)
    data["dcswitch_couples"] = Dict{String,Any}()
    data["dcswitch_couples"] = deepcopy(dcswitch_couples)

    return data, dcswitch_couples
end
