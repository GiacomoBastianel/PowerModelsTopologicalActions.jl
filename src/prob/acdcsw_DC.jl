export run_acdcsw_DC


# DC Busbar splitting for AC/DC grid
"ACDC opf with controllable switches in DC busbar splitting configuration for AC/DC grids"
function run_acdcsw_DC(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_DC; ref_extensions=[add_ref_dcgrid_dcswitch!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcsw_DC(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)

    #_PM.variable_switch_indicator(pm)
    #_PM.variable_switch_power(pm)

    variable_dc_switch_indicator(pm)
    variable_dc_switch_power(pm)


    _PM.variable_branch_power(pm)
    _PM.variable_dcline_power(pm)

    _PM.objective_min_fuel_and_flow_cost(pm)

    _PM.constraint_model_voltage(pm)

    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)
    _PMACDC.constraint_voltage_dc(pm)


    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PM.constraint_power_balance(pm, i)
    end

    #for i in _PM.ids(pm, :switch)
    #    _PM.constraint_switch_on_off(pm, i)
    #    _PM.constraint_switch_thermal_limit(pm, i)
    #end

    for i in _PM.ids(pm, :dcswitch)
        constraint_dc_switch_on_off(pm, i)
        constraint_dc_switch_thermal_limit(pm, i)
    end

    #for i in _PM.ids(pm, :switch_couples)
    #    constraint_exclusivity_switch(pm, i)
    #end

    for i in _PM.ids(pm, :dc_switch_couples)
        constraint_exclusivity_dc_switch(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)

        _PM.constraint_voltage_angle_difference(pm, i)

        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end

    for i in _PM.ids(pm, :dcline)
        _PM.constraint_dcline_power_losses(pm, i)
    end

    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        _PMACDC.constraint_ohms_dc_branch(pm, i)
    end
    for i in _PM.ids(pm, :convdc)
        _PMACDC.constraint_converter_losses(pm, i)
        _PMACDC.constraint_converter_current(pm, i)
        _PMACDC.constraint_conv_transformer(pm, i)
        _PMACDC.constraint_conv_reactor(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end




#=
# AC Busbar splitting for AC/DC grid
"ACDC opf with controllable switches in DC busbar splitting configuration for AC/DC grids"
function _solve_oswpf_DC_busbar_splitting_AC_DC(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, _build_oswpf_DC_busbar_splitting_AC_DC; ref_extensions=[add_ref_dcgrid_!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function _build_oswpf_DC_busbar_splitting_AC_DC(pm::_PM.AbstractPowerModel)
    print("Task_1","\n")
    _PM.variable_bus_voltage(pm)
    print("Task_2","\n")
    _PM.variable_gen_power(pm)
    print("Task_3","\n")
    variable_dc_switch_indicator(pm)
    print("Task_4","\n")
    variable_dc_switch_power(pm)
    print("Task_5","\n")
    _PM.variable_branch_power(pm)
    print("Task_6","\n")
    _PM.variable_dcline_power(pm)
    print("Task_7","\n")
    _PM.objective_min_fuel_and_flow_cost(pm)
    print("Task_8","\n")
    _PM.constraint_model_voltage(pm)
    print("Task_9","\n")
    variable_active_dcbranch_flow(pm)
    print("Task_10","\n")
    variable_dcbranch_current(pm)
    print("Task_11","\n")
    variable_dc_converter(pm)
    print("Task_12","\n")
    variable_dcgrid_voltage_magnitude(pm)
    print("Task_13","\n")
    constraint_voltage_dc(pm)
    print("Task_14","\n")

    for i in _PM.ids(pm, :ref_buses)
        print(i,"\n")
        _PM.constraint_theta_ref(pm, i)
    end
    print("Task_15","\n")
    for i in _PM.ids(pm, :bus)
        print(i,"\n")
        _PM.constraint_power_balance(pm, i)
    end
    print("Task_16","\n")
    for i in _PM.ids(pm, :dcswitch)
        print(i,"\n")
        constraint_dc_switch_on_off(pm, i)
        constraint_dc_switch_thermal_limit(pm, i)
    end
    print("Task_17","\n")
    for i in _PM.ids(pm, :dc_switch_couples)
        print(i,"\n")
        constraint_exclusivity_dc_switch(pm, i)
    end
    print("Task_18","\n")
    for i in _PM.ids(pm, :branch)
        print(i,"\n")
        _PM.constraint_ohms_yt_from(pm, i)
        _PM.constraint_ohms_yt_to(pm, i)

        _PM.constraint_voltage_angle_difference(pm, i)

        _PM.constraint_thermal_limit_from(pm, i)
        _PM.constraint_thermal_limit_to(pm, i)
    end
    print("Task_19","\n")
    for i in _PM.ids(pm, :dcline)
        print(i,"\n")
        _PM.constraint_dcline_power_losses(pm, i)
    end

    for i in _PM.ids(pm, :busdc)
        print(i,"\n")
        constraint_power_balance_dc_switch(pm,i)
    end
    for i in _PM.ids(pm, :branchdc)
        print(i,"\n")
        constraint_ohms_dc_branch(pm, i)
    end
    for i in _PM.ids(pm, :convdc)
        print(i,"\n")
        _PMACDC.constraint_converter_losses(pm, i)
        _PMACDC.constraint_converter_current(pm, i)
        _PMACDC.constraint_conv_transformer(pm, i)
        _PMACDC.constraint_conv_reactor(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            constraint_conv_firing_angle(pm, i)
        end
    end
end
=#