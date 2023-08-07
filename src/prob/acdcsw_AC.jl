export run_acdcsw_AC


# AC Busbar splitting for AC/DC grid
"ACDC opf with controllable switches in AC busbar splitting configuration for AC/DC grids"
function run_acdcsw_AC(file, model_constructor, optimizer; kwargs...)
    return _PM.solve_model(file, model_constructor, optimizer, build_acdcsw_AC; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcsw_AC(pm::_PM.AbstractPowerModel)
    # AC grid
    _PM.variable_bus_voltage(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    _PM.variable_switch_indicator(pm)
    _PM.variable_switch_power(pm)

    # DC grid
    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    # Objective function
    _PM.objective_min_fuel_and_flow_cost(pm)

    # Constraints
    _PM.constraint_model_voltage(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        constraint_power_balance_ac_switch(pm, i)
    end

    for i in _PM.ids(pm, :switch)
        _PM.constraint_switch_on_off(pm, i)
        _PM.constraint_switch_thermal_limit(pm, i)
    end

    for i in _PM.ids(pm, :switch_couples)
        constraint_exclusivity_switch(pm, i)
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
