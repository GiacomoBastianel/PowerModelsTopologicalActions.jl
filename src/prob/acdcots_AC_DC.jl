export run_acdcots_AC_DC


## AC formulation of the AC/DC OTS with both AC and DC branches that can be switched ##

""
function run_acdcots_AC_DC(file::String, model_type::Type, solver; kwargs...)
    data = _PM.parse_file(file)
    _PMACDC.process_additional_data!(data)
    return run_acdcots_AC_DC(data, model_type, solver; ref_extensions = [_PMACDC.add_ref_dcgrid!], kwargs...)
end

""
function run_acdcots_AC_DC(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
    return _PM.solve_model(data, model_type, solver, build_acdcots_AC_DC; ref_extensions = [_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcots_AC_DC(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage_on_off(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)
    _PM.variable_branch_indicator(pm)

    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    variable_dc_branch_indicator(pm)
    variable_dc_conv_indicator(pm)
    variable_voltage_slack_ots(pm)

    _PM.objective_min_fuel_cost(pm)

    _PM.constraint_model_voltage_on_off(pm)
    _PMACDC.constraint_voltage_dc(pm)
    
    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PMACDC.constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from_on_off(pm, i)
        _PM.constraint_ohms_yt_to_on_off(pm, i)
        _PM.constraint_voltage_angle_difference_on_off(pm,i)
        _PM.constraint_thermal_limit_from_on_off(pm,i)
        _PM.constraint_thermal_limit_to_on_off(pm,i)
    end    
    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        constraint_ohms_ots_dc_branch(pm, i)
        constraint_branch_limit_on_off_dc_ots(pm,i)
    end
    for i in _PM.ids(pm, :convdc)
        constraint_converter_losses_dc_ots(pm, i)
        constraint_converter_current_ots(pm,i)
        constraint_conv_transformer_dc_ots(pm, i)
        constraint_conv_reactor_dc_ots(pm, i)
        _PMACDC.constraint_conv_filter(pm, i)
        constraint_converter_limit_on_off_dc_ots(pm,i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end

## AC formulation of the AC/DC OTS with only AC branches that can be switched, binary constraint for each branch linearised between 0 and 1 ##
function run_acdcots_AC_DC_lin(file::String, model_type::Type, solver; kwargs...)
    data = _PM.parse_file(file)
    _PMACDC.process_additional_data!(data)
    return run_acdcots_AC_DC_lin(data, model_type, solver; ref_extensions = [_PMACDC.add_ref_dcgrid!], kwargs...)
end

""
function run_acdcots_AC_DC_lin(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
    return _PM.solve_model(data, model_type, solver, build_acdcots_AC_DC_lin; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcots_AC_DC_lin(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage_on_off(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    _PM.variable_branch_indicator(pm, relax = true)

    variable_dc_branch_indicator(pm, relax = true)
    variable_dc_conv_indicator(pm, relax = true)
    variable_voltage_slack_ots(pm)


    _PM.objective_min_fuel_cost(pm)

    _PM.constraint_model_voltage_on_off(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PMACDC.constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from_on_off(pm, i)
        _PM.constraint_ohms_yt_to_on_off(pm, i)

        _PM.constraint_voltage_angle_difference_on_off(pm,i)

        _PM.constraint_thermal_limit_from_on_off(pm,i)
        _PM.constraint_thermal_limit_to_on_off(pm,i)
    end
    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        constraint_ohms_ots_dc_branch(pm, i)
        constraint_branch_limit_on_off_dc_ots(pm,i)
    end
    for i in _PM.ids(pm, :convdc)
        constraint_converter_losses_dc_ots(pm, i)
        constraint_converter_current_ots(pm,i)
        constraint_conv_transformer_dc_ots(pm, i)
        constraint_conv_reactor_dc_ots(pm, i)
        constraint_conv_filter_dc_ots(pm, i)
        constraint_converter_limit_on_off_dc_ots(pm,i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end


## AC formulation of the AC/DC OTS with only AC branches that can be switched, binary constraint for each branch linearised between 0 and 1 and constraint z(z-1) < err added ##
""
function run_acdcots_AC_DC_lin_constrained(file::String, model_type::Type, solver; kwargs...)
    data = _PM.parse_file(file)
    _PMACDC.process_additional_data!(data)
    return run_acdcots_AC_DC_linearised(data, model_type, solver; ref_extensions = [_PMACDC.add_ref_dcgrid!], kwargs...)
end

""
function run_acdcots_AC_DC_lin_constrained(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
    return _PM.solve_model(data, model_type, solver, build_acdcots_AC_DC_lin_constrained; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcots_AC_DC_lin_constrained(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage_on_off(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    variable_branch_indicator_linearised(pm)
    variable_dc_branch_indicator_linearised(pm)
    variable_dc_conv_indicator_linearised(pm)
    variable_voltage_slack_ots(pm)

    _PM.objective_min_fuel_cost(pm)

    _PM.constraint_model_voltage_on_off(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PMACDC.constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from_on_off(pm, i)
        _PM.constraint_ohms_yt_to_on_off(pm, i)

        _PM.constraint_voltage_angle_difference_on_off(pm,i)

        _PM.constraint_thermal_limit_from_on_off(pm,i)
        _PM.constraint_thermal_limit_to_on_off(pm,i)
        constraint_linearised_binary_variable(pm,i)
    end
    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        constraint_ohms_ots_dc_branch(pm, i)
        constraint_branch_limit_on_off_dc_ots(pm,i)
        constraint_linearised_binary_variable_DC_branch(pm,i)
    end
    for i in _PM.ids(pm, :convdc)
        constraint_converter_losses_dc_ots(pm, i)
        constraint_converter_current_ots(pm,i)
        constraint_conv_transformer_dc_ots(pm, i)
        constraint_conv_reactor_dc_ots(pm, i)
        constraint_conv_filter_dc_ots(pm, i)
        constraint_converter_limit_on_off_dc_ots(pm,i)
        constraint_linearised_binary_variable_DC_conv(pm,i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end

## AC formulation of the AC/DC OTS with only AC branches that can be switched, binary constraint for each branch linearised between 0 and 1 and constraint z(z-1) < err added ##
""
function run_acdcots_AC_DC_lin_constrained_sp(file::String, model_type::Type, solver; kwargs...)
    data = _PM.parse_file(file)
    _PMACDC.process_additional_data!(data)
    return run_acdcots_AC_DC_linearised_sp(data, model_type, solver; ref_extensions = [_PMACDC.add_ref_dcgrid!], kwargs...)
end

""
function run_acdcots_AC_DC_lin_constrained_sp(data::Dict{String,Any}, model_type::Type, solver; kwargs...)
    return _PM.solve_model(data, model_type, solver, build_acdcots_AC_DC_lin_constrained_sp; ref_extensions=[_PMACDC.add_ref_dcgrid!,_PM.ref_add_on_off_va_bounds!], kwargs...)
end

""
function build_acdcots_AC_DC_lin_constrained_sp(pm::_PM.AbstractPowerModel)
    _PM.variable_bus_voltage_on_off(pm)
    _PM.variable_gen_power(pm)
    _PM.variable_branch_power(pm)

    _PMACDC.variable_active_dcbranch_flow(pm)
    _PMACDC.variable_dcbranch_current(pm)
    _PMACDC.variable_dc_converter(pm)
    _PMACDC.variable_dcgrid_voltage_magnitude(pm)

    variable_branch_indicator_linearised_sp(pm)
    variable_dc_branch_indicator_linearised_sp(pm)
    variable_dc_conv_indicator_linearised_sp(pm)
    variable_voltage_slack_ots(pm)

    _PM.objective_min_fuel_cost(pm)

    _PM.constraint_model_voltage_on_off(pm)
    _PMACDC.constraint_voltage_dc(pm)

    for i in _PM.ids(pm, :ref_buses)
        _PM.constraint_theta_ref(pm, i)
    end

    for i in _PM.ids(pm, :bus)
        _PMACDC.constraint_power_balance_ac(pm, i)
    end

    for i in _PM.ids(pm, :branch)
        _PM.constraint_ohms_yt_from_on_off(pm, i)
        _PM.constraint_ohms_yt_to_on_off(pm, i)

        _PM.constraint_voltage_angle_difference_on_off(pm,i)

        _PM.constraint_thermal_limit_from_on_off(pm,i)
        _PM.constraint_thermal_limit_to_on_off(pm,i)
        constraint_linearised_binary_variable(pm,i)
    end
    for i in _PM.ids(pm, :busdc)
        _PMACDC.constraint_power_balance_dc(pm, i)
    end
    for i in _PM.ids(pm, :branchdc)
        constraint_ohms_ots_dc_branch(pm, i)
        constraint_branch_limit_on_off_dc_ots(pm,i)
        constraint_linearised_binary_variable_DC_branch(pm,i)
    end
    for i in _PM.ids(pm, :convdc)
        constraint_converter_losses_dc_ots(pm, i)
        constraint_converter_current_ots(pm,i)
        constraint_conv_transformer_dc_ots(pm, i)
        constraint_conv_reactor_dc_ots(pm, i)
        constraint_conv_filter_dc_ots(pm, i)
        constraint_converter_limit_on_off_dc_ots(pm,i)
        constraint_linearised_binary_variable_DC_conv(pm,i)
        if pm.ref[:it][:pm][:nw][_PM.nw_id_default][:convdc][i]["islcc"] == 1
            _PMACDC.constraint_conv_firing_angle(pm, i)
        end
    end
end