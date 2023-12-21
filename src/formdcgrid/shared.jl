######################################## OTS Constraints ###########################################
# The following is not needed here, use variable_dcgrid_voltage_magnitude

#function variable_dcgrid_voltage_magnitude_ne(pm::_PM.AbstractWModels; kwargs...)
#    variable_dcgrid_voltage_magnitude_sqr_ne(pm; kwargs...)
#    variable_dcgrid_voltage_magnitude_sqr_du(pm; kwargs...) # duplicated to cancel out existing dc voltages(W) from ohms constraint when z = 0
#end
function variable_dcgrid_voltage_magnitude(pm::_PM.AbstractWModels; kwargs...)
    variable_dcgrid_voltage_magnitude_sqr(pm; kwargs...)
end

#=
function variable_dcgrid_voltage_magnitude_sqr_ots_du(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true) 
    bi_bp = Dict([(i, (b["fbusdc"], b["tbusdc"])) for (i,b) in _PM.ref(pm, nw, :branchdc)])
    wdc_fr_ots = _PM.var(pm, nw)[:wdc_du_fr_ots] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_wdc_du_fr",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, bi_bp[i][1]), "Vdc",  1.0)^2,
    )
    wdc_to_ots = _PM.var(pm, nw)[:wdc_du_to_ots] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_wdc_du_to",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, bi_bp[i][1]), "Vdc",  1.0)^2,
    )
    #TODO [OLD FROM PMACDC] replace wdc_du_fr and wdc_du_to with wdc_fr and wdc_to make make it consistent with PM, there multiplication is defined by wr - real and wi- imag
    wdcr_frto_ots = _PM.var(pm, nw)[:wdcr_du] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :branchdc)], base_name="$(nw)_wdcr_du",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, bi_bp[i][1]), "Vdc",  1.0)^2,
    )

    if bounded
        for (i, branchdc) in _PM.ref(pm, nw, :branchdc)
            JuMP.set_lower_bound(wdc_fr_ots[i],  0)
            JuMP.set_upper_bound(wdc_fr_ots[i],  1.21)
            JuMP.set_lower_bound(wdc_to_ots[i],  0)
            JuMP.set_upper_bound(wdc_to_ots[i],  1.21)
            JuMP.set_lower_bound(wdcr_frto_ots[i],  0)
            JuMP.set_upper_bound(wdcr_frto_ots[i],  1.21)
        end
    end
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :wdc_du_fr_ots, _PM.ids(pm, nw, :branchdc), wdc_fr_ots)
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :wdc_du_to_ots, _PM.ids(pm, nw, :branchdc), wdc_to_ots)
    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :branchdc, :wdcr_du_ots, _PM.ids(pm, nw, :branchdc), wdcr_frto_ots)
end

function constraint_ohms_ots_dc_branch(pm::_PM.AbstractWRModels, n::Int, f_bus, t_bus, f_idx, t_idx, r, p)
    l = f_idx[1]
    p_dc_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    p_dc_to = _PM.var(pm, n, :p_dcgrid, t_idx)
    z = _PM.var(pm, n, :z_ots_dc, l)

    wdc_to = []
    wdc_fr = []

    wdc_to, wdc_fr = constraint_ohms_dc_branch_busvoltage_structure_W_ots(pm, n, f_bus, t_bus, wdc_to, wdc_fr)
    wdc_du_to = _PM.var(pm, n, :wdc_du_to_o, l)
    wdc_du_fr = _PM.var(pm, n, :wdc_du_fr_o, l)
    wdc_frto = _PM.var(pm, n, :wdcr, l)
    wdc_du_frto = _PM.var(pm, n, :wdcr_du, l)

    if r == 0
        JuMP.@constraint(pm.model, p_dc_fr + p_dc_to == 0)
    else
        g = 1 / r
        JuMP.@constraint(pm.model, p_dc_fr == p * g *  (wdc_du_fr - wdc_du_frto))
        JuMP.@constraint(pm.model, p_dc_to == p * g *  (wdc_du_to - wdc_du_frto))
        JuMP.@constraint(pm.model, wdc_du_to <= wdc_to - JuMP.lower_bound(wdc_to)*(1-z))
        JuMP.@constraint(pm.model, wdc_du_to >= wdc_to - JuMP.upper_bound(wdc_to)*(1-z))
        JuMP.@constraint(pm.model, wdc_du_fr <= wdc_fr - JuMP.lower_bound(wdc_fr)*(1-z))
        JuMP.@constraint(pm.model, wdc_du_fr >= wdc_fr - JuMP.upper_bound(wdc_fr)*(1-z))
        JuMP.@constraint(pm.model, wdc_du_frto <= wdc_frto - JuMP.lower_bound(wdc_frto)*(1-z))
        JuMP.@constraint(pm.model, wdc_du_frto >= wdc_frto - JuMP.upper_bound(wdc_frto)*(1-z))

        JuMP.@constraint(pm.model, wdc_du_to <= z* JuMP.upper_bound(wdc_to))
        JuMP.@constraint(pm.model, wdc_du_to >= z* JuMP.lower_bound(wdc_to))
        JuMP.@constraint(pm.model, wdc_du_fr <= z* JuMP.upper_bound(wdc_fr))
        JuMP.@constraint(pm.model, wdc_du_fr >= z* JuMP.lower_bound(wdc_fr))
        JuMP.@constraint(pm.model, wdc_du_frto <= z* JuMP.upper_bound(wdc_frto))
        JuMP.@constraint(pm.model, wdc_du_frto >= z* JuMP.lower_bound(wdc_frto))
    end
end
=#

function constraint_ohms_ots_dc_branch(pm::_PM.AbstractWRModels, n::Int,  f_bus, t_bus, f_idx, t_idx, r, p)
    p_dc_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    p_dc_to = _PM.var(pm, n, :p_dcgrid, t_idx)
    wdc_fr = _PM.var(pm, n, :wdc, f_bus)
    wdc_to = _PM.var(pm, n, :wdc, t_bus)
    wdc_frto = _PM.var(pm, n, :wdcr, (f_bus, t_bus))
    l = f_idx[1]
    z = _PM.var(pm, n, :z_ots_dc, l)
    

    if r == 0
        JuMP.@constraint(pm.model, p_dc_fr + p_dc_to == 0)
    else
        g = 1 / r
        JuMP.@constraint(pm.model, p_dc_fr == p * g * z[l] * (wdc_fr - wdc_frto))
        JuMP.@constraint(pm.model, p_dc_to == p * g * z[l] * (wdc_to - wdc_frto))
    end
end

function constraint_dc_switch_voltage_on_off(pm::_PM.AbstractWRModels, n::Int, i, f_busdc, t_busdc)
    wdc_fr = _PM.var(pm, n, :wdc, f_busdc)
    wdc_to = _PM.var(pm, n, :wdc, t_busdc)
    z = _PM.var(pm, n, :z_dcswitch, i)

    JuMP.@constraint(pm.model, z*wdc_fr == z*wdc_to)
end

function constraint_switch_voltage_on_off(pm::_PM.AbstractWRModels, n::Int, i, f_bus, t_bus)
    #vm_fr = _PM.var(pm, n, :vm, f_bus)
    #vm_to = _PM.var(pm, n, :vm, t_bus)
    w_fr = _PM.var(pm, n, :w, f_bus)
    w_to = _PM.var(pm, n, :w, t_bus)
    z = _PM.var(pm, n, :z_switch, i)

    JuMP.@constraint(pm.model, z*w_fr == z*w_to)
    #JuMP.@constraint(pm.model, z*va_fr == z*va_to)
end


function constraint_power_balance_ac_switch(pm::_PM.AbstractWRModels, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_gens, bus_convs_ac, bus_loads, bus_shunts, pd, qd, gs, bs)
    w = _PM.var(pm, n, :w, i)
    p = _PM.var(pm, n,  :p)
    q = _PM.var(pm, n,  :q)
    pg = _PM.var(pm, n,  :pg)
    qg = _PM.var(pm, n,  :qg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)
    qconv_grid_ac = _PM.var(pm, n,  :qconv_tf_fr)
    psw  = _PM.var(pm, n, :psw)
    qsw  = _PM.var(pm, n, :qsw)

    cstr_p = JuMP.@constraint(pm.model, sum(p[a] for a in bus_arcs) + sum(pconv_grid_ac[c] for c in bus_convs_ac) + sum(psw[sw] for sw in bus_arcs_sw) == sum(pg[g] for g in bus_gens)  - sum(pd[d] for d in bus_loads) - sum(gs[s] for s in bus_shunts)*w)
    cstr_q = JuMP.@constraint(pm.model, sum(q[a] for a in bus_arcs) + sum(qconv_grid_ac[c] for c in bus_convs_ac) + sum(qsw[sw] for sw in bus_arcs_sw) == sum(qg[g] for g in bus_gens)  - sum(qd[d] for d in bus_loads) + sum(bs[s] for s in bus_shunts)*w)

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
        _PM.sol(pm, n, :bus, i)[:lam_kcl_i] = cstr_q
    end
end




















#=

######################################## BUSBAR SPLITTING CONSTRAINTS #####################################
## ACDC switch
function constraint_power_balance_ac_switch(pm::_PM.AbstractWModels, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_gens, bus_convs_ac, bus_loads, bus_shunts, pd, qd, gs, bs)
    w = _PM.var(pm, n, :w, i)
    p = _PM.var(pm, n, :p)
    q = _PM.var(pm, n, :q)
    pg = _PM.var(pm, n, :pg)
    qg = _PM.var(pm, n, :qg)
    pconv_grid_ac = _PM.var(pm, n, :pconv_tf_fr)
    qconv_grid_ac = _PM.var(pm, n, :qconv_tf_fr)
    psw  = _PM.var(pm, n, :psw)
    qsw  = _PM.var(pm, n, :qsw)

    cstr_p = JuMP.@NLconstraint(pm.model, sum(p[a] for a in bus_arcs) + sum(pconv_grid_ac[c] for c in bus_convs_ac) + sum(psw[sw] for sw in bus_arcs_sw) == sum(pg[g] for g in bus_gens)  - sum(pd[d] for d in bus_loads) - sum(gs[s] for s in bus_shunts)*w)
    cstr_q = JuMP.@NLconstraint(pm.model, sum(q[a] for a in bus_arcs) + sum(qconv_grid_ac[c] for c in bus_convs_ac) + sum(qsw[sw] for sw in bus_arcs_sw) == sum(qg[g] for g in bus_gens)  - sum(qd[d] for d in bus_loads) + sum(bs[s] for s in bus_shunts)*w)

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
        _PM.sol(pm, n, :bus, i)[:lam_kcl_i] = cstr_q
    end
end

function constraint_switch_voltage_on_off(pm::_PM.AbstractWRModels, n::Int, i, f_bus, t_bus, vad_min, vad_max)
    #vm_fr = _PM.var(pm, n, :vm_fr)
    #vm_to = _PM.var(pm, n, :vm_to)

    w_fr = _PM.var(pm, n, :w, f_bus)
    w_to = _PM.var(pm, n, :w, t_bus)

    z = _PM.var(pm, n, :z_switch, i)

    JuMP.@constraint(pm.model, z*w_fr == z*w_to)
end

function constraint_dc_switch_voltage_on_off(pm::_PM.AbstractWRModels, n::Int, i, f_busdc, t_busdc)
    w_fr = _PM.var(pm, n, :wdc, f_busdc)
    w_to = _PM.var(pm, n, :wdc, t_busdc)
    z = _PM.var(pm, n, :z_dcswitch, i)

    JuMP.@constraint(pm.model, z*w_fr == z*w_to)
end
=#