# DCP formulation
#=
function constraint_ohms_ots_dc_branch(pm::_PM.AbstractDCPModel, n::Int,  f_bus, t_bus, f_idx, t_idx, r, p)
    i, f_bus, t_bus = f_idx
    p_dc_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    p_dc_to = _PM.var(pm, n, :p_dcgrid, t_idx)
    z = _PM.var(pm, n, :z_ots_dc, i)

    JuMP.@constraint(pm.model, z*p_dc_fr + z*p_dc_to == 0)
end

function constraint_converter_losses_dc_ots(pm::_PM.AbstractDCPModel, n::Int,  i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    v = 1 #pu, assumption to approximate current
    cm_conv_ac = pconv_ac/v # can actually be negative, not a very nice model...
    z_DC = _PM.var(pm, n, :z_conv_dc, i)
    if pm.setting["conv_losses_mp"] == true
        JuMP.@constraint(pm.model, z_DC*pconv_ac + z_DC*pconv_dc == a + z_DC*b*cm_conv_ac)
    else
        JuMP.@constraint(pm.model, z_DC*pconv_ac + z_DC*pconv_dc >=   a + z_DC*b*cm_conv_ac)
        JuMP.@constraint(pm.model, z_DC*pconv_ac + z_DC*pconv_dc >=  (a - z_DC*b*cm_conv_ac))
        JuMP.@constraint(pm.model, z_DC*pconv_ac + z_DC*pconv_dc <= plmax)
    end
end

function constraint_conv_reactor_dc_ots(pm::_PM.AbstractDCPModel, n::Int, i::Int, rc, xc, reactor)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    ppr_to = - pconv_ac
    ppr_fr = _PM.var(pm, n,  :pconv_pr_fr, i)

    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    vaf = _PM.var(pm, n, :vaf, i)
    vac = _PM.var(pm, n, :vac, i)

    zc = rc + im*xc
    if reactor
        bc = imag(yc)
        v = 1 # pu, assumption DC approximation
        JuMP.@constraint(pm.model, ppr_fr == -z_DC*bc*(v^2)*(vaf-vac))
        JuMP.@constraint(pm.model, ppr_to == -z_DC*bc*(v^2)*(vac-vaf))
    else
        JuMP.@constraint(pm.model, vac == vaf)
        JuMP.@constraint(pm.model, z_DC*ppr_fr + z_DC*ppr_to  == 0)
    end
end


function constraint_converter_current_dc_ots(pm::_PM.AbstractDCPModel, n::Int, i::Int, Umax, Imax)
    # not used
end

## ACDC switch
function constraint_power_balance_ac_switch(pm::_PM.AbstractDCPModel, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_gens, bus_convs_ac, bus_loads, bus_shunts, pd, qd, gs, bs)
    p = _PM.var(pm, n,  :p)
    pg = _PM.var(pm, n,  :pg)
    pconv_grid_ac = _PM.var(pm, n,  :pconv_tf_fr)
    #pconv_ac = _PM.var(pm, n, :pconv_ac) -> not used in PowerModelsACDC too
    v = 1
    psw  = _PM.var(pm, n, :psw)
    qsw  = _PM.var(pm, n, :qsw)
    print(bus_arcs_sw)
    cstr_p = JuMP.@constraint(pm.model, sum(p[a] for a in bus_arcs) + sum(pconv_grid_ac[c] for c in bus_convs_ac) + sum(psw[sw] for sw in bus_arcs_sw) == sum(pg[g] for g in bus_gens)  - sum(pd[d] for d in bus_loads) - sum(gs[s] for s in bus_shunts)*vm^2)

    if _IM.report_duals(pm)
        _PM.sol(pm, n, :bus, i)[:lam_kcl_r] = cstr_p
    end
end
=#