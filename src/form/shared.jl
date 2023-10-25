function constraint_ohms_ots_dc_branch(pm::_PM.AbstractWRModels, n::Int, f_bus, t_bus, f_idx, t_idx, r, p)
    i, f_bus, t_bus = f_idx
    p_dc_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    p_dc_to = _PM.var(pm, n, :p_dcgrid, t_idx)
    wdc_fr = _PM.var(pm, n, :wdc, f_bus)
    wdc_to = _PM.var(pm, n, :wdc, t_bus)
    wdc_frto = _PM.var(pm, n, :wdcr, (f_bus,t_bus))
    z = _PM.var(pm, n, :z_ots_dc, i)

    if r == 0
        JuMP.@constraint(pm.model, z*p_dc_fr + z*p_dc_to == 0)
    else
        g = 1 / r
        JuMP.@NLconstraint(pm.model, p_dc_fr == z*(p * g * (wdc_fr - wdc_frto)))
        JuMP.@NLconstraint(pm.model, p_dc_to == z*(p * g * (wdc_to - wdc_frto)))
    end
end

function constraint_conv_transformer_dc_ots(pm::_PM.AbstractWRModels, n::Int, i::Int, rtf, xtf, acbus, tm, transformer)
    ptf_fr = _PM.var(pm, n, :pconv_tf_fr, i)
    qtf_fr = _PM.var(pm, n, :qconv_tf_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    w = _PM.var(pm, n,  :w, acbus)  # vm^2
    wf = _PM.var(pm, n,  :wf_ac, i)   # vmf * vmf
    wrf = _PM.var(pm, n,  :wrf_ac, i) # vm*vmf*cos(va-vaf) =  vmf*vm*cos(vaf-va)
    wif = _PM.var(pm, n,  :wif_ac, i) # vm*vmf*sin(va-vaf) = -vmf*vm*sin(vaf-va)


    #vm = _PM.var(pm, n, :vm, acbus)
    #va = _PM.var(pm, n, :va, acbus)
    #vmf = _PM.var(pm, n, :vmf, i)
    #vaf = _PM.var(pm, n, :vaf, i)

    ztf = rtf + im*xtf
    if transformer
        ytf = 1/(rtf + im*xtf)
        gtf = real(ytf)
        btf = imag(ytf)
        c1, c2, c3, c4 = ac_power_flow_constraints_w_dc_ots(pm, gtf, btf, w, wf, wrf, wif, ptf_fr, ptf_to, qtf_fr, qtf_to, tm, z_DC)
        _PMACDC.constraint_voltage_product_converter(pm, wrf, wif, wf, w)
    else
        pcon, qcon = constraint_lossless_section(pm, w, wf, wrf, wif, ptf_fr, ptf_to, qtf_fr, qtf_to)
    end
end

function constraint_conv_filter_dc_ots(pm::_PM.AbstractWModels, n::Int, i::Int, bv, filter) #-> probably not needed
    ppr_fr = _PM.var(pm, n, :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n, :qconv_pr_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    vmf = _PM.var(pm, n, :vmf, i)

    JuMP.@constraint(pm.model,   ppr_fr + ptf_to == 0 )
    JuMP.@NLconstraint(pm.model, qpr_fr + qtf_to +  (-bv) * filter *vmf^2 == 0)
end

function constraint_converter_losses_dc_ots(pm::_PM.AbstractWModels, n::Int, i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@NLconstraint(pm.model, pconv_ac + pconv_dc == z_DC*(a + b*iconv + c*iconv^2))
end

function constraint_conv_reactor_dc_ots(pm::_PM.AbstractWRModels, n::Int, i::Int, rc, xc, reactor)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    qconv_ac = _PM.var(pm, n,  :qconv_ac, i)
    ppr_to = - pconv_ac
    qpr_to = - qconv_ac
    ppr_fr = _PM.var(pm, n,  :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n,  :qconv_pr_fr, i)

    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    wf = _PM.var(pm, n,  :wf_ac, i)
    wc = _PM.var(pm, n,  :wc_ac, i)
    wrc = _PM.var(pm, n,  :wrc_ac, i)
    wic = _PM.var(pm, n,  :wic_ac, i)

    zc = rc + im*xc
    #if reactor
    #    yc = 1/(zc)
    #    gc = real(yc)
    #    bc = imag(yc)
    #    JuMP.@NLconstraint(pm.model, - pconv_ac == z_DC*( gc*vmc^2 + -gc*vmc*vmf*cos(vac-vaf) + -bc*vmc*vmf*sin(vac-vaf))) # JuMP doesn't allow affine expressions in NL constraints
    #    JuMP.@NLconstraint(pm.model, - qconv_ac == z_DC*(-bc*vmc^2 +  bc*vmc*vmf*cos(vac-vaf) + -gc*vmc*vmf*sin(vac-vaf))) # JuMP doesn't allow affine expressions in NL constraints
    #    JuMP.@NLconstraint(pm.model, ppr_fr ==     z_DC*( gc *vmf^2 + -gc *vmf*vmc*cos(vaf - vac) + -bc *vmf*vmc*sin(vaf - vac)))
    #    JuMP.@NLconstraint(pm.model, qpr_fr ==     z_DC*(-bc *vmf^2 +  bc *vmf*vmc*cos(vaf - vac) + -gc *vmf*vmc*sin(vaf - vac)))
    #else
    #    JuMP.@constraint(pm.model, ppr_fr + ppr_to == 0)
    #    JuMP.@constraint(pm.model, qpr_fr + qpr_to == 0)
    #    JuMP.@constraint(pm.model, z_DC*vac == z_DC*vaf)
    #    JuMP.@constraint(pm.model, z_DC*vmc == z_DC*vmf)
    #end

    if reactor
        yc = 1/(zc)
        gc = real(yc)
        bc = imag(yc)
        _PMACDC.constraint_voltage_product_converter(pm, wrc, wic, wf, wc)
        c1, c2, c3, c4 = ac_power_flow_constraints_w_dc_ots(pm, gc, bc, wf, wc, wrc, wic, ppr_fr, ppr_to, qpr_fr, qpr_to, 1, z_DC)
    else
        pcon, qcon = _PMACDC.constraint_lossless_section(pm, wf, wc, wrc, wic, ppr_fr, ppr_to, qpr_fr, qpr_to)
    end

end

#=
function constraint_lossless_section(pm::_PM.AbstractWModels, w_fr, w_to, wr, wi, p_fr, p_to, q_fr, q_to)
    JuMP.@constraint(pm.model, w_fr ==  w_to)
    JuMP.@constraint(pm.model, wr   ==  w_fr)
    JuMP.@constraint(pm.model, wi   ==  0)

    pcon = JuMP.@constraint(pm.model, p_fr + p_to == 0)
    qcon = JuMP.@constraint(pm.model, q_fr + q_to == 0)
    return pcon, qcon
end
=#

"constraints for a voltage magnitude transformer + series impedance"
function ac_power_flow_constraints_w_dc_ots(pm::_PM.AbstractWRModels, g, b, w_fr, w_to, wr, wi, p_fr, p_to, q_fr, q_to, tm, z_c)
    c1 = JuMP.@constraint(pm.model, p_fr == z_c*( g/(tm^2)*w_fr + -g/(tm)*wr + -b/(tm)*wi))
    c2 = JuMP.@constraint(pm.model, q_fr == z_c*(-b/(tm^2)*w_fr +  b/(tm)*wr + -g/(tm)*wi))
    c3 = JuMP.@constraint(pm.model, p_to == z_c*( g*w_to + -g/(tm)*wr     + -b/(tm)*(-wi)))
    c4 = JuMP.@constraint(pm.model, q_to == z_c*(-b*w_to +  b/(tm)*wr     + -g/(tm)*(-wi)))
    return c1, c2, c3, c4
end


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