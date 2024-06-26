function constraint_converter_losses(pm::_PM.AbstractLPACCModel, n::Int, i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)

    JuMP.@constraint(pm.model, pconv_ac + pconv_dc == a + b*iconv)
end

function constraint_conv_transformer(pm::_PM.AbstractLPACCModel, n::Int, i::Int, rtf, xtf, acbus, tm, transformer)
    ptf_fr = _PM.var(pm, n, :pconv_tf_fr, i)
    qtf_fr = _PM.var(pm, n, :qconv_tf_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)

    phi = _PM.var(pm, n, :phi, acbus)
    phi_vmf = _PM.var(pm, n, :phi_vmf, i)
    va = _PM.var(pm, n, :va, acbus)
    vaf = _PM.var(pm, n, :vaf, i)
    cs = _PM.var(pm, n, :cs_vaf,i)

    ztf = rtf + im*xtf
    if transformer
        ytf = 1/(rtf + im*xtf)
        gtf = real(ytf)
        btf = imag(ytf)
        c1, c2, c3, c4 = lpac_power_flow_constraints(pm.model, gtf, btf, phi, phi_vmf, va, vaf, ptf_fr, ptf_to, qtf_fr, qtf_to, tm, cs)
        c5 = constraint_cos_angle_diff_PWL(pm, n, cs, va, vaf)
    else
        JuMP.@constraint(pm.model, ptf_fr + ptf_to == 0)
        JuMP.@constraint(pm.model, qtf_fr + qtf_to == 0)
        JuMP.@constraint(pm.model, va == vaf)
        JuMP.@constraint(pm.model, (1+phi) == (1+phi_vmf))
    end
end

"constraints for a voltage magnitude transformer + series impedance"

function lpac_power_flow_constraints(model, g, b, phi_fr, phi_to, va_fr, va_to, p_fr, p_to, q_fr, q_to, tm, cs)

    c1 = JuMP.@constraint(model, p_fr ==  g/(tm^2)*(1.0 + 2*phi_fr) + (-g/tm)*(cs + phi_fr + phi_to) + (-b/tm)*(va_fr-va_to))
    c2 = JuMP.@constraint(model, q_fr == -b/(tm^2)*(1.0 + 2*phi_fr) - (-b/tm)*(cs + phi_fr + phi_to) + (-g/tm)*(va_fr-va_to))
    c3 = JuMP.@constraint(model, p_to ==  g*(1.0 + 2*phi_to) + (-g/tm)*(cs + phi_fr + phi_to) + (-b/tm)*-(va_fr-va_to))
    c4 = JuMP.@constraint(model, q_to == -b*(1.0 + 2*phi_to) - (-b/tm)*(cs + phi_fr + phi_to) + (-g/tm)*-(va_fr-va_to))
    return c1, c2, c3, c4
end


function constraint_conv_reactor(pm::_PM.AbstractLPACCModel, n::Int, i::Int, rc, xc, reactor)
    pconv_ac  = _PM.var(pm, n, :pconv_ac, i)
    qconv_ac  = _PM.var(pm, n, :qconv_ac, i)
    ppr_to = - pconv_ac
    qpr_to = - qconv_ac
    ppr_fr = _PM.var(pm, n, :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n, :qconv_pr_fr, i)

    phi_vmc = _PM.var(pm, n, :phi_vmc, i)
    phi_vmf = _PM.var(pm, n, :phi_vmf, i)
    vac = _PM.var(pm, n, :vac, i)
    vaf = _PM.var(pm, n, :vaf, i)
    cs = _PM.var(pm, n, :cs_vac,i)

    phi_vmc_ub = JuMP.upper_bound(phi_vmc)
    ppr_to_ub = JuMP.upper_bound(_PM.var(pm, n)[:pconv_ac][i])
    qpr_to_ub = JuMP.upper_bound(_PM.var(pm, n)[:qconv_ac][i])
    Smax = sqrt(ppr_to_ub^2 + qpr_to_ub^2)
    zc = rc + im*xc
    if reactor
        yc = 1/(zc)
        gc = real(yc)
        bc = imag(yc)
        c1, c2, c3, c4 = lpac_power_flow_constraints(pm.model, gc, bc, phi_vmf, phi_vmc, vaf, vac, ppr_fr, ppr_to, qpr_fr, qpr_to, 1, cs)
        c5 = constraint_cos_angle_diff_PWL(pm, n, cs, vaf, vac)
        c6 = constraint_conv_capacity_PWL(pm, n, ppr_to, qpr_to, ppr_to_ub, qpr_to_ub, Smax)
   else
        JuMP.@constraint(pm.model, ppr_fr + ppr_to == 0)
        JuMP.@constraint(pm.model, qpr_fr + qpr_to == 0)
        JuMP.@constraint(pm.model, vac == vaf)
        JuMP.@constraint(pm.model, (1+phi_vmf) == (1+phi_vmc))

    end
end

function constraint_conv_filter(pm::_PM.AbstractLPACCModel, n::Int, i::Int, bv, filter)
    ppr_fr = _PM.var(pm, n, :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n, :qconv_pr_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    phi_vmf = _PM.var(pm, n, :phi_vmf, i)

    JuMP.@constraint(pm.model, ppr_fr + ptf_to == 0 )
    JuMP.@constraint(pm.model, qpr_fr + qtf_to + -bv*filter*(1+2*phi_vmf) == 0)
end


function constraint_converter_current(pm::_PM.AbstractLPACCModel, n::Int, i::Int, Umax, Imax)
    phi_vmc = _PM.var(pm, n, :phi_vmc, i)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    qconv_ac = _PM.var(pm, n, :qconv_ac, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)

    JuMP.@constraint(pm.model, iconv <= Imax)
end


function variable_converter_filter_voltage(pm::_PM.AbstractLPACCModel; kwargs...)
    variable_converter_filter_voltage_magnitude(pm; kwargs...)
    variable_converter_filter_voltage_angle_cs(pm; kwargs...)
    variable_converter_filter_voltage_angle(pm; kwargs...)
end

function variable_converter_internal_voltage(pm::_PM.AbstractLPACCModel; kwargs...)
    variable_converter_internal_voltage_magnitude(pm; kwargs...)
    variable_converter_internal_voltage_angle_cs(pm; kwargs...)
    variable_converter_internal_voltage_angle(pm; kwargs...)
end

function variable_converter_filter_voltage_angle_cs(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded = true, report = true)
    csvaf = _PM.var(pm, nw)[:cs_vaf] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_cs_vaf",
    start = 0
    )
    if bounded
        for (c, convdc) in _PM.ref(pm, nw, :convdc)
            JuMP.set_lower_bound(csvaf[c],  0)
            JuMP.set_upper_bound(csvaf[c],  1)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :convdc, :cs_vaf, _PM.ids(pm, nw, :convdc), csvaf)
end

function variable_converter_internal_voltage_angle_cs(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded = true, report = true)
    csvac = _PM.var(pm, nw)[:cs_vac] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_cs_vac",
    start = 0
    )

    if bounded
        for (c, convdc) in _PM.ref(pm, nw, :convdc)
            JuMP.set_lower_bound(csvac[c],  0)
            JuMP.set_upper_bound(csvac[c],  1)
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :convdc, :cs_vac, _PM.ids(pm, nw, :convdc), csvac)
end

function variable_converter_filter_voltage_magnitude(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded = true, report = true)
    phivmf = _PM.var(pm, nw)[:phi_vmf] = JuMP.@variable(pm.model,
            [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_phi_vmf",
            start = _PM.comp_start_value(_PM.ref(pm, nw, :convdc, i), "phi_start")
        )

        if bounded
            for (c, convdc) in _PM.ref(pm, nw, :convdc)
                JuMP.set_lower_bound(phivmf[c],  convdc["Vmmin"] - 1.0)
                JuMP.set_upper_bound(phivmf[c],  convdc["Vmmax"] - 1.0)
            end
        end

        report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :convdc, :phi_vmf, _PM.ids(pm, nw, :convdc), phivmf)
end

function variable_converter_internal_voltage_magnitude(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded = true, report = true)
    phivmc = _PM.var(pm, nw)[:phi_vmc] = JuMP.@variable(pm.model,
        [i in _PM.ids(pm, nw, :convdc)], base_name="$(nw)_phi_vmc",
        start = _PM.comp_start_value(_PM.ref(pm, nw, :convdc, i), "phi_start")
        )

        if bounded
            for (c, convdc) in _PM.ref(pm, nw, :convdc)
                JuMP.set_lower_bound(phivmc[c],  convdc["Vmmin"] - 1.0)
                JuMP.set_upper_bound(phivmc[c],  convdc["Vmmax"] - 1.0)
            end
        end

        report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :convdc, :phi_vmc, _PM.ids(pm, nw, :convdc), phivmc)
end

function constraint_conv_capacity_PWL(pm::_PM.AbstractLPACCModel, n::Int, ppr_to, qpr_to, Umax, Imax, Smax)
    np = 20 #no. of segments, can be passed as an argument later
    l = 0
    for i = 1:np
        a= Smax*sin(l)
        b = Smax*cos(l)
        c6 = JuMP.@constraint(pm.model, a*ppr_to + b*qpr_to <= Smax^2) #current and voltage bounds to be proper to use Umax*Imax because Umax*Imax == Smax
        l = l + 2*pi/np
    end
end

function constraint_cos_angle_diff_PWL(pm::_PM.AbstractLPACCModel, n::Int, cs, va_fr, va_to)
    nb = 20 #no. of segments, can be passed as an argument later
    l = -pi/6
    h = pi/6
    inc = (h-l)/(nb+1)
    a = l + inc
    diff = va_fr - va_to
    for i = 1:nb
        c5 = JuMP.@constraint(pm.model, cs <= -sin(a)*(diff-a) + cos(a))
        a = a + inc
    end
end

function add_dcconverter_voltage_setpoint(sol, pm::_PM.AbstractLPACCModel)
    _PM.add_setpoint!(sol, pm, "convdc", "vmconv", :phi_vmc, status_name="islcc", inactive_status_value = 4, scale = (x,item,cnd) -> 1.0+x)
    _PM.add_setpoint!(sol, pm, "convdc", "vmfilt", :phi_vmf, status_name="islcc", inactive_status_value = 4, scale = (x,item,cnd) -> 1.0+x)
    _PM.add_setpoint!(sol, pm, "convdc", "vaconv", :vac, status_name="islcc", inactive_status_value = 4)
    _PM.add_setpoint!(sol, pm, "convdc", "vafilt", :vaf, status_name="islcc", inactive_status_value = 4)
end

function variable_voltage_slack_ots(pm::_PM.AbstractLPACCModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=false)
end

function lpac_power_flow_constraints_dc_ots(model, g, b, phi_fr, phi_to, va_fr, va_to, p_fr, p_to, q_fr, q_to, tm, cs, z_DC)
    c1 = JuMP.@constraint(model, p_fr == z_DC*( g/(tm^2)*(1.0 + 2*phi_fr) + (-g/tm)*(cs + phi_fr + phi_to) + (-b/tm)*(va_fr-va_to)))
    c2 = JuMP.@constraint(model, q_fr == z_DC*(-b/(tm^2)*(1.0 + 2*phi_fr) - (-b/tm)*(cs + phi_fr + phi_to) + (-g/tm)*(va_fr-va_to)))
    c3 = JuMP.@constraint(model, p_to == z_DC*( g*(1.0 + 2*phi_to) + (-g/tm)*(cs + phi_fr + phi_to) + (-b/tm)*-(va_fr-va_to)))
    c4 = JuMP.@constraint(model, q_to == z_DC*(-b*(1.0 + 2*phi_to) - (-b/tm)*(cs + phi_fr + phi_to) + (-g/tm)*-(va_fr-va_to)))
    return c1, c2, c3, c4
end

function constraint_conv_transformer_dc_ots(pm::_PM.AbstractLPACCModel, n::Int, i::Int, rtf, xtf, acbus, tm, transformer)
    ptf_fr = _PM.var(pm, n, :pconv_tf_fr, i)
    qtf_fr = _PM.var(pm, n, :qconv_tf_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    phi = _PM.var(pm, n, :phi, acbus)
    phi_vmf = _PM.var(pm, n, :phi_vmf, i)
    va = _PM.var(pm, n, :va, acbus)
    vaf = _PM.var(pm, n, :vaf, i)
    cs = _PM.var(pm, n, :cs_vaf,i)

    ztf = rtf + im*xtf
    if transformer
        ytf = 1/(rtf + im*xtf)
        gtf = real(ytf)
        btf = imag(ytf)
        c1, c2, c3, c4 = lpac_power_flow_constraints_dc_ots(pm.model, gtf, btf, phi, phi_vmf, va, vaf, ptf_fr, ptf_to, qtf_fr, qtf_to, tm, cs, z_DC)
        c5 = _PMACDC.constraint_cos_angle_diff_PWL(pm, n, cs, va, vaf)
    else
        JuMP.@constraint(pm.model, ptf_fr + ptf_to == 0)
        JuMP.@constraint(pm.model, qtf_fr + qtf_to == 0)
        JuMP.@constraint(pm.model, va == vaf)
        JuMP.@constraint(pm.model, (1+phi) == (1+phi_vmf))
    end
end

function constraint_conv_reactor_dc_ots(pm::_PM.AbstractLPACCModel, n::Int, i::Int, rc, xc, reactor)
    pconv_ac  = _PM.var(pm, n, :pconv_ac, i)
    qconv_ac  = _PM.var(pm, n, :qconv_ac, i)
    ppr_to = - pconv_ac
    qpr_to = - qconv_ac
    ppr_fr = _PM.var(pm, n, :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n, :qconv_pr_fr, i)

    phi_vmc = _PM.var(pm, n, :phi_vmc, i)
    phi_vmf = _PM.var(pm, n, :phi_vmf, i)
    vac = _PM.var(pm, n, :vac, i)
    vaf = _PM.var(pm, n, :vaf, i)
    cs = _PM.var(pm, n, :cs_vac,i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    phi_vmc_ub = JuMP.upper_bound(phi_vmc)
    ppr_to_ub = JuMP.upper_bound(_PM.var(pm, n)[:pconv_ac][i])
    qpr_to_ub = JuMP.upper_bound(_PM.var(pm, n)[:qconv_ac][i])
    Smax = sqrt(ppr_to_ub^2 + qpr_to_ub^2)
    zc = rc + im*xc
    if reactor
        yc = 1/(zc)
        gc = real(yc)
        bc = imag(yc)
        c1, c2, c3, c4 = lpac_power_flow_constraints(pm.model, gc, bc, phi_vmf, phi_vmc, vaf, vac, ppr_fr, ppr_to, qpr_fr, qpr_to, 1, cs)
        c5 = constraint_cos_angle_diff_PWL(pm, n, cs, vaf, vac)
        c6 = constraint_conv_capacity_PWL(pm, n, ppr_to, qpr_to, ppr_to_ub, qpr_to_ub, Smax)
   else
        JuMP.@constraint(pm.model, ppr_fr + ppr_to == 0)
        JuMP.@constraint(pm.model, qpr_fr + qpr_to == 0)
        JuMP.@constraint(pm.model, z_DC*vac == z_DC*vaf)
        JuMP.@constraint(pm.model, z_DC*(1+phi_vmf) == z_DC*(1+phi_vmc))

    end
end

function constraint_conv_firing_angle(pm::_PM.AbstractLPACCModel, n::Int, i::Int, S, P1, Q1, P2, Q2)
    p = _PM.var(pm, n, :pconv_ac, i)
    q = _PM.var(pm, n, :qconv_ac, i)
    phi = _PM.var(pm, n, :phiconv, i)

    JuMP.@constraint(pm.model, p == cos(phi) * S)
    JuMP.@constraint(pm.model, q == sin(phi) * S)
end

function constraint_converter_current_ots(pm::_PM.AbstractLPACCModel, n::Int, i::Int, Umax, Imax)
    phi_vmc = _PM.var(pm, n, :phi_vmc, i)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    qconv_ac = _PM.var(pm, n, :qconv_ac, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    z_dc = _PM.var(pm, n, :z_conv_dc, i)

    #JuMP.@NLconstraint(pm.model, pconv_ac^2 + qconv_ac^2 == z_dc * phi_vmc^2 * iconv^2)
    JuMP.@constraint(pm.model, iconv <= Imax)
end

function constraint_converter_losses_dc_ots(pm::_PM.AbstractLPACCModel, n::Int, i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@constraint(pm.model, pconv_ac + pconv_dc == z_DC*(a + b*iconv))
end


function constraint_conv_filter_dc_ots(pm::_PM.AbstractLPACCModel, n::Int, i::Int, bv, filter) #-> probably not needed
    ppr_fr = _PM.var(pm, n, :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n, :qconv_pr_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    phi_vmf = _PM.var(pm, n, :phi_vmf, i)

    JuMP.@constraint(pm.model, ppr_fr + ptf_to == 0 )
    JuMP.@constraint(pm.model, qpr_fr + qtf_to + -bv*filter*(1+2*phi_vmf) == 0)
end