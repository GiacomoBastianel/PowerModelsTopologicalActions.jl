
## DC OTS -> to be modified if we want to add the OTS on the converter side, needed?
function constraint_conv_transformer_dc_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, rtf, xtf, acbus, tm, transformer)
    ptf_fr = _PM.var(pm, n, :pconv_tf_fr, i)
    qtf_fr = _PM.var(pm, n, :qconv_tf_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    vm = _PM.var(pm, n, :vm, acbus)
    va = _PM.var(pm, n, :va, acbus)
    vmf = _PM.var(pm, n, :vmf, i)
    vaf = _PM.var(pm, n, :vaf, i)

    ztf = rtf + im*xtf
    if transformer
        ytf = 1/(rtf + im*xtf)
        gtf = real(ytf)
        btf = imag(ytf)
        gtf_sh = 0
        c1, c2, c3, c4 = ac_power_flow_constraints_dc_ots(pm.model, gtf, btf, gtf_sh, vm, vmf, va, vaf, ptf_fr, ptf_to, qtf_fr, qtf_to, tm, z_DC)
    else
        JuMP.@constraint(pm.model, ptf_fr + ptf_to == 0)
        JuMP.@constraint(pm.model, qtf_fr + qtf_to == 0)
        JuMP.@constraint(pm.model, va == vaf)
        JuMP.@constraint(pm.model, vm == vmf)
    end
end

function ac_power_flow_constraints_dc_ots(model, g, b, gsh_fr, vm_fr, vm_to, va_fr, va_to, p_fr, p_to, q_fr, q_to, tm, z_DC)
    c1 = JuMP.@NLconstraint(model, p_fr == z_DC*( g/(tm^2)*vm_fr^2 + -g/(tm)*vm_fr*vm_to * cos(va_fr-va_to) + -b/(tm)*vm_fr*vm_to*sin(va_fr-va_to)))
    c2 = JuMP.@NLconstraint(model, q_fr == z_DC*(-b/(tm^2)*vm_fr^2 +  b/(tm)*vm_fr*vm_to * cos(va_fr-va_to) + -g/(tm)*vm_fr*vm_to*sin(va_fr-va_to)))
    c3 = JuMP.@NLconstraint(model, p_to == z_DC*( g*vm_to^2 + -g/(tm)*vm_to*vm_fr  *    cos(va_to - va_fr)  + -b/(tm)*vm_to*vm_fr*sin(va_to - va_fr)))
    c4 = JuMP.@NLconstraint(model, q_to == z_DC*(-b*vm_to^2 +  b/(tm)*vm_to*vm_fr  *    cos(va_to - va_fr)  + -g/(tm)*vm_to*vm_fr*sin(va_to - va_fr)))
    return c1, c2, c3, c4
end

function constraint_conv_filter_dc_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, bv, filter) #-> probably not needed
    ppr_fr = _PM.var(pm, n, :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n, :qconv_pr_fr, i)
    ptf_to = _PM.var(pm, n, :pconv_tf_to, i)
    qtf_to = _PM.var(pm, n, :qconv_tf_to, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    vmf = _PM.var(pm, n, :vmf, i)

    JuMP.@constraint(pm.model,   ppr_fr + ptf_to == 0 )
    JuMP.@NLconstraint(pm.model, qpr_fr + qtf_to +  (-bv) * filter *vmf^2 == 0)
end

function constraint_converter_losses_dc_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, a, b, c, plmax)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    pconv_dc = _PM.var(pm, n, :pconv_dc, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@NLconstraint(pm.model, pconv_ac + pconv_dc == z_DC*(a + b*iconv + c*iconv^2))
end

function constraint_conv_reactor_dc_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, rc, xc, reactor)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    qconv_ac = _PM.var(pm, n,  :qconv_ac, i)
    ppr_to = - pconv_ac
    qpr_to = - qconv_ac
    ppr_fr = _PM.var(pm, n,  :pconv_pr_fr, i)
    qpr_fr = _PM.var(pm, n,  :qconv_pr_fr, i)

    z_DC = _PM.var(pm, n, :z_conv_dc, i)

    vmf = _PM.var(pm, n, :vmf, i)
    vaf = _PM.var(pm, n, :vaf, i)
    vmc = _PM.var(pm, n, :vmc, i)
    vac = _PM.var(pm, n, :vac, i)

    zc = rc + im*xc
    if reactor
        yc = 1/(zc)
        gc = real(yc)
        bc = imag(yc)
        JuMP.@NLconstraint(pm.model, - pconv_ac == z_DC*( gc*vmc^2 + -gc*vmc*vmf*cos(vac-vaf) + -bc*vmc*vmf*sin(vac-vaf))) # JuMP doesn't allow affine expressions in NL constraints
        JuMP.@NLconstraint(pm.model, - qconv_ac == z_DC*(-bc*vmc^2 +  bc*vmc*vmf*cos(vac-vaf) + -gc*vmc*vmf*sin(vac-vaf))) # JuMP doesn't allow affine expressions in NL constraints
        JuMP.@NLconstraint(pm.model, ppr_fr ==     z_DC*( gc *vmf^2 + -gc *vmf*vmc*cos(vaf - vac) + -bc *vmf*vmc*sin(vaf - vac)))
        JuMP.@NLconstraint(pm.model, qpr_fr ==     z_DC*(-bc *vmf^2 +  bc *vmf*vmc*cos(vaf - vac) + -gc *vmf*vmc*sin(vaf - vac)))
    else
        JuMP.@constraint(pm.model, ppr_fr + ppr_to == 0)
        JuMP.@constraint(pm.model, qpr_fr + qpr_to == 0)
        JuMP.@constraint(pm.model, z_DC*vac == z_DC*vaf)
        JuMP.@constraint(pm.model, z_DC*vmc == z_DC*vmf)
    end
end

function constraint_conv_firing_angle(pm::_PM.AbstractACPModel, n::Int, i::Int, S, P1, Q1, P2, Q2)
    p = _PM.var(pm, n, :pconv_ac, i)
    q = _PM.var(pm, n, :qconv_ac, i)
    phi = _PM.var(pm, n, :phiconv, i)

    JuMP.@NLconstraint(pm.model, p == cos(phi) * S)
    JuMP.@NLconstraint(pm.model, q == sin(phi) * S)
end

function constraint_converter_current_ots(pm::_PM.AbstractACPModel, n::Int, i::Int, Umax, Imax)
    vmc = _PM.var(pm, n, :vmc, i)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    qconv_ac = _PM.var(pm, n, :qconv_ac, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    z_dc = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@NLconstraint(pm.model, pconv_ac^2 + qconv_ac^2 == z_dc * vmc^2 * iconv^2)
end
