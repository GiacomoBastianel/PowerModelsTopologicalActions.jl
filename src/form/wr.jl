function constraint_voltage_product_converter(pm::_PM.AbstractWRModel, wr, wi, w_fr, w_to)
    InfrastructureModels.relaxation_complex_product(pm.model, w_fr, w_to, wr, wi)
end

function constraint_voltage_product_converter(pm::_PM.AbstractWRConicModel, wr, wi, w_fr, w_to)
    InfrastructureModels.relaxation_complex_product_conic(pm.model, w_fr, w_to, wr, wi)
end


#=
function constraint_ohms_ots_dc_branch(pm::_PM.AbstractWRConicModel, n::Int, f_bus, t_bus, f_idx, t_idx, r, p)
    i, f_bus, t_bus = f_idx
    p_dc_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    p_dc_to = _PM.var(pm, n, :p_dcgrid, t_idx)
    wdc_fr = _PM.var(pm, n, :wdc, f_bus)
    wdc_to = _PM.var(pm, n, :wdc, t_bus)
    wdc_frto = _PM.var(pm, n, :wdcr, (f_bus, t_bus))
    z = _PM.var(pm, n, :z_ots_dc, i)

    if r == 0
        JuMP.@constraint(pm.model, z*p_dc_fr + z*p_dc_to == 0)
    else
        g = 1 / r
        JuMP.@NLconstraint(pm.model, p_dc_fr == z*(p * g * (wdc_fr - wdc_frto)))
        JuMP.@NLconstraint(pm.model, p_dc_to == z*(p * g * (wdc_to - wdc_frto)))
    end
end
=#
function constraint_switch_voltage_on_off(pm::_PM.AbstractWRModel, n::Int, i, f_bus, t_bus, vad_min, vad_max)
    vm_fr = _PM.var(pm, n, :vm, f_bus)
    vm_to = _PM.var(pm, n, :vm, t_bus)
    va_fr = _PM.var(pm, n, :va, f_bus)
    va_to = _PM.var(pm, n, :va, t_bus)
    z = _PM.var(pm, n, :z_switch, i)

    JuMP.@constraint(pm.model, z*vm_fr == z*vm_to)
    JuMP.@constraint(pm.model, z*va_fr == z*va_to)

    #JuMP.@constraint(pm.model, (vm_fr - vm_to) <= 100*(1 - z))
    #JuMP.@constraint(pm.model, (va_fr - va_to) <= 100*(1 - z))
    #JuMP.@constraint(pm.model, - 100*(1 - z) <= (vm_fr - vm_to))
    #JuMP.@constraint(pm.model, - 100*(1 - z) <= (va_fr - va_to))
end

function constraint_dc_switch_state_closed(pm::_PM.AbstractWRModel, n::Int, f_busdc, t_busdc)
    vm_fr = _PM.var(pm, n, :vdcm, f_busdc)
    vm_to = _PM.var(pm, n, :vdcm, t_busdc)

    JuMP.@constraint(pm.model, vm_fr == vm_to)
end

function constraint_dc_switch_voltage_on_off(pm::_PM.AbstractWRModel, n::Int, i, f_busdc, t_busdc)
    vm_fr = _PM.var(pm, n, :vdcm, f_busdc)
    vm_to = _PM.var(pm, n, :vdcm, t_busdc)
    z = _PM.var(pm, n, :z_dcswitch, i)

    JuMP.@constraint(pm.model, z*vm_fr == z*vm_to)
end

## DC OTS -> to be modified if we want to add the OTS on the converter side, needed?

#function constraint_conv_firing_angle(pm::_PM.AbstractWRModel, n::Int, i::Int, S, P1, Q1, P2, Q2)
#    p = _PM.var(pm, n, :pconv_ac, i)
#    q = _PM.var(pm, n, :qconv_ac, i)
#    phi = _PM.var(pm, n, :phiconv, i)
#
#    JuMP.@NLconstraint(pm.model, p == cos(phi) * S)
#    JuMP.@NLconstraint(pm.model, q == sin(phi) * S)
#end

function constraint_converter_current_dc_ots(pm::_PM.AbstractWRModel, n::Int, i::Int, Umax, Imax)
    wc = _PM.var(pm, n,  :wc_ac, i)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    qconv_ac = _PM.var(pm, n,  :qconv_ac, i)
    iconv = _PM.var(pm, n,  :iconv_ac, i)
    iconv_sq = _PM.var(pm, n,  :iconv_ac_sq, i)
    z_dc = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@NLconstraint(pm.model, pconv_ac^2 + qconv_ac^2 <= z_dc * wc * iconv_sq)
    JuMP.@NLconstraint(pm.model, pconv_ac^2 + qconv_ac^2 <= z_dc * (Umax)^2 * iconv^2)
    JuMP.@NLconstraint(pm.model, iconv^2 <= iconv_sq)
    JuMP.@NLconstraint(pm.model, iconv_sq <= z_dc * iconv*Imax)
end

function constraint_converter_current_dc_ots(pm::_PM.AbstractWRConicModel, n::Int,  i::Int, Umax, Imax)
    wc = _PM.var(pm, n,  :wc_ac, i)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    qconv_ac = _PM.var(pm, n,  :qconv_ac, i)
    iconv = _PM.var(pm, n,  :iconv_ac, i)
    iconv_sq = _PM.var(pm, n,  :iconv_ac_sq, i)
    z_dc = _PM.var(pm, n, :z_conv_dc, i)

    JuMP.@constraint(pm.model, [z_dc * wc/sqrt(2), z_dc * iconv_sq/sqrt(2), pconv_ac, qconv_ac] in JuMP.RotatedSecondOrderCone())
    JuMP.@constraint(pm.model, [z_dc * Umax * iconv/sqrt(2), z_dc * Umax * iconv/sqrt(2), pconv_ac, qconv_ac] in JuMP.RotatedSecondOrderCone())
    JuMP.@constraint(pm.model, iconv_sq <= z_dc * iconv*Imax)
end

#=
function constraint_voltage_dc_ots(pm::_PM.AbstractWRModel; nw::Int = _PM.nw_id_default)
    wdc = _PM.var(pm, nw, :wdc)
    wdcr = _PM.var(pm, nw, :wdcr)
    z = _PM.var(pm, n, :z_ots_dc)#, i)

    for (i,j) in _PM.ids(pm, nw, :buspairsdc)
        JuMP.@constraint(pm.model, wdcr[(i,j)]^2 <= z[i]*wdc[i]*wdc[j])
    end
end

function constraint_voltage_dc_ots(pm::_PM.AbstractWRConicModel; nw::Int = _PM.nw_id_default)
    wdc = _PM.var(pm, nw, :wdc)
    wdcr = _PM.var(pm, nw, :wdcr)
    z = _PM.var(pm, n, :z_ots_dc, i)

    for (i,j) in _PM.ids(pm, nw, :buspairsdc)
        relaxation_complex_product_conic(pm.model, wdc[i], wdc[j], wdcr[(i,j)])
    end
end
=#