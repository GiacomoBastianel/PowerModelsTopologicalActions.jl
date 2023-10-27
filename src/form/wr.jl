function constraint_voltage_product_converter(pm::_PM.AbstractWRModel, wr, wi, w_fr, w_to)
    InfrastructureModels.relaxation_complex_product(pm.model, w_fr, w_to, wr, wi)
end

function constraint_voltage_product_converter(pm::_PM.AbstractWRConicModel, wr, wi, w_fr, w_to)
    InfrastructureModels.relaxation_complex_product_conic(pm.model, w_fr, w_to, wr, wi)
end


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
