###################### OTS Constraints ############################
function constraint_voltage_product_converter_ots(pm::_PM.AbstractWRConicModel, wr, wi, w_fr, w_to, z)
    JuMP.@constraint(pm.model, wr >= z*JuMP.lower_bound(wr))
    JuMP.@constraint(pm.model, wr <= z*JuMP.upper_bound(wr))
    JuMP.@constraint(pm.model, wi >= z*JuMP.lower_bound(wi))
    JuMP.@constraint(pm.model, wi <= z*JuMP.upper_bound(wi))
    _PMACDC.relaxation_complex_product_conic_on_off(pm.model, w_fr, w_to, wr, wi, z)
end

function constraint_voltage_product_converter_ots(pm::_PM.AbstractWRModel, wr, wi, w_fr, w_to, z)
    JuMP.@constraint(pm.model, wr >= z*JuMP.lower_bound(wr))
    JuMP.@constraint(pm.model, wr <= z*JuMP.upper_bound(wr))
    JuMP.@constraint(pm.model, wi >= z*JuMP.lower_bound(wi))
    JuMP.@constraint(pm.model, wi <= z*JuMP.upper_bound(wi))
    _IM.relaxation_complex_product_on_off(pm.model, w_fr, w_to, wr, wi, z)
end





"""
Links converter power & current

```
pconv_ac[i]^2 + pconv_dc[i]^2 <= wc[i] * iconv_ac_sq[i]
pconv_ac[i]^2 + pconv_dc[i]^2 <= (Umax)^2 * (iconv_ac[i])^2
```
"""

# Same as PMACDC, consider using thos constraints
function constraint_converter_current_ots(pm::_PM.AbstractWRConicModel, n::Int, i::Int, Umax, Imax)
    wc = _PM.var(pm, n, :wc_ac, i)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    qconv_ac = _PM.var(pm, n, :qconv_ac, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    iconv_sq = _PM.var(pm, n, :iconv_ac_sq, i)
    JuMP.@constraint(pm.model, [wc/sqrt(2), iconv_sq/sqrt(2), pconv_ac, qconv_ac] in JuMP.RotatedSecondOrderCone())
    JuMP.@constraint(pm.model, [Umax * iconv/sqrt(2), Umax * iconv/sqrt(2), pconv_ac, qconv_ac] in JuMP.RotatedSecondOrderCone())
    JuMP.@constraint(pm.model, iconv_sq <= iconv*Imax)
end


function constraint_converter_current_ots(pm::_PM.AbstractWRModel, n::Int, i::Int, Umax, Imax)
    wc = _PM.var(pm, n, :wc_ac, i)
    pconv_ac = _PM.var(pm, n, :pconv_ac, i)
    qconv_ac = _PM.var(pm, n, :qconv_ac, i)
    iconv = _PM.var(pm, n, :iconv_ac, i)
    iconv_sq = _PM.var(pm, n, :iconv_ac_sq, i)

    JuMP.@constraint(pm.model, pconv_ac^2 + qconv_ac^2 <= wc * iconv_sq)
    JuMP.@constraint(pm.model, pconv_ac^2 + qconv_ac^2 <= (Umax)^2 * iconv^2)
    JuMP.@constraint(pm.model,  iconv^2 <= iconv_sq)
    JuMP.@constraint(pm.model, iconv_sq <= iconv*Imax)
end

