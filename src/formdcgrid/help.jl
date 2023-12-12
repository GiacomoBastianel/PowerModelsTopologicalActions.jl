"formconv/wr.jl"

function constraint_voltage_product_converter(pm::_PM.AbstractWRModel, wr, wi, w_fr, w_to)
    InfrastructureModels.relaxation_complex_product(pm.model, w_fr, w_to, wr, wi)
end

function constraint_voltage_product_converter(pm::_PM.AbstractWRConicModel, wr, wi, w_fr, w_to)
    InfrastructureModels.relaxation_complex_product_conic(pm.model, w_fr, w_to, wr, wi)
end
"""
Links converter power & current

```
pconv_ac[i]^2 + pconv_dc[i]^2 <= wc[i] * iconv_ac_sq[i]
pconv_ac[i]^2 + pconv_dc[i]^2 <= (Umax)^2 * (iconv_ac[i])^2
```
"""
function constraint_converter_current(pm::_PM.AbstractWRModel, n::Int, i::Int, Umax, Imax)
    wc = _PM.var(pm, n,  :wc_ac, i)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    qconv_ac = _PM.var(pm, n,  :qconv_ac, i)
    iconv = _PM.var(pm, n,  :iconv_ac, i)
    iconv_sq = _PM.var(pm, n,  :iconv_ac_sq, i)

    JuMP.@constraint(pm.model, pconv_ac^2 + qconv_ac^2 <= wc * iconv_sq)
    JuMP.@constraint(pm.model, pconv_ac^2 + qconv_ac^2 <= (Umax)^2 * iconv^2)
    JuMP.@constraint(pm.model, iconv^2 <= iconv_sq)
    JuMP.@constraint(pm.model, iconv_sq <= iconv*Imax)
end

function constraint_converter_current(pm::_PM.AbstractWRConicModel, n::Int,  i::Int, Umax, Imax)
    wc = _PM.var(pm, n,  :wc_ac, i)
    pconv_ac = _PM.var(pm, n,  :pconv_ac, i)
    qconv_ac = _PM.var(pm, n,  :qconv_ac, i)
    iconv = _PM.var(pm, n,  :iconv_ac, i)
    iconv_sq = _PM.var(pm, n,  :iconv_ac_sq, i)

    JuMP.@constraint(pm.model, [wc/sqrt(2), iconv_sq/sqrt(2), pconv_ac, qconv_ac] in JuMP.RotatedSecondOrderCone())
    JuMP.@constraint(pm.model, [Umax * iconv/sqrt(2), Umax * iconv/sqrt(2), pconv_ac, qconv_ac] in JuMP.RotatedSecondOrderCone())
    JuMP.@constraint(pm.model, iconv_sq <= iconv*Imax)
end


"formdcgrid/wr.jl"

"""
Model to approximate cross products of node voltages

```
wdcr[(i,j)] <= wdc[i]*wdc[j]
```
"""
function constraint_voltage_dc(pm::_PM.AbstractWRModel; nw::Int = _PM.nw_id_default)
    wdc = _PM.var(pm, nw, :wdc)
    wdcr = _PM.var(pm, nw, :wdcr)

    for (i,j) in _PM.ids(pm, nw, :buspairsdc)
        JuMP.@constraint(pm.model, wdcr[(i,j)]^2 <= wdc[i]*wdc[j])
    end
end

function constraint_voltage_dc(pm::_PM.AbstractWRConicModel; nw::Int = _PM.nw_id_default)
    wdc = _PM.var(pm, nw, :wdc)
    wdcr = _PM.var(pm, nw, :wdcr)

    for (i,j) in _PM.ids(pm, nw, :buspairsdc)
        relaxation_complex_product_conic(pm.model, wdc[i], wdc[j], wdcr[(i,j)])
    end
end

"""
Limits dc branch current

```
p[f_idx] <= wdc[f_bus] * Imax
```
"""
function constraint_dc_branch_current(pm::_PM.AbstractWRModel, n::Int, f_bus, f_idx, ccm_max, p)
    p_dc_fr = _PM.var(pm, n, :p_dcgrid, f_idx)
    wdc_fr = _PM.var(pm, n, :wdc, f_bus)

    JuMP.@constraint(pm.model, p_dc_fr <= wdc_fr * ccm_max * p^2)
end

