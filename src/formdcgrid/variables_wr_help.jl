"variable: `vdcm[i]` for `i` in `dcbus`es"
function variable_dcgrid_voltage_magnitude_sqr(pm::_PM.AbstractPowerModel; nw::Int=_PM.nw_id_default, bounded::Bool = true, report::Bool=true)
    wdc = _PM.var(pm, nw)[:wdc] = JuMP.@variable(pm.model,
    [i in _PM.ids(pm, nw, :busdc)], base_name="$(nw)_wdc",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, i), "Vdc", 1.0)^2
    )
    wdcr = _PM.var(pm, nw)[:wdcr] = JuMP.@variable(pm.model,
    [(i,j) in _PM.ids(pm, nw, :buspairsdc)], base_name="$(nw)_wdcr",
    start = _PM.comp_start_value(_PM.ref(pm, nw, :busdc, i), "Vdc", 1.0)^2
    )

    if bounded
        for (i, busdc) in _PM.ref(pm, nw, :busdc)
            JuMP.set_lower_bound(wdc[i],  busdc["Vdcmin"]^2)
            JuMP.set_upper_bound(wdc[i],  busdc["Vdcmax"]^2)
        end
        for (bp, buspairdc) in _PM.ref(pm, nw, :buspairsdc)
            JuMP.set_lower_bound(wdcr[bp],  0)
            JuMP.set_upper_bound(wdcr[bp],  buspairdc["vm_fr_max"] * buspairdc["vm_to_max"])
        end
    end

    report && _IM.sol_component_value(pm, _PM.pm_it_sym, nw, :busdc, :wdc, _PM.ids(pm, nw, :busdc), wdc)
end


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

function constraint_voltage_product_converter(pm::_PM.AbstractWRModel, wr, wi, w_fr, w_to)
    InfrastructureModels.relaxation_complex_product(pm.model, w_fr, w_to, wr, wi)
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



# This one is not influenced by the model
function constraint_branch_limit_on_off(pm::_PM.AbstractWRModels, n::Int, i, f_idx, t_idx, pmax, pmin, imax, imin)
    p_fr = _PM.var(pm, n, :p_dcgrid_ne)[f_idx]
    p_to = _PM.var(pm, n, :p_dcgrid_ne)[t_idx]
    z = _PM.var(pm, n, :branchdc_ne)[i]
    JuMP.@constraint(pm.model,  p_fr <= pmax * z)
    JuMP.@constraint(pm.model,  p_fr >= pmin * z)
    JuMP.@constraint(pm.model,  p_to <= pmax * z)
    JuMP.@constraint(pm.model,  p_to >= pmin * z)
end