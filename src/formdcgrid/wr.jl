

function constraint_voltage_dc_ots(pm::_PM.AbstractWRModel; nw::Int = _PM.nw_id_default)
    wdc = _PM.var(pm, nw, :wdc)
    wdcr = _PM.var(pm, nw, :wdcr)
    z_dc = _PM.var(pm, nw, :z_ots_dc)

    for (i,j) in _PM.ids(pm, nw, :buspairsdc)
        JuMP.@constraint(pm.model, wdcr[(i,j)]^2 <= wdc[i]*wdc[j])
    end
end

function constraint_voltage_dc_ots(pm::_PM.AbstractWRConicModel; nw::Int = _PM.nw_id_default)
    wdc = _PM.var(pm, nw, :wdc)
    wdcr = _PM.var(pm, nw, :wdcr)

    for (i,j) in _PM.ids(pm, nw, :buspairsdc)
        relaxation_complex_product_conic(pm.model, wdc[i], wdc[j], wdcr[(i,j)])
    end
end
